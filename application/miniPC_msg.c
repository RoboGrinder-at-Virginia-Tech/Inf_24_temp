/**
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  * @file       miniPC_msg.c/h
  * @brief      miniPC_msg.  miniPC message related files
  * @note       message information about miniPC <-> TypeC board communication(Tx and Rx)
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-16-2023     Zelin Shen      basic comminication functions
	*
  *
  @verbatim
  ==============================================================================
	This .c and .h files include specific raw data package & data after process if needed
	File dependency: this file is used with miniPC_comm_task, fifo data structure file, 
	RM CRC8_CRC16 files, lower level USART drivers
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  */
	
#include "miniPC_msg.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "miniPC_comm_task.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "detect_task.h"
#include "SuperCap_comm.h"
#include "referee.h"
#include "arm_math.h"

extern pc_comm_unpack_data_t pc_comm_unpack_data_obj;

//包头实体, frame head
pc_comm_frame_header_t pc_comm_receive_header; //this means embed receive msg from miniPC
pc_comm_embed_send_header_t pc_send_header; //embed send msg from miniPC
embed_send_protocol_t embed_send_protocol;

//消息实体 原始数据, raw origial data
//miniPC -> Embedded
pc_ui_msg_t pc_ui_msg;
pc_cmd_chassis_control_t pc_cmd_chassis_control;
pc_cmd_gimbal_ctrl_t pc_cmd_gimbal_ctrl_aid;
pc_cmd_gimbal_ctrl_t pc_cmd_gimbal_ctrl_full;
//processed cmd
pc_info_t pc_info; //msg info from pc

//origial information to pc
embed_msg_to_pc_t embed_msg_to_pc;

//Embedded -> miniPC
embed_base_info_t embed_base_info;
embed_gimbal_info_t embed_gimbal_info;

/*
chassis_info 60 Hz
1/60 = 0.01666 s = 16.667s

gimbal_info 60 Hz

*/
const uint32_t chassis_info_embed_sendFreq = 16; //10; //17; //pre-determined send freq
const uint32_t gimbal_info_embed_sendFreq = 16; 

/*init_miniPC_comm_struct_data; this is the main task
pc -> embed
*/
void init_pc_to_embed_Main_comm_struct_data(void)
{
	const static fp32 yawMove_aid_order_filter[1] = {0.3333333333f};
	const static fp32 pitchMove_aid_order_filter[1] = {0.3333333333f};
	
	memset(&pc_comm_receive_header, 0, sizeof(pc_comm_frame_header_t));
	memset(&pc_send_header, 0, sizeof(pc_comm_embed_send_header_t));
	
	memset(&pc_ui_msg, 0, sizeof(pc_ui_msg_t));
	memset(&pc_cmd_chassis_control, 0, sizeof(pc_cmd_chassis_control_t));
	memset(&pc_cmd_gimbal_ctrl_aid, 0, sizeof(pc_cmd_gimbal_ctrl_t));
	memset(&pc_cmd_gimbal_ctrl_full, 0, sizeof(pc_cmd_gimbal_ctrl_t));
	
	memset(&pc_info, 0, sizeof(pc_info_t)); // this is used in both tasks
	
	//init important control related values
	pc_info.autoAimFlag = 0;
	
	//低通滤波
	first_order_filter_init(&pc_info.pitchMove_aid_filter, MINIPC_AID_GIMBAL_CONTROL_MSG_TIME, pitchMove_aid_order_filter);
	first_order_filter_init(&pc_info.yawMove_aid_filter, MINIPC_AID_GIMBAL_CONTROL_MSG_TIME, yawMove_aid_order_filter);
	
}
//embed -> pc
void init_embed_to_pc_comm_struct_data(void)
{
	memset(&embed_base_info, 0, sizeof(embed_base_info_t));
	memset(&embed_gimbal_info, 0, sizeof(embed_gimbal_info_t));
	
	memset(&embed_send_protocol, 0, sizeof(embed_send_protocol_t));
	memset(&embed_msg_to_pc, 0, sizeof(embed_msg_to_pc_t));
	
	//init information pckg
	embed_msg_to_pc.chassis_move_ptr = get_chassis_pointer();
	embed_msg_to_pc.gimbal_control_ptr = get_gimbal_pointer();
	embed_msg_to_pc.quat_ptr = get_INS_quat(); //get_INS_gimbal_quat();
	embed_msg_to_pc.shoot_control_ptr = get_robot_shoot_control();
	embed_msg_to_pc.chassis_odom_ptr = get_chassis_odom_pointer();
	
	embed_send_protocol.p_header = &pc_send_header;
	embed_send_protocol.chassis_info_embed_send_TimeStamp = xTaskGetTickCount();
	embed_send_protocol.gimbal_info_embed_send_TimeStamp = xTaskGetTickCount();
}

/* ---------- setter method 赋值到 pc_info中 ---------- */
void set_autoAimFlag(uint8_t autoAimFlag)
{
	pc_info.autoAimFlag = autoAimFlag;
}
/* ---------- setter method end ---------- */

/* ---------- getter method 获取最终解包到 pc_info 中的数据 ---------- */
// see the struct for the detailed information
// fp32 yawMove_aid; enable_not_detect_set_zero=1使能 未检测到时 返回值set为0
fp32 get_yawMove_aid(uint8_t enable_not_detect_set_zero)
{
	if(enable_not_detect_set_zero)
	{
		if(is_enemy_detected_with_pc_toe())
		{
			return pc_info.yawMove_aid;
		}
		else
		{
			return 0.0f;
		}
	}
	else
	{
		return pc_info.yawMove_aid;
	}
}

//fp32 pitchMove_aid; enable_not_detect_set_zero=1使能 未检测到时 返回值set为0
fp32 get_pitchMove_aid(uint8_t enable_not_detect_set_zero)
{
	if(enable_not_detect_set_zero)
	{
		if(is_enemy_detected_with_pc_toe())
		{
			return pc_info.pitchMove_aid;
		}
		else
		{
			return 0.0f;
		}
	}
	else
	{
		return pc_info.pitchMove_aid;
	}
}

//fp32 yawMove_absolute;
fp32 get_yawMove_absolute()
{
	return pc_info.yawMove_absolute;
}

//fp32 pitchMove_absolute;
fp32 get_pitchMove_absolute()
{
	return pc_info.pitchMove_absolute;
}

//uint8_t enemy_detected_with_pc_toe
/*true - enemy detected; false - NOT detected*/
bool_t is_enemy_detected_with_pc_toe()
{
	if(toe_is_error(PC_TOE))
	{
		return 0;
	}
	
	return (pc_info.enemy_detected == 0xff);
}
/*不考虑掉线的情况 - 直接判断enemy_detected*/
bool_t is_enemy_detected()
{
	return (pc_info.enemy_detected == 0xff);
}
//原始数据 enemy_detected
uint8_t get_enemy_detected()
{
	return pc_info.enemy_detected;
}

/*当裁判系统掉线时不开火*/
uint8_t get_shootCommand_with_referee_toe()
{
	if(toe_is_error(REFEREE_TOE))
	{
		return 0x00; //不开火
	}
	else
	{
		return pc_info.shootCommand;
	}
	
}

//uint8_t shootCommand;
uint8_t get_shootCommand()
{
	return pc_info.shootCommand;
}

//uint8_t cv_gimbal_sts;
uint8_t get_cv_gimbal_sts()
{
	return pc_info.cv_gimbal_sts;
}

//fp32 aim_pos_dis;
fp32 get_aim_pos_dis()
{
	return pc_info.aim_pos_dis;
}

//uint8_t autoAimFlag
uint8_t get_autoAimFlag()
{
	return pc_info.autoAimFlag;
}

/* ---------- getter method end ---------- */

void cmd_process_pc_cmd_chassis_control(void)
{
	pc_info.vx_m = (fp32)pc_cmd_chassis_control.vx_mm_wrt_gimbal / 1000.0f;
	pc_info.vy_m = (fp32)pc_cmd_chassis_control.vy_mm_wrt_gimbal / 1000.0f;
	pc_info.vw_m = (fp32)pc_cmd_chassis_control.vw_mm / 1000.0f;
	
	if(pc_cmd_chassis_control.chassis_mode == 0)
	{
		pc_info.chassis_mode = PC_CHASSIS_NO_FOLLOW_YAW;
	}
	else if(pc_cmd_chassis_control.chassis_mode == 1)
	{
		pc_info.chassis_mode = PC_CHASSIS_FOLLOW_GIMBAL_YAW;
	}
	else if(pc_cmd_chassis_control.chassis_mode == 2)
	{
		pc_info.chassis_mode = PC_CHASSIS_SPIN;
	}
	else
	{
		pc_info.chassis_mode = PC_CHASSIS_NO_FOLLOW_YAW;
	}
	
}

void cmd_process_pc_cmd_gimbal_ctrl_aid(void) //TODO添加数据合理性判断
{
	//测试滤波
	first_order_filter_cali(&pc_info.yawMove_aid_filter, pc_cmd_gimbal_ctrl_aid.yaw);
	first_order_filter_cali(&pc_info.pitchMove_aid_filter, pc_cmd_gimbal_ctrl_aid.pitch);
	
	pc_info.yawMove_aid = 0.0010f * (fp32)pc_cmd_gimbal_ctrl_aid.yaw / 10000.0f; //003f 008f 0015f - 0005f
	pc_info.pitchMove_aid = 0.001f * (fp32)pc_cmd_gimbal_ctrl_aid.pitch / 10000.0f; //008f 004f 002f - 0005f
//	pc_info.yawMove_aid = 0.003f * (fp32)pc_info.yawMove_aid_filter.out / 10000.0f; //003f 008f
//	pc_info.pitchMove_aid = 0.008f * (fp32)pc_info.pitchMove_aid_filter.out / 10000.0f; //008f 010f
	
	pc_info.enemy_detected = pc_cmd_gimbal_ctrl_aid.is_detect;
	pc_info.shootCommand = pc_cmd_gimbal_ctrl_aid.shoot;

	pc_info.cv_gimbal_sts = 1; //aim mode FSM
	
	//Need to handle the erase of miniPC_info.yawMove_absolute and pitchMove?
	//3-26-2023 no need to do that for now
}

void cmd_process_pc_cmd_gimbal_ctrl_full(void) //TODO添加数据合理性判断
{
	pc_info.yawMove_absolute = (fp32)pc_cmd_gimbal_ctrl_full.yaw / 10000.0f;
	pc_info.pitchMove_absolute = (fp32)pc_cmd_gimbal_ctrl_full.pitch / 10000.0f;
	pc_info.enemy_detected = pc_cmd_gimbal_ctrl_full.is_detect;
	pc_info.shootCommand = pc_cmd_gimbal_ctrl_full.shoot;

	pc_info.cv_gimbal_sts = 2; //aim mode FSM
	
	//Erase the other
	pc_info.yawMove_aid = 0.0f;
	pc_info.pitchMove_aid = 0.0f;
}

void cmd_process_pc_ui_msg(void) //TODO添加数据合理性判断
{
	pc_info.dis_raw = pc_ui_msg.detected_enemy_distance;
	
	pc_info.dis = (fp32)pc_ui_msg.detected_enemy_distance / 100.0f;
	pc_info.aim_pos_dis = (fp32)pc_ui_msg.aim_pos_distance / 100.0f;
}

void pc_comm_data_solve(uint8_t *frame)
{
		//detect_hook(PC_TOE); //for target of evaluation
	
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&pc_comm_receive_header, frame, sizeof(pc_comm_frame_header_t));

    index += sizeof(pc_comm_frame_header_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
	
		//for cmdid debug
	  pc_comm_unpack_data_obj.cmd_id_pc_comm_data_solve_debug = cmd_id;

    switch (cmd_id)
    {
        case CHASSIS_REL_CTRL_CMD_ID:
        {
            memcpy(&pc_cmd_chassis_control, frame + index, sizeof(pc_cmd_chassis_control_t));
						cmd_process_pc_cmd_chassis_control();
        }
        break;
        case GIMBAL_REL_AID_CTRL_CMD_ID:
        {
            memcpy(&pc_cmd_gimbal_ctrl_aid, frame + index, sizeof(pc_cmd_gimbal_ctrl_t));
					  cmd_process_pc_cmd_gimbal_ctrl_aid();
        }
        break;
        case GIMBAL_REL_FULL_CTRL_CMD_ID:
        {
            memcpy(&pc_cmd_gimbal_ctrl_full, frame + index, sizeof(pc_cmd_gimbal_ctrl_t));
						cmd_process_pc_cmd_gimbal_ctrl_full();
        }
        break;

        case UI_REL_MSG_CMD_ID:
        {
            memcpy(&pc_ui_msg, frame + index, sizeof(pc_ui_msg_t));
				  	cmd_process_pc_ui_msg();
        }
        break;
       
        default:
        {
            break;
        }
    }
}

void pc_offline_proc()
{
		pc_info.pc_connection_status = PC_OFFLINE;
	  pc_info.cv_gimbal_sts = 0;
}

bool_t pc_is_data_error_proc()
{
		pc_info.pc_connection_status = PC_ONLINE;
		return 0;
}

/* -------------------------------- USART SEND DATA FILL-------------------------------- */
void embed_all_info_update_from_sensor()
{
	/*
	fp32 s_vx_m; // m/s
	fp32 s_vy_m; // m/s
	fp32 s_vw_m; // radian/s
	
	uint8_t energy_buff_pct; //get_superCap_charge_pwr
	
	fp32 yaw_relative_angle; //= rad
  fp32 pitch_relative_angle;

	fp32 quat[4];

  fp32 shoot_bullet_speed; // = m/s

  uint8_t robot_id;
	*/
	embed_msg_to_pc.vx_wrt_gimbal = embed_msg_to_pc.chassis_move_ptr->vx_gimbal_orientation; //embed_msg_to_pc.chassis_move_ptr->vx;
	embed_msg_to_pc.vy_wrt_gimbal = embed_msg_to_pc.chassis_move_ptr->vy_gimbal_orientation; //embed_msg_to_pc.chassis_move_ptr->vy;
	embed_msg_to_pc.vw_wrt_chassis = embed_msg_to_pc.chassis_move_ptr->wz;
	embed_msg_to_pc.energy_buff_pct = (uint8_t) get_current_cap_pct();
	
	//9-30新增里程计 or use embed_msg_to_pc.chassis_odom_ptr
	embed_msg_to_pc.odom_dist_x = get_chassis_gimbal_dir_distance_x(); //not used
	embed_msg_to_pc.odom_dist_y = get_chassis_gimbal_dir_distance_y(); //not used
	embed_msg_to_pc.odom_dist_wz = get_chassis_gimbal_dir_distance_wz(); //not used
	
	//10-28 全局坐标 相对于0点 里程计
	embed_msg_to_pc.odom_coord_x = get_chassis_odom_coord_x();
	embed_msg_to_pc.odom_coord_y = get_chassis_odom_coord_y();
	embed_msg_to_pc.odom_coord_z = 0.0f; //no altitude for RMUL
	
	embed_msg_to_pc.yaw_absolute_angle = embed_msg_to_pc.gimbal_control_ptr->gimbal_yaw_motor.absolute_angle; //6-22修改relative_angle
	embed_msg_to_pc.pitch_absolute_angle = embed_msg_to_pc.gimbal_control_ptr->gimbal_pitch_motor.absolute_angle; //relative_angle
	
	embed_msg_to_pc.quat[0] = embed_msg_to_pc.quat_ptr[0];
	embed_msg_to_pc.quat[1] = embed_msg_to_pc.quat_ptr[1];
	embed_msg_to_pc.quat[2] = embed_msg_to_pc.quat_ptr[2];
	embed_msg_to_pc.quat[3] = embed_msg_to_pc.quat_ptr[3];
	
	// = (uint16_t)(shoot_control.predict_shoot_speed*10); //anticipated predicated bullet speed
	embed_msg_to_pc.shoot_bullet_speed = embed_msg_to_pc.shoot_control_ptr->predict_shoot_speed;
	
	// 7-7-2023 保证身份安全
	if(toe_is_error(REFEREE_TOE))
	{
		embed_msg_to_pc.robot_id = 0x00;
	}
	else
	{
		embed_msg_to_pc.robot_id = get_robot_id(); //RED_STANDARD_1; //TODO: whether get from ref or hardcode - fail safe
	}
	
	//6-25-2023 新增云台角速度 原始数据单位为 rad/s
	embed_msg_to_pc.gimbal_yaw_rate = embed_msg_to_pc.gimbal_control_ptr->gimbal_yaw_motor.motor_gyro;
	
}

void embed_base_info_msg_data_update(embed_base_info_t* embed_base_info_ptr, embed_msg_to_pc_t* embed_msg_to_pc_ptr)
{	
	//m/s * 1000 <-->mm/s 
	embed_base_info_ptr->vx_mm_wrt_gimbal = (int16_t) (embed_msg_to_pc_ptr->vx_wrt_gimbal * 1000.0f);
	embed_base_info_ptr->vy_mm_wrt_gimbal = (int16_t) (embed_msg_to_pc_ptr->vy_wrt_gimbal * 1000.0f);
	embed_base_info_ptr->vw_krad = (int16_t) (embed_msg_to_pc_ptr->vw_wrt_chassis * 1000.0f);
	
	embed_base_info_ptr->energy_buff_pct = embed_msg_to_pc_ptr->energy_buff_pct;
	
	//10-28新增底盘里程计 Pose Message miniPC needs https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
	
	//Point Message
	embed_base_info_ptr->odom_coord_x_mm = (int16_t) (embed_msg_to_pc_ptr->odom_coord_x * 1000.0f);
	embed_base_info_ptr->odom_coord_y_mm = (int16_t) (embed_msg_to_pc_ptr->odom_coord_y * 1000.0f);
	embed_base_info_ptr->odom_coord_z_mm = 0; //(int16_t) (embed_msg_to_pc_ptr->odom_coord_z * 1000.0f);
	
	//Quaternion Message in x y z w order - quat sync with base information
	embed_base_info_ptr->quat[3] = (uint16_t) ( (embed_msg_to_pc_ptr->quat[0]+1) * 10000.0f ); //(quat[i]+1)*10000; linear trans.
	embed_base_info_ptr->quat[0] = (uint16_t) ( (embed_msg_to_pc_ptr->quat[1]+1) * 10000.0f );
	embed_base_info_ptr->quat[1] = (uint16_t) ( (embed_msg_to_pc_ptr->quat[2]+1) * 10000.0f );
	embed_base_info_ptr->quat[2] = (uint16_t) ( (embed_msg_to_pc_ptr->quat[3]+1) * 10000.0f );
	
	// check that quat is a valid nuumber
	for(uint8_t i = 0; i < 4; i++)
	{
			if(fabs(embed_msg_to_pc_ptr->quat[i]) > 1.01f)
			{
				for(uint8_t j = 0; j < 4; j++)
				{
					embed_base_info_ptr->quat[j] = 0;
				}
				break;
			}
	}
	
//	//For debug only
//	embed_base_info_ptr->vx_mm = 0x1234;
//	embed_base_info_ptr->vy_mm = 0x5678;
//	embed_base_info_ptr->vw_mm = 0x9ABC;
//	
//	embed_base_info_ptr->energy_buff_pct = 0xDE;
}

void embed_gimbal_info_msg_data_update(embed_gimbal_info_t* embed_gimbal_info_ptr, embed_msg_to_pc_t* embed_msg_to_pc_ptr)
{
	embed_gimbal_info_ptr->pitch_absolute_angle = (int16_t) (embed_msg_to_pc_ptr->pitch_absolute_angle * 10000.0f); //= rad * 10000
	embed_gimbal_info_ptr->yaw_absolute_angle = (int16_t) (embed_msg_to_pc_ptr->yaw_absolute_angle * 10000.0f);
	
	for(uint8_t i = 0; i < 4; i++) //quat sync with gimbal
	{
			if(fabs(embed_msg_to_pc_ptr->quat[i]) > 1.01f)
			{
				for(uint8_t j = 0; j < 4; j++)
				{
					embed_gimbal_info_ptr->quat[j] = 0;
				}
				break;
			}
			
			embed_gimbal_info_ptr->quat[i] = (uint16_t) ( (embed_msg_to_pc_ptr->quat[i]+1) * 10000.0f ); //(quat[i]+1)*10000; linear trans.
	}
	
	embed_gimbal_info_ptr->shoot_bullet_speed = (uint16_t) (embed_msg_to_pc_ptr->shoot_bullet_speed * 10.0f);
	embed_gimbal_info_ptr->robot_id = embed_msg_to_pc_ptr->robot_id;
//	embed_gimbal_info_ptr->yaw_rate = (uint16_t) (embed_msg_to_pc_ptr->gimbal_control_ptr->gimbal_yaw_motor.motor_gyro * 10000.0f); //= rad/s * 10000
	embed_gimbal_info_ptr->yaw_rate = embed_msg_to_pc_ptr->gimbal_control_ptr->gimbal_yaw_motor.motor_gyro;
}

/**
 * @brief refresh data struct to ring buffer fifo or send directly
 * 
 * To use: uniform refresh function substitute: embed_base_info_refresh + embed_gimbal_info_refresh
 * The use of global variable pc_send_header for debug purpose
 * @param data section point; data_size: variable lengthed data; data section data size; cmdid
 */
void embed_info_msg_refresh(uint8_t* embed_info_ptr, uint32_t data_size, uint16_t cmd_id_para)
{
	//uint8_t is unsigned char for STM32
	uint8_t* framepoint;  //read write pointer
  uint16_t frametailCRC=0xFFFF;  //CRC16 check sum
	uint16_t* frametail_ptr;
	
	framepoint = (uint8_t*)embed_send_protocol.p_header;
	embed_send_protocol.p_header->SOF = PC_HEADER_SOF; //0xAF;
	embed_send_protocol.p_header->frame_length = PC_HEADER_CRC_CMDID_LEN + data_size; //frame_length是整帧长度
	embed_send_protocol.p_header->seq = 1; //TODO: add global for this
	embed_send_protocol.p_header->CRC8 = get_CRC8_check_sum((unsigned char *) framepoint, PC_PROTOCOL_HEADER_SIZE-1, 0xFF);
	
	embed_send_protocol.p_header->cmd_id = cmd_id_para;
	
	
	//put header + cmdid to ram buffer
	for(embed_send_protocol.index = 0; embed_send_protocol.index < PC_HEADER_CMDID_LEN; embed_send_protocol.index++ )
	{
		embed_send_protocol.send_ram_buff[embed_send_protocol.index] = *framepoint;
		uart1_embed_send_byte(embed_send_protocol.send_ram_buff[embed_send_protocol.index]); //现在发送
		
		frametailCRC = get_CRC16_check_sum(framepoint, 1, frametailCRC);
		framepoint++;
	}
	
	//put data section to ram buffer
	framepoint = (uint8_t*)embed_info_ptr;
	for(; embed_send_protocol.index < PC_HEADER_CMDID_LEN + data_size; embed_send_protocol.index++ )
	{
		embed_send_protocol.send_ram_buff[embed_send_protocol.index] = *framepoint;
		uart1_embed_send_byte(embed_send_protocol.send_ram_buff[embed_send_protocol.index]); //现在发送
		
		frametailCRC = get_CRC16_check_sum(framepoint, 1, frametailCRC);
		framepoint++;
	}
	
	//put frame tail check sum to ram buffer CRC16 小端模式 换向
	frametail_ptr = (uint16_t*) &embed_send_protocol.send_ram_buff[embed_send_protocol.index];
	frametail_ptr[0] = frametailCRC;
	
	//发送包尾
	for(; embed_send_protocol.index < embed_send_protocol.p_header->frame_length; embed_send_protocol.index++)
	{
		uart1_embed_send_byte(embed_send_protocol.send_ram_buff[embed_send_protocol.index]); //现在发送
	}
	
	/*
	3-26现象记录: sendFreq = 16, 实际上在ROS rqt上显示只有52Hz左右. 分析:
	1/60 = 0.016s = 16ms; 1/52 = 0.0192; 1/50 = 0.020 = 20ms; 且 20ms - 16ms = 4ms
	vTaskDelay(4); 在这个task的while中循环; 一般情况sendFreq = 16里, 都会远小于16ms
	理想情况下, 假设这个task是实时的: 没有vTaskDelay(4), sendFreq = 16会使得 执行这个东西的频率为60Hz;
	16  16  16  16  16  16  16  16 理想周期, 16ms 一次
	 |---|晚开始赋值 只能 影响相位差 "比"的那个点更靠后了
	|   |   |   |   |   |   |   |
	(2) (2) (2) (2) (2) (2) (2) (2) 每一次执行sendFreq = 16中的耗时
	
	如果while中循环一次再vTaskDelay(4), 是会使得16的理想发送周期 + 4ms = 20ms
	20  20  20  20  20  20  20  20
	|   |   |   |   |   |   |   |
	(2) (2) (2) (2) (2) (2) (2) (2) 每一次执行sendFreq = 16中的耗时
	也就是观察到的 1/50 = 0.020 = 20ms
	*/
	
}

void embed_send_data_to_pc_loop()
{
	//Update info from robot sensor; update every iteration
	embed_all_info_update_from_sensor();
	
//	//定时创建一次动态的--------------
//	if(xTaskGetTickCount() - ui_dynamic_crt_sendFreq > ui_dynamic_crt_send_TimeStamp)
//	{
//			ui_dynamic_crt_send_TimeStamp = xTaskGetTickCount(); //更新时间戳 
//			ui_dynamic_crt_send_fuc(); //到时间了, 在客户端创建一次动态的图像
//	}
	
	//chassis_info gimbal_info 60Hz create and enqued to fifo
	if(xTaskGetTickCount() - chassis_info_embed_sendFreq > embed_send_protocol.chassis_info_embed_send_TimeStamp)
	{
		embed_send_protocol.chassis_info_embed_send_TimeStamp = xTaskGetTickCount();
		//its time to do 1 msg send
		
		//Copy value to send struct. simular to Float_Draw
		embed_base_info_msg_data_update(&embed_base_info, &embed_msg_to_pc);
		embed_gimbal_info_msg_data_update(&embed_gimbal_info, &embed_msg_to_pc);
		
		//msg to fifo; this is like refresh
//		embed_base_info_refresh(&embed_chassis_info, sizeof(embed_chassis_info_t));
//		uart1_poll_dma_tx();
//		embed_gimbal_info_refresh(&embed_gimbal_info, sizeof(embed_gimbal_info_t)); // written later
		
		//try uniform function to refresh
		embed_info_msg_refresh((uint8_t*) &embed_base_info, sizeof(embed_base_info_t), BASE_INFO_CMD_ID);	
		uart1_poll_dma_tx();
		embed_info_msg_refresh((uint8_t*) &embed_gimbal_info, sizeof(embed_gimbal_info_t), GIMBAL_INFO_CMD_ID);
		
		/*
		进入这个if的频率决定了生产频率, 到时间后才进入这个if, 没到时间不进入这个if
		没到时间需确认消费者在消费状态
		*/
	}//
//	else //testing
//	{
//		vTaskDelay(1); //频率 58.37 ~ 58.4
//		{
//			uint8_t i = 0;
//			i++;
//		}
//	}
	
	
	//enable uart tx DMA which is the DMA poll
	if(uart1_poll_dma_tx())
	{
		embed_send_protocol.relative_send_fail_cnts++;
	}
	else
	{
		embed_send_protocol.relative_send_fail_cnts = 0;
	}
	
	//if reach a certain number, enforce sending, ensure fifo will not be used out
	if(embed_send_protocol.relative_send_fail_cnts >= 5)
	{
		while(!(get_uart1_embed_send_status()==0))
		{
			vTaskDelay(1);
			uart1_poll_dma_tx();
		}
		
		uart1_poll_dma_tx();
		embed_send_protocol.relative_send_fail_cnts = 0;
	}
//}//----------------
}

/* -------------------------------- USART SEND DATA FILL END-------------------------------- */



/* -------------------------------- Old code back up --------------------------------------- */
/*这个是集中发送的版本 仅作保存 时间复杂度略微的比现在的高*/
//void embed_info_msg_refresh(uint8_t* embed_info_ptr, uint32_t data_size, uint16_t cmd_id_para)
//{
//	//uint8_t is unsigned char for STM32
//	uint8_t* framepoint;  //read write pointer
//  uint16_t frametailCRC=0xFFFF;  //CRC16 check sum
//	uint16_t* frametail_ptr;
//	
//	framepoint = (uint8_t*)embed_send_protocol.p_header;
//	embed_send_protocol.p_header->SOF = PC_HEADER_SOF; //0xAF;
//	embed_send_protocol.p_header->frame_length = PC_HEADER_CRC_CMDID_LEN + data_size; //frame_length是整帧长度
//	embed_send_protocol.p_header->seq = 1; //TODO: add global for this
//	embed_send_protocol.p_header->CRC8 = get_CRC8_check_sum((unsigned char *) framepoint, PC_PROTOCOL_HEADER_SIZE-1, 0xFF);
//	
//	embed_send_protocol.p_header->cmd_id = cmd_id_para;
//	
//	
//	//put header + cmdid to ram buffer
//	for(embed_send_protocol.index = 0; embed_send_protocol.index < PC_HEADER_CMDID_LEN; embed_send_protocol.index++ )
//	{
//		embed_send_protocol.send_ram_buff[embed_send_protocol.index] = *framepoint;
//		frametailCRC = get_CRC16_check_sum(framepoint, 1, frametailCRC);
//		framepoint++;
//	}
//	
//	//put data section to ram buffer
//	framepoint = (uint8_t*)embed_info_ptr;
//	for(; embed_send_protocol.index < PC_HEADER_CMDID_LEN + data_size; embed_send_protocol.index++ )
//	{
//		embed_send_protocol.send_ram_buff[embed_send_protocol.index] = *framepoint;
//		frametailCRC = get_CRC16_check_sum(framepoint, 1, frametailCRC);
//		framepoint++;
//	}
//	
//	//put frame tail check sum to ram buffer CRC16 小端模式 换向
//	frametail_ptr = (uint16_t*) &embed_send_protocol.send_ram_buff[embed_send_protocol.index];
//	frametail_ptr[0] = frametailCRC;
//	
//	//集中发送
//	for(embed_send_protocol.index = 0; embed_send_protocol.index < embed_send_protocol.p_header->frame_length; embed_send_protocol.index++)
//	{
//		uart1_embed_send_byte(embed_send_protocol.send_ram_buff[embed_send_protocol.index]);
//	}
//	
//}

///**
// * @brief refresh data struct to ring buffer fifo or send directly
// * 仅作保存 时间复杂度略微的比现在的高
// * @param variable lengthed data
// */
//void embed_gimbal_info_refresh(embed_gimbal_info_t* embed_gimbal_info_ptr, uint32_t data_size)
//{
//	unsigned char *framepoint;  //read write pointer
//  uint16_t frametailCRC=0xFFFF;  //CRC16 check sum
//	uint16_t* frametail_ptr;
//	
//	framepoint = (unsigned char *)embed_send_protocol.p_header;
//	embed_send_protocol.p_header->SOF = PC_HEADER_SOF; //0xAF;
//	embed_send_protocol.p_header->frame_length = PC_HEADER_CRC_CMDID_LEN + data_size;
//	embed_send_protocol.p_header->seq = 1; //TODO: add global for this
//	embed_send_protocol.p_header->CRC8 = get_CRC8_check_sum(framepoint, PC_PROTOCOL_HEADER_SIZE-1, 0xFF);
//	
//	embed_send_protocol.p_header->cmd_id = GIMBAL_INFO_CMD_ID;
//	
//	
//	//put header + cmdid to ram buffer
//	for(embed_send_protocol.index = 0; embed_send_protocol.index < PC_HEADER_CMDID_LEN; embed_send_protocol.index++ )
//	{
//		embed_send_protocol.send_ram_buff[embed_send_protocol.index] = *framepoint;
//		frametailCRC = get_CRC16_check_sum(framepoint, 1, frametailCRC);
//		framepoint++;
//	}
//	
//	//put data section to ram buffer
//	framepoint = (unsigned char *)embed_gimbal_info_ptr;
//	for(; embed_send_protocol.index < PC_HEADER_CMDID_LEN + data_size; embed_send_protocol.index++ )
//	{
//		embed_send_protocol.send_ram_buff[embed_send_protocol.index] = *framepoint;
//		frametailCRC = get_CRC16_check_sum(framepoint, 1, frametailCRC);
//		framepoint++;
//	}
//	
//	//put frame tail check sum to ram buffer CRC16 小端模式 换向(就是避开之前的8bits写, 直接16bits写, STM32 小端模式)
//	frametail_ptr = (uint16_t*) &embed_send_protocol.send_ram_buff[embed_send_protocol.index];
//	frametail_ptr[0] = frametailCRC;
//	
//	//集中发送
//	for(embed_send_protocol.index = 0; embed_send_protocol.index < embed_send_protocol.p_header->frame_length; embed_send_protocol.index++)
//	{
//		uart1_embed_send_byte(embed_send_protocol.send_ram_buff[embed_send_protocol.index]);
//	}
//}

///**
// * @brief refresh data struct to ring buffer fifo or send directly
// *
// * @param data_size: variable lengthed data; data section data size
// */
//void embed_base_info_refresh(embed_chassis_info_t* embed_chassis_info_ptr, uint32_t data_size)
//{
//	unsigned char *framepoint;  //read write pointer
//  uint16_t frametailCRC=0xFFFF;  //CRC16 check sum
//	uint16_t* frametail_ptr;
//	
//	framepoint = (unsigned char *)embed_send_protocol.p_header;
//	embed_send_protocol.p_header->SOF = PC_HEADER_SOF; //0xAF;
//	embed_send_protocol.p_header->frame_length = PC_HEADER_CRC_CMDID_LEN + data_size;
//	embed_send_protocol.p_header->seq = 1; //TODO: add global for this
//	embed_send_protocol.p_header->CRC8 = get_CRC8_check_sum(framepoint, PC_PROTOCOL_HEADER_SIZE-1, 0xFF);
//	
//	embed_send_protocol.p_header->cmd_id = BASE_INFO_CMD_ID;
//	
//	
//	//put header + cmdid to ram buffer
//	for(embed_send_protocol.index = 0; embed_send_protocol.index < PC_HEADER_CMDID_LEN; embed_send_protocol.index++ )
//	{
//		embed_send_protocol.send_ram_buff[embed_send_protocol.index] = *framepoint;
//		frametailCRC = get_CRC16_check_sum(framepoint, 1, frametailCRC);
//		framepoint++;
//	}
//	
//	//put data section to ram buffer
//	framepoint = (unsigned char *)embed_base_info_ptr;
//	for(; embed_send_protocol.index < PC_HEADER_CMDID_LEN + data_size; embed_send_protocol.index++ )
//	{
//		embed_send_protocol.send_ram_buff[embed_send_protocol.index] = *framepoint;
//		frametailCRC = get_CRC16_check_sum(framepoint, 1, frametailCRC);
//		framepoint++;
//	}
//	
//	//put frame tail check sum to ram buffer CRC16 小端模式 换向
//	frametail_ptr = (uint16_t*) &embed_send_protocol.send_ram_buff[embed_send_protocol.index];
//	frametail_ptr[0] = frametailCRC;
//	
//	//集中发送
//	for(embed_send_protocol.index = 0; embed_send_protocol.index < embed_send_protocol.p_header->frame_length; embed_send_protocol.index++)
//	{
//		uart1_embed_send_byte(embed_send_protocol.send_ram_buff[embed_send_protocol.index]);
//	}
//	
//}
