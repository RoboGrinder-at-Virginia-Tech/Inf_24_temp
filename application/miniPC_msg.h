#ifndef __MINIPC_MSG__
#define __MINIPC_MSG__

#include "main.h"
#include "struct_typedef.h"
#include "chassis_task.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "shoot.h"
#include "odometer_task.h"
#include "AHRS_middleware.h"

/*---------------------------------------------------- Raw Data Msg ----------------------------------------------------*/
//robot_id_t is defined in referee.h file
/*
typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
		RED_DART				= 8,
		RED_RADAR 			= 9,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
		BLUE_DART				= 108,
		BLUE_RADAR 			= 109,
} robot_id_t;
*/

/*
UI related information - miniPC->Embeded
*/
typedef __packed struct
{
    // = m * 100 = cm
    //int16_t detected_enemy_distance; //=0 when not detected
    //int16_t aim_pos_distance;

		//distance to a recognised enemy; =0 when not detected
		uint16_t detected_enemy_distance;
		//distance to the current barrel pointing target(pixel point)
    uint16_t aim_pos_distance;
}pc_ui_msg_t; //UI_REL_MSG_CMD_ID

/*
miniPC to control the robot's chassis - miniPC->Embeded
*/
typedef enum
{
   PC_BASE_CHASSIS_NO_FOLLOW_YAW=0, //uint8_t 0
   PC_BASE_CHASSIS_FOLLOW_GIMBAL_YAW=1, //uint8_t 1
   PC_BASE_CHASSIS_SPIN=2, //uint8_t 2
} pc_base_chassis_mode_e;
typedef __packed struct
{
  int16_t vx_mm_wrt_gimbal; // forward/back
//: m/s * 1000 <-->mm/s 

  int16_t vy_mm_wrt_gimbal; // left/right
  int16_t vw_mm; // ccw positive
//vw_mm: radian/s * 1000

 //base_chassis_mode_e chassis_mode;
  uint8_t chassis_mode;
}pc_cmd_base_control_t; //CHASSIS_REL_CTRL_CMD_ID

/*
chassis related info. Send to PC
Embedded -> miniPC
*/
typedef __packed struct
{
	//10-14 new odom protocol
	//Point: m/s * 1000 <-->mm/s 
	int32_t odom_coord_x_mm; // forward/back x odom in mm
	int32_t odom_coord_y_mm; // left/right
	int32_t odom_coord_z_mm; // z axis height = 0 for RMUL
	//Quaternion:
	/*quaternion msg: uint16_t quat[i]: [0],[1],[2],[3]
       In embeded, quat has range:(-1, +1), 
       ->transform: +1->range:(0, +2)
       ->transform: * 10000->quat[i] with 16 bits uint16_t 
		Quat with respect to LIDAR, only yaw axis should have a changing number. Roll Pitch rotational axis fixed
		quat[0] - x: direction points out of the barrel, roll axis
		quat[1] - y: direction points toward left of turret, pitch axis
		quat[2] - z: direction points up, yaw axis
		quat[3] - w: direction points out of the barrel
		*/
   uint16_t quat[4];
	
	//Other: dist the base rotate - # of rotation rounds
	int32_t odom_dist_wz_krad; // ccw positive
//or wz_mm: radian * 1000, thousand rad

//Other: velocity info
  int16_t vx_mm_wrt_gimbal; // forward/back
//: m/s * 1000 <-->mm/s 

  int16_t vy_mm_wrt_gimbal; // left/right
  int16_t vw_krad; // ccw positive
//or vw_mm: radian/s * 1000, thousand rad/s

 uint8_t energy_buff_pct; //chassis available energy left
//superCap or equivalent energy percentage
	
}embed_base_info_t; //BASE_INFO_CMD_ID

/*
Gimbal related control - miniPC->Embedded
*/
typedef __packed struct
{
    int16_t yaw; //=rad*10000
    int16_t pitch;
    uint8_t is_detect; //0 = NOT detected, 1 = detected
    uint8_t shoot; //0x00 = Hold, 0xff = Fire
}pc_cmd_gimbal_ctrl_t;
//GIMBAL_REL_AID_CTRL_CMD_ID
//GIMBAL_REL_FULL_CTRL_CMD_ID

/*
Gimbal information - Embedded -> miniPC
*/
typedef __packed struct
{
    int16_t yaw_absolute_angle; //= rad * 10000
    int16_t pitch_absolute_angle;

    /*quaternion msg: uint16_t quat[i]: [0],[1],[2],[3] same as embeded order
       In embeded, quat has range:(-1, +1), 
       ->transform: +1->range:(0, +2)
       ->transform: * 10000->quat[i] with 16 bits uint16_t */
    uint16_t quat[4];

    /*
    原始数据: x m/s,
     (x*10)后发给PC, 即通信单位为 dm/s
    */
    // = m/s * 10 = dm/s
    uint16_t shoot_bullet_speed; //anticipated speed

/*
decimal val = hex val
0=0x00                
1=0x01                RED_HERO
3=0x03                RED_STANDARD_1
4=0x04                RED_STANDARD_2
5=0x05                RED_STANDARD_3
6=0x06                RED_AERIAL
7=0x07                RED_SENTRY
                
101=0x65                BLUE_HERO
103=0x67                BLUE_STANDARD_1
104=0x68                BLUE_STANDARD_2
105=0x69                BLUE_STANDARD_3
106=0x6A                BLUE_AERIAL
107=0x6B                BLUE_SENTRY
*/
    uint8_t robot_id;
		
//		int16_t yaw_rate; //= rad/s * 10000
		fp32 yaw_rate;
}embed_gimbal_info_t; //GIMBAL_INFO_CMD_ID
/*---------------------------------------------------- Raw Data Msg - End Above ----------------------------------------------------*/

/*---------------------------------------------------- Processed Data ----------------------------------------------------*/
//uint8_t  autoAimFlag = 1; //自动瞄准开关状态 0关 1自动瞄准
//uint8_t shootCommand = 0x00;//自动开火指令  0x00 = 停火  0xff = 开火
//uint8_t fricCommand = 0x01;// 摩擦轮转速指令  0x01 =低转  0x02 = 高转
//fp32 yawMove = 0;  //云台数据
//fp32 pitchMove = 0;  //云台数据
//uint32_t timeFlag = 0; //
typedef enum
{
	PC_OFFLINE,
	PC_ONLINE
}pc_connection_status_e;

/*
Embed to PC
float EBPct_fromCap; //relative to 0J 0%-100%
*/
//SZL 1-25-2023 EE->CV
typedef struct
{
	/* ------------------ sensor & information sent to pc ------------------*/
	//pointer to original source of information
	const chassis_move_t* chassis_move_ptr;
	const gimbal_control_t* gimbal_control_ptr;
	const fp32* quat_ptr; //const fp32 *get_INS_gimbal_quat(void)
	const shoot_control_t* shoot_control_ptr;
	const chassis_odom_info_t* chassis_odom_ptr;
	
	fp32 vx_wrt_gimbal; // 云台朝向 vx m/s
	fp32 vy_wrt_gimbal; // 云台朝向 vy m/s
	fp32 vw_wrt_chassis; // 底盘 radian/s
	
	uint8_t energy_buff_pct; //get_superCap_charge_pwr
	
	//9-30新增底盘 云台方向当前前进 里程计 odometer
	fp32 odom_dist_x; // forward/back x odom in m
	fp32 odom_dist_y; // left/right
	fp32 odom_dist_wz; // ccw positive rad unit
	//
	
	//10-28 相对于0点 全局坐标 里程计
	fp32 odom_coord_x;
	fp32 odom_coord_y;
	fp32 odom_coord_z; // = 0 for RMUL
	
	fp32 yaw_absolute_angle; //= rad based on gyro
  fp32 pitch_absolute_angle;

	fp32 quat[4];
	Quaternion quat_temp;
	
	//filter pitch
	Euler ang_filter_pit;
	Quaternion quat_filter_pit;
	Euler fil_quat_to_ang_debug_Euler;
	fp32 fil_quat_to_ang_debug_fp[3]; //roll-pitch-yaw order

  fp32 shoot_bullet_speed; // = m/s

  uint8_t robot_id;
	
	fp32 gimbal_yaw_rate; // 云台 yaw角速度 = rad/s
	
	/* ------------------ sensor & information sent to pc END ------------------*/
}embed_msg_to_pc_t;

//底盘任务控制间隔 0.002s
#define MINIPC_AID_GIMBAL_CONTROL_MSG_TIME 0.06f

typedef struct
{
	/* -------------------- Var from cv comm -------------------- */
	//uint8_t cv_chassis_sts;	
	//int16_t yawCommand; //delta yaw
	//int16_t pitchCommand; //delta pitch
	
	//FSM regarding connection
	pc_connection_status_e pc_connection_status;
	
	//chassis related: pc_cmd_chassis_control_t 
	fp32 vx_m; // m/s
	fp32 vy_m; // m/s
	fp32 vw_m; // radian/s
	
	pc_base_chassis_mode_e chassis_mode;
	
	//gimbal related: pc_cmd_gimbal_ctrl_t
	fp32 yawMove_aid;
	fp32 pitchMove_aid;
	
	fp32 yawMove_aid_filtered_val;
	fp32 pitchMove_aid_filtered_val;
	first_order_filter_type_t yawMove_aid_filter;
	first_order_filter_type_t pitchMove_aid_filter;
	
	fp32 yawMove_absolute;
	fp32 pitchMove_absolute;
	
	uint8_t enemy_detected; //0 NOT detected, 1 detected
	//Auto fire command; 0x00 = Hold, 0xff = Fire
	uint8_t shootCommand; //自动开火指令  0x00 = 停火  0xff = 开火
	//uint8_t cv_status; //0自瞄关闭, 1AID, 2LOCK
	/*
	0 no cmd; happens when cv offline; old: cv_status
	1 cmd for gimbal AID
	2 cmd for gimbal LOCK
	--6-25-2023修改: 由于辅助瞄准和绝对坐标瞄准同时发送 1 和 2会频繁切换, 但是0只在掉线时出现
	*/
	uint8_t cv_gimbal_sts;
	
	//UI related: pc_ui_msg_t
	uint16_t dis_raw;
	//after unit conversion; distance to a recognised enemy; = 0 when NO target detected
	fp32 dis;
	//after unit conversion; distance to the current barrel pointing target(pixel point)
	fp32 aim_pos_dis;
  /* -------------------- Var from cv comm - END -------------------- */
	
  /* -------------------- CV ON/OFF controled by operator -------------------- */
	//Auto aim ON/OFF switch status: 0-off; 1-auto aim AID; 2-auto aim LOCK
	uint8_t autoAimFlag;
	/* -------------------- CV ON/OFF controled by operator - END -------------------- */
}pc_info_t;
/*---------------------------------------------------- Processed Data - End Above ----------------------------------------------------*/

void init_pc_to_embed_Main_comm_struct_data(void);
void init_embed_to_pc_comm_struct_data(void);

void pc_comm_data_solve(uint8_t *frame);
void embed_send_data_to_pc_loop(void);

//declear getter method
fp32 get_yawMove_aid(uint8_t enable_not_detect_set_zero);
fp32 get_pitchMove_aid(uint8_t enable_not_detect_set_zero);
fp32 get_yawMove_absolute(void);
fp32 get_pitchMove_absolute(void);
bool_t is_enemy_detected_with_pc_toe(void);
bool_t is_enemy_detected(void);
uint8_t get_enemy_detected(void);
uint8_t get_shootCommand(void);
uint8_t get_cv_gimbal_sts(void);
fp32 get_aim_pos_dis(void);
uint8_t get_autoAimFlag(void);

//miniPC base control
void get_base_ctrl_vx_vy_wrt_gimbal(fp32* vx_out, fp32* vy_out);
fp32 get_base_ctrl_vx_wrt_gimbal(void);
fp32 get_base_ctrl_vy_wrt_gimbal(void);
fp32 get_base_ctrl_yaw_aid(void);

//declear setter method
void set_autoAimFlag(uint8_t autoAimFlag);

#endif
