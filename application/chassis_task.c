/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "gimbal_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "chassis_energy_regulate.h"
#include "lowpass_filter.h"
#include "linear_throttle.h"
#include "SuperCap_comm.h"
#include "referee_interact_task.h"
#include "referee.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

static void chassis_init(chassis_move_t *chassis_move_init);

static void chassis_set_mode(chassis_move_t *chassis_move_mode);

void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

static void chassis_feedback_update(chassis_move_t *chassis_move_update);

static void chassis_set_contorl(chassis_move_t *chassis_move_control);

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//底盘运动数据
chassis_move_t chassis_move;
uint32_t dial_ccw_set_val_time_chassis;
uint32_t dial_cw_set_val_time_chassis;

static void chassis_motor_PID_reset()
{		
		uint8_t i=0;
		for (i = 0; i < 4; i++)
    {
				chassis_move.motor_speed_pid[i].out = 0;
				chassis_move.motor_speed_pid[i].Pout = 0;
				chassis_move.motor_speed_pid[i].Iout = 0;
				chassis_move.motor_speed_pid[i].Dout = 0;
    }
}

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //底盘初始化
    chassis_init(&chassis_move);
    //make sure all chassis motor is online,
    //判断底盘电机是否都在线
		//
//    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
//    {
//        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//    }

    while (1)
    {
        //set chassis control mode
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //模式切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //set chassis control set-point 
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

        //make sure  one motor is online at least, so that the control CAN message can be received
        //确保至少一个电机在线， 这样CAN控制包可以被接收到
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //when remote control is offline, chassis motor should receive zero current. 
            //当遥控器掉线的时候，发送给底盘电机零电流.
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
                //send control current
                //发送控制电流
                CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            }
        }
				//控制函数, 专门用于超级电容的发送
				superCap_control_loop();
				
        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    
    //chassis angle PID
    //底盘角度pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning， chassis mode is raw 
    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_ZERO_FORCE;
		chassis_move_init->chassis_vector_mode = CHASSIS_VECTOR_RAW;
		
		// 底盘能量模式为 NORMAL
		chassis_move_init->chassis_energy_mode = CHASSIS_NORMAL;
		
    //get remote control point
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();//SZL 2-2-2022 从get_INS_angle_point改为现在这个
    //get gimbal motor data point
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //get chassis motor data point,  initialize motor speed PID
    //获取底盘电机数据指针，初始化PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    //first order low-pass filter  replace ramp function
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
		
		// 键盘用线性油门而不是低通滤波
		linear_throttle_init(&chassis_move_init->linear_throttle_vx, CHASSIS_CONTROL_TIME, LINEAR_THROTTLE_NORMAL_TARGET_SPEED, LINEAR_THROTTLE_NORMAL_INIT_SPEED);
		linear_throttle_init(&chassis_move_init->linear_throttle_vy, CHASSIS_CONTROL_TIME, LINEAR_THROTTLE_NORMAL_TARGET_SPEED, LINEAR_THROTTLE_NORMAL_INIT_SPEED);

    //max and min speed
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
		
//		//SZL 3-10-2022 Debug
//		debug_max_pwr = 80;
//		debug_fail_safe_pwr = 100;
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    
		//remote control  set chassis behaviour mode
    //遥控器设置模式 rc_data[TEMP].rc.switch_right
		
		// 右侧开关状态[下], 底盘云台设置为无力状态
    if (switch_is_down(chassis_move_mode->chassis_RC[TEMP].rc.switch_right))
		{
			chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
			chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_RAW;
		}
    // 右侧开关状态[中], 底盘云台设置为跟随状态
    else if (switch_is_mid(chassis_move_mode->chassis_RC[TEMP].rc.switch_right))
    {
			chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW;
			chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_SPEED;
    }
		// 右侧开关状态[上], 底盘云台设置为跟随状态
    else if (switch_is_up(chassis_move_mode->chassis_RC[TEMP].rc.switch_right))
    {
			// 仅当 不是跟随且不是小陀螺时, 将底盘模式置为跟随
			if (chassis_move_mode->last_chassis_mode != CHASSIS_ROTATE && chassis_move_mode->last_chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW)
			{
				chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
				chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_SPEED;
			}
			
			// 遥控器滚轮滤波时间
			if(chassis_move_mode->chassis_RC[TEMP].rc.dial > 500)
			{
				dial_ccw_set_val_time_chassis++;
			}
			else
			{
				dial_ccw_set_val_time_chassis = 0;
			}
			
			if(chassis_move_mode->chassis_RC[TEMP].rc.dial < -500)
			{
				dial_cw_set_val_time_chassis++;
			}
			else
			{
				dial_cw_set_val_time_chassis = 0;
			}
			
			if(dial_cw_set_val_time_chassis > 50)
			{
				chassis_move_mode->chassis_mode = CHASSIS_ROTATE_CW_INSPECTION;
				chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_SPEED;
			}
			else if(dial_ccw_set_val_time_chassis > 50)
			{
				chassis_move_mode->chassis_mode = CHASSIS_ROTATE_CCW_INSPECTION;
				chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_SPEED;
			}
			else
			{
				// 仅当 不是跟随且不是小陀螺时, 将底盘模式置为跟随
				if (chassis_move_mode->last_chassis_mode == CHASSIS_ROTATE_CCW_INSPECTION && chassis_move_mode->last_chassis_mode == CHASSIS_ROTATE_CW_INSPECTION)
				{
					chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
					chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_SPEED;
				}
			}

      // 按F启动小陀螺 press F to start chassis spin
      switch (chassis_move_mode->chassis_RC[TEMP].key[KEY_PRESS].f)
      {
				case 1:
					chassis_move_mode->chassis_mode = CHASSIS_ROTATE;
					chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_SPEED;
				break;

        default:
					// does nothings
				break;
			}

			// 按G关闭小陀螺 press G to stop chassis spin
			switch (chassis_move_mode->chassis_RC[TEMP].key[KEY_PRESS].g)
			{
				case 1:
					chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
					chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_SPEED;
				break;

				default:
					// does nothings
				break;
			}
		}

    //when gimbal in some mode, such as init mode, chassis must's move
    //当云台在某些模式下，像初始化， 底盘不动
    if (gimbal_cmd_to_chassis_stop())
    {
			chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
			chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_RAW;
    }
		
		// 当裁判系统在上线且指示chassis口断电时, 底盘设为无力
		if ( (!toe_is_error(REFEREE_TOE)) && (get_chassis_power_output_status() == 0))
		{
			chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;
			chassis_move_mode->chassis_vector_mode = CHASSIS_VECTOR_RAW;
		}
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    } else {
			// 刷新UI
			if (chassis_move_transit->last_chassis_mode >= CHASSIS_FOLLOW_GIMBAL_YAW || chassis_move_transit->chassis_mode >= CHASSIS_FOLLOW_GIMBAL_YAW)
			{
				set_interactive_flag_chassis_mode_flag(1);
			}
		}

    //change to follow gimbal angle mode
    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //change to no follow angle
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_NO_FOLLOW) && chassis_move_transit->chassis_mode == CHASSIS_NO_FOLLOW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
//    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
//    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
//    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
		// 2023 全向轮底盘 正运动学
		chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX / OWHE_ANG_INVK_COEF;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY / OWHE_ANG_INVK_COEF;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
		// 6-25-2023: 计算云台朝向的 vx vy
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		sin_yaw = arm_sin_f32(-chassis_move_update->chassis_yaw_motor->relative_angle);
    cos_yaw = arm_cos_f32(-chassis_move_update->chassis_yaw_motor->relative_angle);
		chassis_move_update->vx_gimbal_orientation = (cos_yaw * chassis_move_update->vx) - (sin_yaw * chassis_move_update->vy);
		chassis_move_update->vy_gimbal_orientation = (sin_yaw * chassis_move_update->vx) + (cos_yaw * chassis_move_update->vy);

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
		//SZL 2-3-2022更改
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_gimbal_angle_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_gimbal_angle_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_gimbal_angle_ROLL_ADDRESS_OFFSET);
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void DJI_rc_to_base_XY_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *vz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL || vz_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel, wz_channel;
    fp32 vx_set_channel, vy_set_channel, wz_set_channel; //遥控器摇杆通道
		fp32 vx_set_key, vy_set_key;
		fp32 vx_set_step, vy_set_step;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC[TEMP].rc.rocker_r1, vx_channel, 10); //CHASSIS_RC_DEADLINE
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC[TEMP].rc.rocker_r_, vy_channel, 10);
		rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC[TEMP].rc.rocker_l_, wz_channel, 10);

    //遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例 rocker value (max 660) change to speed (m/s)
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
		wz_set_channel = wz_channel * -CHASSIS_WZ_RC_SEN;
		
		// 开始处理按键的线性油门
		
		// 由底盆能量模式确定当前线性油门参数
		switch (chassis_move_rc_to_vector->chassis_energy_mode)
		{
			case CHASSIS_BOOST:
				chassis_move_rc_to_vector->linear_throttle_vx.abs_target = LINEAR_THROTTLE_BOOST_TARGET_SPEED;
				vx_set_step = LINEAR_THROTTLE_BOOST_STEP;
				chassis_move_rc_to_vector->linear_throttle_vx.abs_init = LINEAR_THROTTLE_BOOST_INIT_SPEED;
			break;
			
			case CHASSIS_NORMAL:
				chassis_move_rc_to_vector->linear_throttle_vx.abs_target = LINEAR_THROTTLE_NORMAL_TARGET_SPEED;
				vx_set_step = LINEAR_THROTTLE_NORMAL_STEP;
				chassis_move_rc_to_vector->linear_throttle_vx.abs_init = LINEAR_THROTTLE_NORMAL_INIT_SPEED;
			break;
			
			case CHASSIS_CHARGE:
				chassis_move_rc_to_vector->linear_throttle_vx.abs_target = LINEAR_THROTTLE_NORMAL_TARGET_SPEED;
				vx_set_step = LINEAR_THROTTLE_NORMAL_STEP;
				chassis_move_rc_to_vector->linear_throttle_vx.abs_init = LINEAR_THROTTLE_NORMAL_INIT_SPEED;
			break;

			default:
				// as normal
				chassis_move_rc_to_vector->linear_throttle_vx.abs_target = LINEAR_THROTTLE_NORMAL_TARGET_SPEED;
				vx_set_step = LINEAR_THROTTLE_NORMAL_STEP;
				chassis_move_rc_to_vector->linear_throttle_vx.abs_init = LINEAR_THROTTLE_NORMAL_INIT_SPEED;
			break;
		}
		// vy 始终为normal模式
		chassis_move_rc_to_vector->linear_throttle_vy.abs_target = LINEAR_THROTTLE_NORMAL_TARGET_SPEED;
		vy_set_step = LINEAR_THROTTLE_NORMAL_STEP;
		chassis_move_rc_to_vector->linear_throttle_vy.abs_init = LINEAR_THROTTLE_NORMAL_INIT_SPEED;
		
		// 先处理 x 方向 线性油门
		switch (chassis_move_rc_to_vector->chassis_RC[TEMP].key[KEY_PRESS].w)
		{
			case 0:
			  if(!chassis_move_rc_to_vector->chassis_RC[TEMP].key[KEY_PRESS].s)
				{
					linear_throttle_clear_out(&chassis_move_rc_to_vector->linear_throttle_vx);
					vx_set_key = 0.0f;
				}
			break;
			
			case 1:
				linear_throttle_calc(&chassis_move_rc_to_vector->linear_throttle_vx, vx_set_step);
			  vx_set_key = chassis_move_rc_to_vector->linear_throttle_vx.out;
			break;

			default:
				// does nothings
			break;
		}
		switch (chassis_move_rc_to_vector->chassis_RC[TEMP].key[KEY_PRESS].s)
		{
			case 0:
				if(!chassis_move_rc_to_vector->chassis_RC[TEMP].key[KEY_PRESS].w)
				{
					linear_throttle_clear_out(&chassis_move_rc_to_vector->linear_throttle_vx);
					vx_set_key = 0.0f;
				}
			break;
			
			case 1:
				linear_throttle_calc(&chassis_move_rc_to_vector->linear_throttle_vx, -vx_set_step);
			  vx_set_key = chassis_move_rc_to_vector->linear_throttle_vx.out;
			break;

			default:
				// does nothings
			break;
		}
		
		// 再处理 y 方向 线性油门
		switch (chassis_move_rc_to_vector->chassis_RC[TEMP].key[KEY_PRESS].a)
		{
			case 0:
				if(!chassis_move_rc_to_vector->chassis_RC[TEMP].key[KEY_PRESS].d)
				{
					linear_throttle_clear_out(&chassis_move_rc_to_vector->linear_throttle_vy);
					vy_set_key = 0.0f;
				}
			break;
			
			case 1:
				linear_throttle_calc(&chassis_move_rc_to_vector->linear_throttle_vy, vy_set_step);
			  vy_set_key = chassis_move_rc_to_vector->linear_throttle_vy.out;
			break;

			default:
				// does nothings
			break;
		}
		switch (chassis_move_rc_to_vector->chassis_RC[TEMP].key[KEY_PRESS].d)
		{
			case 0:
				if(!chassis_move_rc_to_vector->chassis_RC[TEMP].key[KEY_PRESS].a)
				{
					linear_throttle_clear_out(&chassis_move_rc_to_vector->linear_throttle_vy);
					vy_set_key = 0.0f;
				}
			break;
			
			case 1:
				linear_throttle_calc(&chassis_move_rc_to_vector->linear_throttle_vy, -vy_set_step);
			  vy_set_key = chassis_move_rc_to_vector->linear_throttle_vy.out;
			break;

			default:
				// does nothings
			break;
		}
		
		// 注意 目前的 vx_set_key vy_set_key 并未二次的去过低通滤波, 而是直接叠加再杆量上的

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入 only apply to X and Y
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out + vx_set_key;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out + vy_set_key;
		*vz_set = wz_set_channel; // 没有低通滤波
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
//根据底盘状态机 控制底盘
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 angle_set = 0.0f;
		fp32 rc_x = 0.0f, rc_y = 0.0f, rc_z = 0.0f;
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		
		// 在这里判断底盘功率调控 chassis energy regulate
		chassis_energy_regulate(chassis_move_control);
		
		// 从遥控器, 或键盘输入端, 获取vx, vy, wz等信息
		DJI_rc_to_base_XY_control_vector(&rc_x, &rc_y, &rc_z, chassis_move_control);
		
    //get three control set-point, 获取三个控制设置值 xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    //chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
		
		//
		switch (chassis_move_control->chassis_mode)
    {
			case CHASSIS_ZERO_FORCE: //同时reset pid
				chassis_move_control->vx_set=0.0f;
        chassis_move_control->vy_set=0.0f;
        chassis_move_control->wz_set=0.0f;
				chassis_motor_PID_reset();
      break;

      case CHASSIS_NO_MOVE: 
				chassis_move_control->vx_set=0.0f;
        chassis_move_control->vy_set=0.0f;
        chassis_move_control->wz_set=0.0f;
      break;
			
			case CHASSIS_OPEN:
				chassis_move_control->vx_set=0.0f;
				chassis_move_control->vy_set=0.0f;
				chassis_move_control->wz_set=0.0f;
			  //open for testing motor movements
      break;
			
			case CHASSIS_NO_FOLLOW:
				//"angle_set" is rotation speed set-point
        //“angle_set” 是旋转速度控制
        chassis_move_control->wz_set = rc_z;
        chassis_move_control->vx_set = fp32_constrain(rc_x, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(rc_y, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			break;
			
			case CHASSIS_FOLLOW_GIMBAL_YAW:
				//rotate chassis direction, make sure vertial direction follow gimbal 
				//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
				sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
				cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
				chassis_move_control->vx_set = cos_yaw * rc_x + sin_yaw * rc_y;
				chassis_move_control->vy_set = -sin_yaw * rc_x + cos_yaw * rc_y;
			
				angle_set = 0.0f; // TODO: 后期可改成两个角度, 但这是变符号的PI角度, 需要改wz_set的符号
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //calculate ratation speed
        //计算旋转PID角速度
			
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
	
      break;
				
			case CHASSIS_ROTATE:
				//rotate chassis direction, make sure vertial direction follow gimbal 
				//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
				sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
				cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
				chassis_move_control->vx_set = cos_yaw * rc_x + sin_yaw * rc_y;
				chassis_move_control->vy_set = -sin_yaw * rc_x + cos_yaw * rc_y;
			
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        
			  //set ratation speed
        //小陀螺 模式下 设置旋转角速度
				chassis_move_control->wz_set = chassis_move_control->spin_speed;
		
			break;
			
			case CHASSIS_ROTATE_CCW_INSPECTION:
				//rotate chassis direction, make sure vertial direction follow gimbal 
				//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
				sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
				cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
				chassis_move_control->vx_set = cos_yaw * rc_x + sin_yaw * rc_y;
				chassis_move_control->vy_set = -sin_yaw * rc_x + cos_yaw * rc_y;
			
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        
			  //set ratation speed
        //小陀螺 模式下 设置旋转角速度
				chassis_move_control->wz_set = RAD_PER_SEC_FROM_RPM(50);
			break;
			
			case CHASSIS_ROTATE_CW_INSPECTION:
				//rotate chassis direction, make sure vertial direction follow gimbal 
				//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
				sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
				cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
				chassis_move_control->vx_set = cos_yaw * rc_x + sin_yaw * rc_y;
				chassis_move_control->vy_set = -sin_yaw * rc_x + cos_yaw * rc_y;
			
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        
			  //set ratation speed
        //小陀螺 模式下 设置旋转角速度
				chassis_move_control->wz_set = -RAD_PER_SEC_FROM_RPM(50);
			break;
    
      default://CHASSIS_NO_MOVE

      break;
		}
		//

		if(chassis_move_control->chassis_vector_mode == CHASSIS_VECTOR_SPEED)
		{
			//speed limit
      //速度限幅
      chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
      chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			chassis_move_control->wz_set = chassis_move_control->wz_set;
		}
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
      //in raw mode, set-point is sent to CAN bus
      //在原始模式，设置值是发送到CAN总线 - 无限幅
			chassis_move_control->vx_set = chassis_move_control->vx_set;
      chassis_move_control->vy_set = chassis_move_control->vy_set;
			chassis_move_control->wz_set = chassis_move_control->wz_set;
      chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
      chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
  * @brief          four mecanum wheels speed is calculated by three param. 
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
//	  //SZL: 不管这个昂, 这个是官方底盘的代码
//    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
//    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
//    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	
//		//新的解算Matrix
//	  wheel_speed[0] = -vx_set - vy_set - (MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[1] = vx_set - vy_set - (MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[2] = vx_set + vy_set - (MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[3] = -vx_set + vy_set - (MOTOR_DISTANCE_TO_CENTER * wz_set);
	
		//全向轮 运动学结算矩阵
		wheel_speed[0] = ((-OWHE_ANG_INVK_COEF) * vx_set) - (OWHE_ANG_INVK_COEF * vy_set) - (MOTOR_DISTANCE_TO_CENTER * wz_set);
    wheel_speed[1] = (OWHE_ANG_INVK_COEF * vx_set)    - (OWHE_ANG_INVK_COEF * vy_set) - (MOTOR_DISTANCE_TO_CENTER * wz_set);
    wheel_speed[2] = (OWHE_ANG_INVK_COEF * vx_set)    + (OWHE_ANG_INVK_COEF * vy_set) - (MOTOR_DISTANCE_TO_CENTER * wz_set);
    wheel_speed[3] = ((-OWHE_ANG_INVK_COEF) * vx_set) + (OWHE_ANG_INVK_COEF * vy_set) - (MOTOR_DISTANCE_TO_CENTER * wz_set);
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //mecanum wheel speed calculation
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_vector_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //in raw mode, derectly return
        //raw控制直接返回
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

#if USE_SpeedAdaptiveChassisPowerControl
		//calculate pid and power control
//		speed_adaptive_chassis_power_control(chassis_move_control_loop); 测试
//		gen3_superCap_speed_adaptive_chassis_power_control(chassis_move_control_loop); 测试
		general_speed_adaptive_chassis_power_control(chassis_move_control_loop);

		
#else
    //calculate pid
    //计算pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }


    //功率控制power control
    //chassis_power_control_non_speed(chassis_move_control_loop);
#endif

    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}

const chassis_move_t *get_chassis_pointer(void)
{
	return &chassis_move;
}

/* ---------- chassis module getter method ---------- */
chassis_mode_e get_chassis_mode(void)
{
	return chassis_move.chassis_mode;
}

chassis_mode_e get_last_chassis_mode(void)
{
	return chassis_move.last_chassis_mode;
}

chassis_energy_mode_e get_chassis_energy_mode(void)
{
	return chassis_move.chassis_energy_mode;
}

chassis_energy_mode_e get_last_chassis_energy_mode(void)
{
	return chassis_move.last_chassis_energy_mode;
}
/* ---------- getter method end ---------- */
