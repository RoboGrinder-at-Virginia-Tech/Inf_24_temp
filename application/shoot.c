/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee_usart_task.h"

#include "miniPC_msg.h"
#include "prog_msg_utility.h"
#include "odometer_task.h"

#include "stdlib.h"

#include "bsp_buzzer.h"

// shootL: left barrel
#define shootL_fric1_on(pwm) L_barrel_fric1_on((pwm)) //left barrel 摩擦轮1pwm宏定义
#define shootL_fric2_on(pwm) L_barrel_fric2_on((pwm)) //left barrel 摩擦轮2pwm宏定义
#define shootL_fric_off()    L_barrel_fric_off()      //关闭两个摩擦轮

// shootR: right barrel
#define shootR_fric3_on(pwm) R_barrel_fric3_on((pwm)) //left barrel 摩擦轮1pwm宏定义
#define shootR_fric4_on(pwm) R_barrel_fric4_on((pwm)) //left barrel 摩擦轮2pwm宏定义
#define shootR_fric_off()    R_barrel_fric_off()      //关闭两个摩擦轮

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO 只是定义了
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)


//extern miniPC_info_t miniPC_info; //3-26-2023 update never use this again
extern osThreadId gimbalTaskHandle; //声明 用于挂起和恢复
extern osThreadId chassisTaskHandle; //声明 用于挂起和恢复

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          Left barrel 堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void L_barrel_trigger_motor_turn_back_17mm(void);

/**
  * @brief          Right barrel 堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void R_barrel_trigger_motor_turn_back_17mm(void);

/**
  * @brief          Left barrel 射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
//static void L_barrel_shoot_bullet_control_17mm(void);

//新的绝对角度控制 -左枪管
static void L_barrel_shoot_bullet_control_absolute_17mm(void); //单发
static void L_barrel_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq); //连发
/**
  * @brief          Right barrel 射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
//static void R_barrel_shoot_bullet_control_17mm(void);

//新的绝对角度控制 -右枪管
static void R_barrel_shoot_bullet_control_absolute_17mm(void); //单发
static void R_barrel_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq); //连发

//连发交替开火
static void L_R_barrel_alternate_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq, uint32_t phase_diff_ms);
static void L_R_barrel_alternate_shoot_bullet_control_17mm_timer_reset(uint32_t phase_diff_ms);

/*
尝试卡尔曼滤波
*/
static void snail_fric_wheel_kalman_adjustment(ramp_function_source_t *fric1, ramp_function_source_t *fric2);


uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat);

shoot_control_t shoot_control;          //射击数据


int16_t temp_rpm_left; // debug Jscope
int16_t temp_rpm_right; // debug Jscope

/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
		// left barrel trig pid init
    static const fp32 L_barrel_Trigger_speed_pid[3] = {L_BARREL_TRIGGER_SPEED_IN_PID_KP, L_BARREL_TRIGGER_SPEED_IN_PID_KI, L_BARREL_TRIGGER_SPEED_IN_PID_KD};//速度环
		//左侧枪管 外环 位置环PID
		static const fp32 L_barrel_Trigger_position_pid_17mm_outerLoop[3] = {L_BARREL_TRIGGER_ANGLE_PID_OUTER_KP, L_BARREL_TRIGGER_ANGLE_PID_OUTER_KI, L_BARREL_TRIGGER_ANGLE_PID_OUTER_KD};//位置环
		
		//right barrel trig pid init
		static const fp32 R_barrel_Trigger_speed_pid[3] = {R_BARREL_TRIGGER_SPEED_IN_PID_KP, R_BARREL_TRIGGER_SPEED_IN_PID_KI, R_BARREL_TRIGGER_SPEED_IN_PID_KD};//速度环
		//右侧枪管 外环 位置环PID
		static const fp32 R_barrel_Trigger_position_pid_17mm_outerLoop[3] = {R_BARREL_TRIGGER_ANGLE_PID_OUTER_KP, R_BARREL_TRIGGER_ANGLE_PID_OUTER_KI, R_BARREL_TRIGGER_ANGLE_PID_OUTER_KD};//位置环
		
    // shoot_control.shoot_mode = SHOOT_STOP;
		shoot_control.shoot_mode_L = SHOOT_STOP;
		shoot_control.shoot_mode_R = SHOOT_STOP;
		
		
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
//    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		shoot_control.shoot_motor_L_measure = get_trigger_motor_L_measure_point();
		shoot_control.shoot_motor_R_measure = get_trigger_motor_R_measure_point();
		
    //初始化PID
//		PID_init(&shoot_control.L_barrel_trigger_motor_pid, PID_POSITION, L_barrel_Trigger_speed_pid, L_BARREL_TRIGGER_READY_PID_MAX_OUT, L_BARREL_TRIGGER_READY_PID_MAX_IOUT);
//		PID_init(&shoot_control.R_barrel_trigger_motor_pid, PID_POSITION, R_barrel_Trigger_speed_pid, R_BARREL_TRIGGER_READY_PID_MAX_OUT, R_BARREL_TRIGGER_READY_PID_MAX_IOUT);
		shoot_PID_init(&shoot_control.L_barrel_trigger_motor_pid, SHOOT_PID_SEPARATED_INTEGRAL_IN_SPEED, L_barrel_Trigger_speed_pid, L_BARREL_TRIGGER_READY_PID_MAX_OUT, L_BARREL_TRIGGER_READY_PID_MAX_IOUT);
		shoot_PID_init(&shoot_control.R_barrel_trigger_motor_pid, SHOOT_PID_SEPARATED_INTEGRAL_IN_SPEED, R_barrel_Trigger_speed_pid, R_BARREL_TRIGGER_READY_PID_MAX_OUT, R_BARREL_TRIGGER_READY_PID_MAX_IOUT);
		
		//17mm外环PID
		shoot_PID_init(&shoot_control.L_barrel_trigger_motor_angle_pid, SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS, L_barrel_Trigger_position_pid_17mm_outerLoop, L_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_OUT, L_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_IOUT);
		shoot_PID_init(&shoot_control.R_barrel_trigger_motor_angle_pid, SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS, R_barrel_Trigger_position_pid_17mm_outerLoop, R_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_OUT, R_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_IOUT);

    //更新数据
    shoot_feedback_update();
		// left barrel ramp
    ramp_init(&shoot_control.L_barrel_fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_OFF, FRIC_OFF); //FRIC_DOWN, FRIC_OFF
    ramp_init(&shoot_control.L_barrel_fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_OFF, FRIC_OFF);
		// right barrel ramp
		ramp_init(&shoot_control.R_barrel_fric3_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_OFF, FRIC_OFF);
    ramp_init(&shoot_control.R_barrel_fric4_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_OFF, FRIC_OFF);
		
		// left barrel - control
		shoot_control.L_barrel_fric_pwm1 = FRIC_OFF;
		shoot_control.L_barrel_fric_pwm2 = FRIC_OFF;
		shoot_control.L_barrel_ecd_count = 0;
    shoot_control.L_barrel_angle = shoot_control.shoot_motor_L_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.L_barrel_given_current = 0;
    shoot_control.L_barrel_move_flag = 0;
    shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
    shoot_control.L_barrel_speed = 0.0f;
    shoot_control.L_barrel_speed_set = 0.0f;
    shoot_control.L_barrel_key_time = 0;
		
		// right barrel - control
		shoot_control.R_barrel_fric_pwm3 = FRIC_OFF;
		shoot_control.R_barrel_fric_pwm4 = FRIC_OFF;
		shoot_control.R_barrel_ecd_count = 0;
    shoot_control.R_barrel_angle = shoot_control.shoot_motor_R_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.R_barrel_given_current = 0;
    shoot_control.R_barrel_move_flag = 0;
    shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
    shoot_control.R_barrel_speed = 0.0f;
    shoot_control.R_barrel_speed_set = 0.0f;
    shoot_control.R_barrel_key_time = 0;
		
		
		/*12-28-2021 SZL add for 
		infantry pid shooter friction wheel LEFT and RIGHT
		Everything above keep the same as the old PWM shooter
		*/
//		//初始化基本射击参数 - not used for MD
//		shoot_control.currentLeft_speed_set = 0;
//		shoot_control.currentRight_speed_set = 0;
		// left barrel speed
		shoot_control.L_barrel_fric1_speed_set = 0;
		shoot_control.L_barrel_fric2_speed_set = 0;
		// right barrel speed
		shoot_control.R_barrel_fric3_speed_set = 0;
		shoot_control.R_barrel_fric4_speed_set = 0;
		
		shoot_control.currentLIM_shoot_speed_17mm = 0;
		
//		// NOT used for MD
//		//LEFT friction PID const init
//		static const fp32 Left_friction_speed_pid[3] = {M3508_LEFT_FRICTION_PID_KP, M3508_LEFT_FRICTION_PID_KI, M3508_LEFT_FRICTION_PID_KD};
//		//RIGHT friction PID const init
//		static const fp32 Right_friction_speed_pid[3] = {M3508_RIGHT_FRICTION_PID_KP, M3508_RIGHT_FRICTION_PID_KI, M3508_RIGHT_FRICTION_PID_KD};

//		//电机指针 M3508屁股 左右摩擦轮 - not used for MD
//		shoot_control.left_friction_motor_measure = get_left_friction_motor_measure_point();
//		shoot_control.right_friction_motor_measure = get_right_friction_motor_measure_point();
		
//		//初始化PID - not used for MD
//		PID_init(&shoot_control.left_fric_motor_pid, PID_POSITION, Left_friction_speed_pid, M3508_LEFT_FRICTION_PID_MAX_OUT, M3508_LEFT_FRICTION_PID_MAX_IOUT);
//		PID_init(&shoot_control.right_fric_motor_pid, PID_POSITION, Right_friction_speed_pid, M3508_RIGHT_FRICTION_PID_MAX_OUT, M3508_RIGHT_FRICTION_PID_MAX_IOUT);
		
		L_barrel_fric_off();
		R_barrel_fric_off();
		
		//设置油门 斜坡开启行程
		shoot_control.L_barrel_fric1_ramp.max_value = FRIC_OFF; //重复-初始化max与min
		shoot_control.L_barrel_fric1_ramp.min_value = FRIC_OFF; //重复-初始化max与min
		shoot_control.L_barrel_fric1_ramp.out = 0; //FRIC_OFF;//主要-覆盖掉out
		
		shoot_control.L_barrel_fric2_ramp.max_value = FRIC_OFF;
		shoot_control.L_barrel_fric2_ramp.min_value = FRIC_OFF;
		shoot_control.L_barrel_fric2_ramp.out = 0; //FRIC_OFF;
		
		shoot_control.R_barrel_fric3_ramp.max_value = FRIC_OFF;
		shoot_control.R_barrel_fric3_ramp.min_value = FRIC_OFF;
		shoot_control.R_barrel_fric3_ramp.out = 0; //FRIC_OFF;
		
		shoot_control.R_barrel_fric4_ramp.max_value = FRIC_OFF;
		shoot_control.R_barrel_fric4_ramp.min_value = FRIC_OFF;
		shoot_control.R_barrel_fric4_ramp.out = 0; //FRIC_OFF;
		
		//本地热量 - 左枪管 ID1
		get_shooter_id1_17mm_heat_limit_and_heat(&shoot_control.L_barrel_heat_limit, &shoot_control.L_barrel_heat);
		shoot_control.L_barrel_local_heat_limit = shoot_control.L_barrel_heat_limit; //通用 数据
		shoot_control.L_barrel_local_cd_rate = get_shooter_id1_17mm_cd_rate(); //通用 数据
		
		//本地热量 - 右枪管 ID2
		get_shooter_id2_17mm_heat_limit_and_heat(&shoot_control.R_barrel_heat_limit, &shoot_control.R_barrel_heat);
		shoot_control.R_barrel_local_heat_limit = shoot_control.R_barrel_heat_limit; //通用 数据
		shoot_control.R_barrel_local_cd_rate = get_shooter_id2_17mm_cd_rate(); //通用 数据
		
		//射频时间初始化
		shoot_control.L_barrel_last_tick = xTaskGetTickCount(); //使用RTOS时间源
		shoot_control.R_barrel_last_tick = xTaskGetTickCount(); //使用RTOS时间源
		shoot_control.L_barrel_alternate_shoot_last_tick = xTaskGetTickCount(); //使用RTOS时间源
		shoot_control.R_barrel_alternate_shoot_last_tick = xTaskGetTickCount(); //使用RTOS时间源
}

/*6-22-2023 12v稳压后 标定 - 没换同一套 snail
15m/s:
uint16_t new_fric_allms_debug_L1 = 1340; //1189; //NEW_FRIC_15ms;-1号对应: 看向前进方向, 左侧发射机构, 上面那个枪管
uint16_t new_fric_allms_debug_L2 = 1340; //1189; //NEW_FRIC_15ms;-2号对应: 看向前进方向, 左侧发射机构, 下面那个枪管
uint16_t new_fric_allms_debug_R3 = 1558;//1200; //1200;//NEW_FRIC_15ms;-3号对应: 看向前进方向, 右侧发射机构, 上面那个枪管
uint16_t new_fric_allms_debug_R4 = 1558; //1214; //1200;//NEW_FRIC_15ms;-4号对应: 看向前进方向, 右侧发射机构, 下面那个枪管

18m/s:
uint16_t new_fric_allms_debug_L1 = 1400; //-1号对应: 看向前进方向, 左侧发射机构, 上面那个枪管
uint16_t new_fric_allms_debug_L2 = 1400; //-2号对应: 看向前进方向, 左侧发射机构, 下面那个枪管
uint16_t new_fric_allms_debug_R3 = 1630;//-3号对应: 看向前进方向, 右侧发射机构, 上面那个枪管
uint16_t new_fric_allms_debug_R4 = 1630; //-4号对应: 看向前进方向, 右侧发射机构, 下面那个枪管

7-6-2023 12v稳压后, 换成同一套 snail之后:
15m/s:
uint16_t new_fric_allms_debug_L1_15ms = 1210; //-1号对应: 看向前进方向, 左侧发射机构, 上面那个枪管 1338
uint16_t new_fric_allms_debug_L2_15ms = 1210; //-2号对应: 看向前进方向, 左侧发射机构, 下面那个枪管 1338
uint16_t new_fric_allms_debug_R3_15ms = 1210; //-3号对应: 看向前进方向, 右侧发射机构, 上面那个枪管 1555
uint16_t new_fric_allms_debug_R4_15ms = 1210; //-4号对应: 看向前进方向, 右侧发射机构, 下面那个枪管 1563

*/
uint16_t new_fric_allms_debug_L1_15ms = 1210; //1220; //1230;//1338; //-1号对应: 看向前进方向, 左侧发射机构, 上面那个枪管 1338
uint16_t new_fric_allms_debug_L2_15ms = 1210; //1220; //1230;//1338; //-2号对应: 看向前进方向, 左侧发射机构, 下面那个枪管 1338
uint16_t new_fric_allms_debug_R3_15ms = 1210; //1220; //1230;//1558;//-3号对应: 看向前进方向, 右侧发射机构, 上面那个枪管 1555
uint16_t new_fric_allms_debug_R4_15ms = 1210; //1220; //1230;//1558; //-4号对应: 看向前进方向, 右侧发射机构, 下面那个枪管 1563

uint16_t new_fric_allms_debug_L1_18ms = 1230; //-1号对应: 看向前进方向, 左侧发射机构, 上面那个枪管
uint16_t new_fric_allms_debug_L2_18ms = 1230; //-2号对应: 看向前进方向, 左侧发射机构, 下面那个枪管
uint16_t new_fric_allms_debug_R3_18ms = 1230;//-3号对应: 看向前进方向, 右侧发射机构, 上面那个枪管
uint16_t new_fric_allms_debug_R4_18ms = 1230; //-4号对应: 看向前进方向, 右侧发射机构, 下面那个枪管

uint16_t new_fric_allms_debug_L1_30ms = 1240; //-1号对应: 看向前进方向, 左侧发射机构, 上面那个枪管 1800 1580
uint16_t new_fric_allms_debug_L2_30ms = 1240; //-2号对应: 看向前进方向, 左侧发射机构, 下面那个枪管 1580	
uint16_t new_fric_allms_debug_R3_30ms = 1240;//-3号对应: 看向前进方向, 右侧发射机构, 上面那个枪管
uint16_t new_fric_allms_debug_R4_30ms = 1240; //-4号对应: 看向前进方向, 右侧发射机构, 下面那个枪管
/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
	
//		//开始判断速度上限
//	  robot_Level = get_robot_level();

//	 	 if(robot_Level == 0){
//	  shoot_control.fric1_ramp.max_value = FRIC_LV1; //(_const)
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }else if(robot_Level == 1){
//	  shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }else if(robot_Level == 2){
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV2;
//    shoot_control.fric2_ramp.max_value = FRIC_LV2;
//	 }else if(robot_Level == 3){
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV3;
//    shoot_control.fric2_ramp.max_value = FRIC_LV3;
//	 }else{
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }
//		//以上为老版本的------------------------------------	 

//	----------------- 动态适应 射速 Seattle 比赛使用
	  if(toe_is_error(REFEREE_TOE))
    {
       shoot_control.referee_current_shooter_17mm_speed_limit = INITIAL_PROJECTILE_SPEED_LIMIT_17mm; 
    }
	  else
 	  {
			//TODO: 添加比大小 选择射速最小的那一档 ------------------------------------
 			 shoot_control.referee_current_shooter_17mm_speed_limit = min_uint16(get_shooter_id1_17mm_speed_limit(), get_shooter_id2_17mm_speed_limit());
	  }
	 
	  /*TODO 数据超出最大合理数值时的操作*/
	  if(shoot_control.referee_current_shooter_17mm_speed_limit > 35)
	  {
		  shoot_control.referee_current_shooter_17mm_speed_limit = 15;
	  }
		
	  //17mm 的两档  15
//	  shoot_control.referee_current_shooter_17mm_speed_limit = 15;//强制使其=.. 用于调试-----------------------------------------------------------------------------------------------
	  if(shoot_control.referee_current_shooter_17mm_speed_limit == 15)
	  {
		  shoot_control.currentLIM_shoot_speed_17mm = (fp32)(15 - 3.0);//待定 没用
		  shoot_control.predict_shoot_speed = 14.4f; //14.7 (7-1)shoot_control.currentLIM_shoot_speed_17mm + 2;//待定
		  /*1) 6-22-2023经过测试 14.7f
		  */
		  // snail 摩擦轮 预期速度 只作为目标数值参考 没用
		  shoot_control.L_barrel_fric1_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.L_barrel_fric2_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric3_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric4_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  
		  // 更新MD snail 摩擦轮的PWM上限
		  shoot_control.L_barrel_fric1_ramp.max_value_constant = new_fric_allms_debug_L1_15ms; //NEW_FRIC_15ms; //NEW_FRIC_15ms_higher
		  shoot_control.L_barrel_fric2_ramp.max_value_constant = new_fric_allms_debug_L2_15ms; //NEW_FRIC_15ms;
		  shoot_control.R_barrel_fric3_ramp.max_value_constant = new_fric_allms_debug_R3_15ms; //NEW_FRIC_15ms;
		  shoot_control.R_barrel_fric4_ramp.max_value_constant = new_fric_allms_debug_R4_15ms; //NEW_FRIC_15ms;
	  }
	  else if(shoot_control.referee_current_shooter_17mm_speed_limit == 18)
		{ //TODO: 按照上面那档修改
		  shoot_control.currentLIM_shoot_speed_17mm = (fp32)(18 - 4.5);// 没用
		  shoot_control.predict_shoot_speed = 16.5f; //17.5f (7-6) //shoot_control.currentLIM_shoot_speed_17mm + 3;
		  /*
		  1) 发给ZYZ那 17.5 测出来 17.5
		  */
		  // snail 摩擦轮 预期速度 只作为目标数值参考 没用
		  shoot_control.L_barrel_fric1_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.L_barrel_fric2_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric3_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric4_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  
		  // 更新MD snail 摩擦轮的PWM上限
		  shoot_control.L_barrel_fric1_ramp.max_value_constant = new_fric_allms_debug_L1_18ms; //NEW_FRIC_18ms; //NEW_FRIC_15ms_higher
		  shoot_control.L_barrel_fric2_ramp.max_value_constant = new_fric_allms_debug_L2_18ms;
		  shoot_control.R_barrel_fric3_ramp.max_value_constant = new_fric_allms_debug_R3_18ms;
		  shoot_control.R_barrel_fric4_ramp.max_value_constant = new_fric_allms_debug_R4_18ms;
	  }
		else if(shoot_control.referee_current_shooter_17mm_speed_limit == 30)
		{
			shoot_control.currentLIM_shoot_speed_17mm = (fp32)(18 - 4.5);// 没用
		  shoot_control.predict_shoot_speed = 24.5f;
		  /*
		  1) 发给ZYZ那 16.5 测出来 16.5
		  */
		  // snail 摩擦轮 预期速度 只作为目标数值参考 没用
		  shoot_control.L_barrel_fric1_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.L_barrel_fric2_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric3_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric4_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  
		  // 更新MD snail 摩擦轮的PWM上限
		  shoot_control.L_barrel_fric1_ramp.max_value_constant = new_fric_allms_debug_L1_30ms; //NEW_FRIC_18ms; //NEW_FRIC_15ms_higher
		  shoot_control.L_barrel_fric2_ramp.max_value_constant = new_fric_allms_debug_L2_30ms;
		  shoot_control.R_barrel_fric3_ramp.max_value_constant = new_fric_allms_debug_R3_30ms;
		  shoot_control.R_barrel_fric4_ramp.max_value_constant = new_fric_allms_debug_R4_30ms;
		}
	  else
	  {//默认射速15
		  shoot_control.currentLIM_shoot_speed_17mm = (fp32)(15 - 3.0);//待定 没用
		  shoot_control.predict_shoot_speed = 14.4f; //shoot_control.currentLIM_shoot_speed_17mm + 2;//待定

		  // snail 摩擦轮 预期速度 只作为目标数值参考 没用
		  shoot_control.L_barrel_fric1_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.L_barrel_fric2_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric3_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  shoot_control.R_barrel_fric4_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
		  
		  // 更新MD snail 摩擦轮的PWM上限
		  shoot_control.L_barrel_fric1_ramp.max_value_constant = new_fric_allms_debug_L1_15ms; //NEW_FRIC_15ms; //NEW_FRIC_15ms_higher
		  shoot_control.L_barrel_fric2_ramp.max_value_constant = new_fric_allms_debug_L2_15ms; //NEW_FRIC_15ms;
		  shoot_control.R_barrel_fric3_ramp.max_value_constant = new_fric_allms_debug_R3_15ms; //NEW_FRIC_15ms;
		  shoot_control.R_barrel_fric4_ramp.max_value_constant = new_fric_allms_debug_R4_15ms; //NEW_FRIC_15ms;
	  }
//	----------------- 动态适应 射速 Seattle 比赛使用 -- end --
		
//  ---------- 动态适应 射频与交替开火相位差 Seattle 比赛使用
	  if(toe_is_error(REFEREE_TOE))
    {
			 shoot_control.local_cd_rate_min = LOCAL_CD_RATE_SAFE_VAL; //get_shooter_id2_17mm_cd_rate(); get_shooter_id1_17mm_cd_rate();
			 shoot_control.shoot_freq_set = 6; //默认射频6
			 shoot_control.phase_diff_ms_set = 50; //默认50ms相位差
			 shoot_control.local_shoot_heat_remain_value_var_set = LOCAL_SHOOT_HEAT_REMAIN_VALUE; //默认预留值
    }
	  else
 	  {
			 shoot_control.local_cd_rate_min = min_uint16( get_shooter_id1_17mm_cd_rate(), get_shooter_id2_17mm_cd_rate() ); //最小的那个冷却值
			 
			 //PRE: referee_current_shooter_17mm_speed_limit确定是两根枪管的最小值
			 //更具弹速 和 cd值来确定 射频和相位差 - 和上面判断分开, 没有耦合
			 if(shoot_control.referee_current_shooter_17mm_speed_limit == 30)
			 {//一定是弹速优先了
				 if(shoot_control.local_cd_rate_min == 35)
				 { //弹速优先 35cd 三级步兵
						shoot_control.shoot_freq_set = 6; //射频
						shoot_control.phase_diff_ms_set = 50; //相位差
					  shoot_control.local_shoot_heat_remain_value_var_set = 20;
				 }
				 else if(shoot_control.local_cd_rate_min == 25)
				 { //弹速优先 25cd 二级步兵
					  shoot_control.shoot_freq_set = 6; //射频
						shoot_control.phase_diff_ms_set = 50; //相位差
					  shoot_control.local_shoot_heat_remain_value_var_set = 20;
				 }
				 else
				 { //弹速优先 一级步兵
					 // 默认 + 15cd
					  shoot_control.shoot_freq_set = 6; //射频
						shoot_control.phase_diff_ms_set = 50; //相位差
					  shoot_control.local_shoot_heat_remain_value_var_set = 20;
					 
				 }
			 }
			 else if(shoot_control.referee_current_shooter_17mm_speed_limit == 18)
			 {
				 if(shoot_control.local_cd_rate_min == 80)
				 { //冷却优先 80cd 三级步兵
					 shoot_control.shoot_freq_set = 12; //射频 - 未来可以更高
					 shoot_control.phase_diff_ms_set = 20; //相位差
					 shoot_control.local_shoot_heat_remain_value_var_set = 35;
				 }
				 else
				 { //冷却优先 60cd 二级步兵
					 //默认 + 60cd
					 shoot_control.shoot_freq_set = 12; //射频 - 刚好一边6cd
					 shoot_control.phase_diff_ms_set = 20; //相位差
					 shoot_control.local_shoot_heat_remain_value_var_set = 35;
				 }
				 
			 }
			 else
			 {
				 //默认 + 15m/s 这一档
				 if(shoot_control.local_cd_rate_min == 40)
				 { //冷却优先 40cd 一级步兵
					 shoot_control.shoot_freq_set = 8; //射频 - 刚好一边4cd
					 shoot_control.phase_diff_ms_set = 50; //相位差
					 shoot_control.local_shoot_heat_remain_value_var_set = 25;
				 }
				 else if(shoot_control.local_cd_rate_min == 35)
				 { //爆发优先 35cd 三级步兵
					 shoot_control.shoot_freq_set = 12; //射频 - 可能调整
					 shoot_control.phase_diff_ms_set = 20; //相位差
					 shoot_control.local_shoot_heat_remain_value_var_set = 35;
				 }
				 else if(shoot_control.local_cd_rate_min == 25)
				 { //爆发优先 25cd 二级步兵
					 shoot_control.shoot_freq_set = 12; //射频 - 可能调整
					 shoot_control.phase_diff_ms_set = 20; //相位差
					 shoot_control.local_shoot_heat_remain_value_var_set = 35;
				 }
				 else
				 { //爆发优先 15cd 一级步兵
					 // 默认 + 15cd
					 shoot_control.shoot_freq_set = 12; //射频
					 shoot_control.phase_diff_ms_set = 20; //10; //相位差
					 shoot_control.local_shoot_heat_remain_value_var_set = 35;
				 }
			 }
	  }
		
//		//9-30 1v1 event
//		shoot_control.shoot_freq_set = 8; //射频
//		shoot_control.phase_diff_ms_set = 50; //相位差
//		shoot_control.local_shoot_heat_remain_value_var_set = 30;
		
		/*射频 12 相位差 10 or 20 没问题 - 推测 10 相位差 50 没问题
		local_shoot_heat_remain_value_var_set =35->12射频; =25->8射频; =20->6射频
		*/
//  ---------- 动态适应 射频与交替开火相位差 Seattle 比赛使用 -- end --
		
		// 先判断是 交替发射
		if( (shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET) && (shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET) )
		{
				/* 6-16-2023注释: 射频4使用(4, 100); 射频8使用(8, 50)
					 射频 根据选择的模式 自适应
				*/
//			  L_R_barrel_alternate_shoot_bullet_control_continuous_17mm(8, 50); //6, 50
				L_R_barrel_alternate_shoot_bullet_control_continuous_17mm(shoot_control.shoot_freq_set, shoot_control.phase_diff_ms_set);
		}
		
		// 先处理 left barrel的 FSM
    if (shoot_control.shoot_mode_L == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.L_barrel_speed_set = 0; // .speed_set = 0;
				
			  //一直重置PID - 防止累计误差
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_pid);
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_angle_pid);
			
				//初始化第一次PID帧的计算
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
				shoot_control.L_barrel_speed_set = shoot_control.L_barrel_speed;
    }
    else if (shoot_control.shoot_mode_L == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.L_barrel_speed_set = 0; // .speed_set = 0;
				//第一次重置PID
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_pid);
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_angle_pid);
			
				//初始化第一次PID帧的计算
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
				shoot_control.L_barrel_speed_set = shoot_control.L_barrel_speed;
    }
    else if(shoot_control.shoot_mode_L ==SHOOT_READY_BULLET)
    {
        shoot_control.L_barrel_trigger_speed_set = 0.0f; //.trigger_speed_set
        shoot_control.L_barrel_speed_set = 0.0f; //.speed_set
        //这个if 并不是 基本没啥用
        shoot_control.L_barrel_trigger_motor_pid.max_out = L_BARREL_TRIGGER_READY_PID_MAX_OUT;
        shoot_control.L_barrel_trigger_motor_pid.max_iout = L_BARREL_TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode_L == SHOOT_READY)
    {
				//shoot_control.trigger_speed_set = 0.0f;//------------
        //设置拨弹轮的速度
         shoot_control.L_barrel_speed_set = 0.0f; // .speed_set
    }
    else if (shoot_control.shoot_mode_L == SHOOT_BULLET)
    {
        shoot_control.L_barrel_trigger_motor_pid.max_out = L_BARREL_TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.L_barrel_trigger_motor_pid.max_iout = L_BARREL_TRIGGER_BULLET_PID_MAX_IOUT;
//        L_barrel_shoot_bullet_control_17mm();
				L_barrel_shoot_bullet_control_absolute_17mm();
    }
    else if (shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET)
    {
//        //设置拨弹轮的拨动速度,并开启堵转反转处理 5-31-2023前老代码
//        shoot_control.L_barrel_trigger_speed_set = CONTINUE_TRIGGER_SPEED_L;
//        L_barrel_trigger_motor_turn_back_17mm();
			  shoot_control.L_barrel_trigger_motor_pid.max_out = L_BARREL_TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.L_barrel_trigger_motor_pid.max_iout = L_BARREL_TRIGGER_BULLET_PID_MAX_IOUT;
				L_barrel_shoot_bullet_control_continuous_17mm(5); //10 4
    }
    else if(shoot_control.shoot_mode_L == SHOOT_DONE)
    {
        shoot_control.L_barrel_speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode_L == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.L_barrel_given_current = 0; // .given_current
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
//        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
//        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
			
				shoot_control.L_barrel_fric_pwm1 = FRIC_OFF; //.fric_pwm1
				shoot_control.L_barrel_fric_pwm2 = FRIC_OFF; //.fric_pwm2
				//关闭不需要斜坡关闭
			
//				//更改斜坡数据 挪到初始化中
//				shoot_control.L_barrel_fric1_ramp.max_value = FRIC_OFF;
//				shoot_control.L_barrel_fric1_ramp.min_value = FRIC_OFF;
//				shoot_control.L_barrel_fric1_ramp.out = FRIC_OFF;
//			
//				shoot_control.L_barrel_fric2_ramp.max_value = FRIC_OFF;
//				shoot_control.L_barrel_fric2_ramp.min_value = FRIC_OFF;
//				shoot_control.L_barrel_fric2_ramp.out = FRIC_OFF;
			
			
//			//SZL添加, 也可以使用斜波开启 低通滤波 //NOT USED for MD
//			shoot_control.currentLeft_speed_set = M3508_FRIC_STOP;
//			shoot_control.currentRight_speed_set = M3508_FRIC_STOP;
    }
    else
    {
        shoot_laser_on(); //激光开启
			
				
				//6-6-2023增加串级PID----
			  if(shoot_control.L_barrel_block_flag == 0)
				{ //退弹不用串级PID
//					shoot_control.speed_set = PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.angle, shoot_control.set_angle);
					shoot_control.L_barrel_speed_set = shoot_PID_calc(&shoot_control.L_barrel_trigger_motor_angle_pid, shoot_control.L_barrel_angle, shoot_control.L_barrel_set_angle);
        }
				
        //计算拨弹轮电机PID
//				PID_calc(&shoot_control.L_barrel_trigger_motor_pid, shoot_control.L_barrel_speed, shoot_control.L_barrel_speed_set);
        shoot_PID_calc(&shoot_control.L_barrel_trigger_motor_pid, shoot_control.L_barrel_speed, shoot_control.L_barrel_speed_set);
        
#if TRIG_MOTOR_TURN_LEFT_BARREL
				shoot_control.L_barrel_given_current = -(int16_t)(shoot_control.L_barrel_trigger_motor_pid.out);
#else
				shoot_control.L_barrel_given_current = (int16_t)(shoot_control.L_barrel_trigger_motor_pid.out);
#endif
        if(shoot_control.shoot_mode_L < SHOOT_READY_BULLET)
        {
            shoot_control.L_barrel_given_current = 0;
        }
				// TODO: 卡尔曼滤波 结合裁判系统返回子弹速度 调整摩擦轮速度 也就是调整ramp.max_value
				snail_fric_wheel_kalman_adjustment(&shoot_control.L_barrel_fric1_ramp, &shoot_control.L_barrel_fric2_ramp);
				
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.L_barrel_fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE); //.fric1_ramp
        ramp_calc(&shoot_control.L_barrel_fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE); //.fric2_ramp
				// Update fric PWM
				shoot_control.L_barrel_fric_pwm1 = (uint16_t)(shoot_control.L_barrel_fric1_ramp.out);// + 19); //.fric_pwm1 .fric1_ramp
				shoot_control.L_barrel_fric_pwm2 = (uint16_t)(shoot_control.L_barrel_fric2_ramp.out);   //.fric_pwm2 .fric2_ramp
				
//				//SZL添加, 也可以使用斜波开启 低通滤波 //NOT USED for MD
//				shoot_control.currentLeft_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
//				shoot_control.currentRight_speed_set = shoot_control.currentLIM_shoot_speed_17mm;

    }
		// ------------------------------ Left Right 分割线 ------------------------------
		// 处理 right barrel 的FSM
    if (shoot_control.shoot_mode_R == SHOOT_STOP) // shoot_mode_R
    {
        //设置拨弹轮的速度
        shoot_control.R_barrel_speed_set = 0; //.speed_set
			
			  //一直重置PID - 防止累计误差
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_pid);
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_angle_pid);
			
				//初始化第一次PID帧的计算
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				shoot_control.R_barrel_speed_set = shoot_control.R_barrel_speed;
    }
    else if (shoot_control.shoot_mode_R == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.R_barrel_speed_set = 0; //.speed_set
			
				//第一次重置PID
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_pid);
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_angle_pid);
			
				//初始化第一次PID帧的计算
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				shoot_control.R_barrel_speed_set = shoot_control.R_barrel_speed;
    }
    else if(shoot_control.shoot_mode_R ==SHOOT_READY_BULLET)
    {
        shoot_control.R_barrel_trigger_speed_set = 0.0f; //.trigger_speed_set
        shoot_control.R_barrel_speed_set = 0.0f; //.speed_set
        //这个if 并不是 基本没啥用
        shoot_control.R_barrel_trigger_motor_pid.max_out = R_BARREL_TRIGGER_READY_PID_MAX_OUT;
        shoot_control.R_barrel_trigger_motor_pid.max_iout = R_BARREL_TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode_R == SHOOT_READY)
    {
				//shoot_control.trigger_speed_set = 0.0f;//------------
        //设置拨弹轮的速度
         shoot_control.R_barrel_speed_set = 0.0f; //.speed_set
    }
    else if (shoot_control.shoot_mode_R == SHOOT_BULLET)
    {
        shoot_control.R_barrel_trigger_motor_pid.max_out = R_BARREL_TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.R_barrel_trigger_motor_pid.max_iout = R_BARREL_TRIGGER_BULLET_PID_MAX_IOUT;
        R_barrel_shoot_bullet_control_absolute_17mm();
    }
    else if (shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
    {
//        //设置拨弹轮的拨动速度,并开启堵转反转处理 5-31-2023前老代码
//        shoot_control.R_barrel_trigger_speed_set = CONTINUE_TRIGGER_SPEED_R; //.trigger_speed_set
//        R_barrel_trigger_motor_turn_back_17mm();
			  shoot_control.R_barrel_trigger_motor_pid.max_out = R_BARREL_TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.R_barrel_trigger_motor_pid.max_iout = R_BARREL_TRIGGER_BULLET_PID_MAX_IOUT;
			  R_barrel_shoot_bullet_control_continuous_17mm(5);
    }
    else if(shoot_control.shoot_mode_R == SHOOT_DONE)
    {
        shoot_control.R_barrel_speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode_R == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.R_barrel_given_current = 0;
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
//        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
//        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
			
				shoot_control.R_barrel_fric_pwm3 = FRIC_OFF; //.fric_pwm1
				shoot_control.R_barrel_fric_pwm4 = FRIC_OFF; //.fric_pwm2
				//关闭不需要斜坡关闭
			
//				//更改斜坡数据 挪到初始化中
//				shoot_control.R_barrel_fric1_ramp.max_value = FRIC_OFF;
//				shoot_control.R_barrel_fric1_ramp.min_value = FRIC_OFF;
//				shoot_control.R_barrel_fric1_ramp.out = FRIC_OFF;
//			
//				shoot_control.R_barrel_fric2_ramp.max_value = FRIC_OFF;
//				shoot_control.R_barrel_fric2_ramp.min_value = FRIC_OFF;
//				shoot_control.R_barrel_fric2_ramp.out = FRIC_OFF;
			
//			//SZL添加, 也可以使用斜波开启 低通滤波 //NOT USED for MD
//			shoot_control.currentLeft_speed_set = M3508_FRIC_STOP;
//			shoot_control.currentRight_speed_set = M3508_FRIC_STOP;
    }
    else
    {
        shoot_laser_on(); //激光开启
			
				//6-6-2023增加串级PID
			  if(shoot_control.R_barrel_block_flag == 0)
				{ //退弹不用串级PID
//					shoot_control.speed_set = PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.angle, shoot_control.set_angle);
					shoot_control.R_barrel_speed_set = shoot_PID_calc(&shoot_control.R_barrel_trigger_motor_angle_pid, shoot_control.R_barrel_angle, shoot_control.R_barrel_set_angle);
        }
				
        //计算拨弹轮电机PID
//        PID_calc(&shoot_control.R_barrel_trigger_motor_pid, shoot_control.R_barrel_speed, shoot_control.R_barrel_speed_set);
				shoot_PID_calc(&shoot_control.R_barrel_trigger_motor_pid, shoot_control.R_barrel_speed, shoot_control.R_barrel_speed_set);
        
#if TRIG_MOTOR_TURN_RIGHT_BARREL
				shoot_control.R_barrel_given_current = -(int16_t)(shoot_control.R_barrel_trigger_motor_pid.out);
#else
				shoot_control.R_barrel_given_current = (int16_t)(shoot_control.R_barrel_trigger_motor_pid.out);
#endif
        if(shoot_control.shoot_mode_R < SHOOT_READY_BULLET)
        {
            shoot_control.R_barrel_given_current = 0;
        }
				// TODO: 卡尔曼滤波 结合裁判系统返回子弹速度 调整摩擦轮速度 也就是调整ramp.max_value
				snail_fric_wheel_kalman_adjustment(&shoot_control.R_barrel_fric3_ramp, &shoot_control.R_barrel_fric4_ramp);
				
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.R_barrel_fric3_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.R_barrel_fric4_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
				// Update fric PWM
				shoot_control.R_barrel_fric_pwm3 = (uint16_t)(shoot_control.R_barrel_fric3_ramp.out);
				shoot_control.R_barrel_fric_pwm4 = (uint16_t)(shoot_control.R_barrel_fric4_ramp.out);
				
//				//SZL添加, 也可以使用斜波开启 低通滤波 //NOT USED for MD
//				shoot_control.currentLeft_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
//				shoot_control.currentRight_speed_set = shoot_control.currentLIM_shoot_speed_17mm;

    }
		// left and right barrel FSM 处理完成; 以下开始 实际输出
//		// 挪到了if状态判断里面
//		if((shoot_control.shoot_mode_L != SHOOT_STOP) || (shoot_control.shoot_mode_R != SHOOT_STOP))
//		{
//			shoot_control.L_barrel_fric_pwm1 = (uint16_t)(shoot_control.L_barrel_fric1_ramp.out);// + 19); //.fric_pwm1 .fric1_ramp
//			shoot_control.L_barrel_fric_pwm2 = (uint16_t)(shoot_control.L_barrel_fric2_ramp.out);   //.fric_pwm2 .fric2_ramp
//			
//			shoot_control.R_barrel_fric_pwm1 = (uint16_t)(shoot_control.R_barrel_fric1_ramp.out);
//			shoot_control.R_barrel_fric_pwm2 = (uint16_t)(shoot_control.R_barrel_fric2_ramp.out);
//		}
		
		
		// set the calculated final pwm to TIM, actually control the motor
		L_barrel_fric1_on(shoot_control.L_barrel_fric_pwm1);
		L_barrel_fric2_on(shoot_control.L_barrel_fric_pwm2);
		R_barrel_fric3_on(shoot_control.R_barrel_fric_pwm3);
		R_barrel_fric4_on(shoot_control.R_barrel_fric_pwm4);
		
//		//NOT USED for MD
//		M3508_fric_wheel_spin_control(-shoot_control.currentLeft_speed_set, shoot_control.currentRight_speed_set);
		
    return shoot_control.L_barrel_given_current; // only return L_barrel_given_current. All other store in global variable 
}




/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
/*
关于摩擦轮状态机的切换流程: 上拨一次 SHOOT_READY_FRIC; 再拨一次 SHOOT_STOP;
斜坡启动 换启动 之前几秒钟是缓启动 缓输出; 当斜坡到MAX时 即完成预热

*/
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode_L == SHOOT_STOP))
    {
        shoot_control.shoot_mode_L = SHOOT_READY_FRIC;//上拨一次开启摩擦轮
			  shoot_control.shoot_mode_R = SHOOT_READY_FRIC;
			  
			  shoot_control.user_fire_ctrl = user_SHOOT_BOTH;//开启摩擦轮 默认auto
			  shoot_control.key_Q_cnt = 2;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode_L != SHOOT_STOP))
    {
        shoot_control.shoot_mode_L = SHOOT_STOP;//上拨一次再关闭摩擦轮
			  shoot_control.shoot_mode_R = SHOOT_STOP;
			  shoot_control.key_Q_cnt = 0;
    }
				
    //处于中档， 可以使用键盘开启摩擦轮
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode_L == SHOOT_STOP)
    {
        shoot_control.shoot_mode_L = SHOOT_READY_FRIC; 
				shoot_control.shoot_mode_R = SHOOT_READY_FRIC; 
				shoot_control.user_fire_ctrl = user_SHOOT_AUTO;//开启摩擦轮 默认auto
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode_L != SHOOT_STOP)
    {
        shoot_control.shoot_mode_L = SHOOT_STOP;
				shoot_control.shoot_mode_R = SHOOT_STOP;
			  shoot_control.key_Q_cnt = 0;
    }

		//处于中档时的 按键Q 按下检测 即 用户火控状态 模式判断
		if(switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && (shoot_control.shoot_mode_L > SHOOT_STOP))
		{
				//shoot_control.key_Q_cnt++;
				if(shoot_control.last_key_Q_sts == 0)
				{
					shoot_control.key_Q_cnt++;
					//shoot_control.shoot_mode = SHOOT_READY;
					shoot_control.last_key_Q_sts = 1;
				}
				else
				{
					shoot_control.last_key_Q_sts = 1;
				}
		}
		else
		{
			 shoot_control.last_key_Q_sts = 0;
		}
		
		if(shoot_control.key_Q_cnt > 1) //4
		{
			shoot_control.key_Q_cnt = 1;//实现 周期性
		}
		
		if(shoot_control.key_Q_cnt == 1)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_AUTO;
		}
		else if(shoot_control.key_Q_cnt == 2)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_BOTH;
		}
		else if(shoot_control.key_Q_cnt == 3)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_L_CONT;
		}
		else if(shoot_control.key_Q_cnt == 4)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_R_CONT;
		}
		else if(shoot_control.key_Q_cnt == 0)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_OFF;
		}
		//---------Q按键计数以及相关检测结束---------
		/*这里是对老DJI开源代码的兼容 - 通过按键 低通滤波值 之前的shoot_mode, 前面有(按键<-map->user_fire_ctrl);
			先对当前 shoot_mode 赋值一次(按键其它<-map->shoot_mode), 后面根据user_fire_ctrl会给shoot_mode赋值第二次(user_fire_mode<-map->shoot_mode) - 是因为shoot_mode切换很快, 控制会直接用这个状态机
			实现多个user_fire_ctrl映射到有限个shoot_mode - 按键扫描还可以优化*/
		// left barrel related FSM, 先处理
    if(shoot_control.shoot_mode_L == SHOOT_READY_FRIC && shoot_control.L_barrel_fric1_ramp.out == shoot_control.L_barrel_fric1_ramp.max_value && shoot_control.L_barrel_fric2_ramp.out == shoot_control.L_barrel_fric2_ramp.max_value)
    {
        shoot_control.shoot_mode_L = SHOOT_READY_BULLET; //当摩擦轮完成预热 //A
    }
    else if(shoot_control.shoot_mode_L == SHOOT_READY_BULLET) //&& shoot_control.key == SWITCH_TRIGGER_ON)
    {
			shoot_control.shoot_mode_L = SHOOT_READY;  //shoot_control.key被默认初始化为0 所以:第一次会进入A 第二次会进入这儿 使得shoot_mode = SHOOT_READY
    }
    else if(0) //shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_L = SHOOT_READY_BULLET;//从不会进入这个else if
    }
    else if(shoot_control.shoot_mode_L == SHOOT_READY)
    {
			if(shoot_control.trigger_motor17mm_L_is_online)//发射机构断电时, shoot_mode状态机不会被置为发射相关状态
			{
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode_L = SHOOT_BULLET;
        }
			}
    }
    else if(shoot_control.shoot_mode_L == SHOOT_DONE)
    {
        shoot_control.L_barrel_key_time++; // .key_time++;
				//微动开关 抖动时间到了之后, 再弄成SHOOT_READY_BULLET
				//现在是 缓冲时间
        if(shoot_control.L_barrel_key_time > SHOOT_DONE_KEY_OFF_TIME)
        {
            shoot_control.L_barrel_key_time = 0;
            shoot_control.shoot_mode_L = SHOOT_READY_BULLET;
        }
    }
		
		// right barrel related FSM, 后处理
		if(shoot_control.shoot_mode_R == SHOOT_READY_FRIC && shoot_control.R_barrel_fric3_ramp.out == shoot_control.R_barrel_fric3_ramp.max_value && shoot_control.R_barrel_fric4_ramp.out == shoot_control.R_barrel_fric4_ramp.max_value)
    {
        shoot_control.shoot_mode_R = SHOOT_READY_BULLET; //当摩擦轮完成预热 //A
    }
    else if(shoot_control.shoot_mode_R == SHOOT_READY_BULLET) //&& shoot_control.key == SWITCH_TRIGGER_ON)
    {
			shoot_control.shoot_mode_R = SHOOT_READY;  //shoot_control.key被默认初始化为0 所以:第一次会进入A 第二次会进入这儿 使得shoot_mode = SHOOT_READY
    }
    else if(0) //shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_R = SHOOT_READY_BULLET;//从不会进入这个else if
    }
    else if(shoot_control.shoot_mode_R == SHOOT_READY)
    {
			if(shoot_control.trigger_motor17mm_R_is_online)//发射机构断电时, shoot_mode状态机不会被置为发射相关状态
			{
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode_R = SHOOT_BULLET;
        }
			}
    }
    else if(shoot_control.shoot_mode_R == SHOOT_DONE)
    {
        shoot_control.R_barrel_key_time++; //.key_time++; //key_time
				//微动开关 抖动时间到了之后, 再弄成SHOOT_READY_BULLET
				//现在是 缓冲时间
        if(shoot_control.R_barrel_key_time > SHOOT_DONE_KEY_OFF_TIME)
        {
            shoot_control.R_barrel_key_time = 0;
            shoot_control.shoot_mode_R = SHOOT_READY_BULLET;
        }
    }
		
/*
    if(shoot_control.shoot_mode > SHOOT_READY_FRIC){ //自动开火指令处理
		   if(shootCommand == 0xff){
			 shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			 }else if(shootCommand == 0x00){
			 shoot_control.shoot_mode = SHOOT_READY_BULLET;
			 }
		}
	*/

		/*更改自瞄开启逻辑  X按键计数*/
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
		{
			if(shoot_control.last_key_X_sts == 0)
			{
				shoot_control.key_X_cnt++;
				shoot_control.last_key_X_sts = 1;
			}
			else
			{
				shoot_control.last_key_X_sts = 1;
			}
		}
		else
		{
			shoot_control.last_key_X_sts = 0;
		}
		
		if(shoot_control.key_X_cnt > 2)
		{
			shoot_control.key_X_cnt = 1;//实现 周期性
		}
		//press X to turn on auto aim, 1=aid 2=lock 
		//或 即按键只能开启aim
		if(shoot_control.key_X_cnt == 0)
		{
			set_autoAimFlag(0); //miniPC_info.autoAimFlag = 0;
		}
		else if(shoot_control.key_X_cnt == 1) 
		{
			set_autoAimFlag(1); //miniPC_info.autoAimFlag = 1;
		}
		else if(shoot_control.key_X_cnt == 2)
		{
			//miniPC_info.autoAimFlag = 2;
			set_autoAimFlag(1); //miniPC_info.autoAimFlag = 1;
		}
		
		if( (shoot_control.press_r_time == PRESS_LONG_TIME_R && is_enemy_detected() ) || shoot_control.press_key_V_time == PRESS_LONG_TIME_V) //(shoot_control.press_r_time == PRESS_LONG_TIME_R || shoot_control.press_key_V_time == PRESS_LONG_TIME_V)
		{// 按下鼠标左键 且 识别到目标才进入 绝对瞄准
			set_autoAimFlag(2); //miniPC_info.autoAimFlag = 2;
			//shoot_control.key_X_cnt = 2;
		}
//		else
//		{
//			miniPC_info.autoAimFlag = 1;
//			shoot_control.key_X_cnt = 1;
//		}
		
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_C) // press C to turn off auto aim
		{
			set_autoAimFlag(0); //miniPC_info.autoAimFlag = 0;
			shoot_control.key_X_cnt = 0;
		}
		//X按键计数以及相关检测结束
		
		//10-某一天-2022修改
		//连续发弹判断; 发射机构断电时, shoot_mode状态机不会被置为发射相关状态
		
		//left barrel 连续发弹判断; 发射机构断电时, shoot_mode状态机不会被置为发射相关状态
    if(shoot_control.shoot_mode_L > SHOOT_READY_FRIC && shoot_control.trigger_motor17mm_L_is_online)
    {
        //鼠标长按一直进入射击状态 保持连发
				if(shoot_control.user_fire_ctrl==user_SHOOT_R_CONT)
				{
					//排除项, user_SHOOT_R_CONT是处理右枪管, 和这里无关
					if(shoot_control.shoot_mode_L == SHOOT_BULLET || shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET || shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------注意这里的
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_BOTH)
				{
					//当shoot_control.shoot_mode_L = SHOOT_BULLET 由前序逻辑处理
					if (((get_shootCommand() == 0xff) && (get_autoAimFlag() > 0))|| (shoot_control.press_l_time == PRESS_LONG_TIME_L ) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode_L = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_L_CONT)
				{
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode_L = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_AUTO)
				{
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode_L = SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
				else
				{	//默认的模式
					if (((get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode_L = SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
					}
				}
    }
		
		//以下开始热量环 --------------------------------------------------------------
		shoot_heat_update_calculate(&shoot_control); //就这里执行一次 -------------------------------------------------
		
    //左枪管 ID1 ref热量限制 --------------------------------------------------------------------------------------------------
    get_shooter_id1_17mm_heat_limit_and_heat(&shoot_control.L_barrel_heat_limit, &shoot_control.L_barrel_heat); //.heat_limit .heat
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.L_barrel_heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.L_barrel_heat_limit))
    {
				// || shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------注意这里的
        if(shoot_control.shoot_mode_L == SHOOT_BULLET || shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
        }
				
				if(shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET)
				{
					shoot_control.L_barrel_overheat_stop = 1;
				}
    }//调试: 难道referee uart掉线后 就没有热量保护了?
		else
		{
			shoot_control.L_barrel_overheat_stop = 0;
		}
		
		//使用实时里程计的超热量保护 - 按上面修改
		if(shoot_control.L_barrel_rt_odom_local_heat[0] + shoot_control.local_shoot_heat_remain_value_var_set >= (fp32)shoot_control.L_barrel_local_heat_limit)
    {
				// || shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------注意这里的
        if(shoot_control.shoot_mode_L == SHOOT_BULLET || shoot_control.shoot_mode_L == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode_L =SHOOT_READY_BULLET;
        }
				
				if(shoot_control.shoot_mode_L == SHOOT_ALTERNATE_CONTINUE_BULLET)
				{
					shoot_control.L_barrel_overheat_stop = 1;
				}
    }
		else
		{
			shoot_control.L_barrel_overheat_stop = 0;
		}
		// --------------------------------------------------------------------------------------------------------------------------
		
		//right barrel 连续发弹判断; 发射机构断电时, shoot_mode状态机不会被置为发射相关状态
    if(shoot_control.shoot_mode_R > SHOOT_READY_FRIC && shoot_control.trigger_motor17mm_R_is_online)
    {
        //鼠标长按一直进入射击状态 保持连发
			  if(shoot_control.user_fire_ctrl==user_SHOOT_L_CONT)
				{
					 //排除项, user_SHOOT_L_CONT是处理有枪管, 和这里无关
					if(shoot_control.shoot_mode_R == SHOOT_BULLET || shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET || shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------注意这里的>
					{
							shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_BOTH)
				{
					//当shoot_control.shoot_mode_R = SHOOT_BULLET 由前序逻辑处理
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0))|| (shoot_control.press_l_time == PRESS_LONG_TIME_L ) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode_R = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_R_CONT)
				{
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode_R = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_AUTO)
				{
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode_R = SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
					}
				}
				else
				{ //默认的模式
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0)) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode_R = SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode_R =SHOOT_ALTERNATE_CONTINUE_BULLET;
					}
				}
    }

		//右枪管 ID2 ref热量限制
    get_shooter_id2_17mm_heat_limit_and_heat(&shoot_control.R_barrel_heat_limit, &shoot_control.R_barrel_heat);
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.R_barrel_heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.R_barrel_heat_limit))
    {
				// || shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------注意这里的
        if(shoot_control.shoot_mode_R == SHOOT_BULLET || shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
        }
				
				if(shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET)
				{
					shoot_control.R_barrel_overheat_stop = 1;
				}
    }//调试: 难道referee uart掉线后 就没有热量保护了?
		else
		{
			shoot_control.R_barrel_overheat_stop = 0;
		}
		
		//使用实时里程计的超热量保护
		if(shoot_control.R_barrel_rt_odom_local_heat[0] + shoot_control.local_shoot_heat_remain_value_var_set >= (fp32)shoot_control.R_barrel_local_heat_limit)
    {
				// || shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET) //--------注意这里的
        if(shoot_control.shoot_mode_R == SHOOT_BULLET || shoot_control.shoot_mode_R == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode_R =SHOOT_READY_BULLET;
        }
				
				if(shoot_control.shoot_mode_R == SHOOT_ALTERNATE_CONTINUE_BULLET)
				{
					shoot_control.R_barrel_overheat_stop = 1;
				}
    }
		else
		{
			shoot_control.R_barrel_overheat_stop = 0;
		}
		
//    //如果云台状态是 无力状态，就关闭射击
//    if (gimbal_cmd_to_shoot_stop())
//    {
//        shoot_control.shoot_mode = SHOOT_STOP;
//    }
		
		/*2022 - 2023 RMUL 经济体系 检测并进行发射机构断电+上电后的重启工作 自动重启 功能开始---------------------------------------------------------------
			当且仅当 不在线时 把auto_restart_needed=1 与其他功能没有耦合*/
		if(shoot_control.shoot_mode_L == SHOOT_READY_BULLET && shoot_control.shoot_mode_R == SHOOT_READY_BULLET)
		{
			shoot_control.auto_rst_signal = 0;//重置rst指令
		}
		
		//shoot_control.auto_restart_needed = (shoot_control.trigger_motor17mm_L_is_online)?(0):(1);
		//检测离线 并计算离线时间
//		if(shoot_control.trigger_motor17mm_L_is_online || shoot_control.trigger_motor17mm_R_is_online) //6-18修改 STOP时不启动 自动重启功能
		if( (shoot_control.trigger_motor17mm_L_is_online || shoot_control.trigger_motor17mm_R_is_online) || (shoot_control.shoot_mode_L == SHOOT_STOP && shoot_control.shoot_mode_R == SHOOT_STOP) )
		{
			shoot_control.rst_m_off_time = 0;
		}
		else
		{
			shoot_control.rst_m_off_time++;//两个拨弹电机都不在线
		}
		
		//离线时间到, 主动关闭摩擦轮
		if(shoot_control.rst_m_off_time > 999)
		{
			shoot_control.auto_rst_signal = 1;
			//shoot_control.auto_rst_signal = 0时机为摩擦轮完成预热 - 不受电机online offline的频繁切换影响
      
			shoot_control.shoot_mode_L = SHOOT_STOP;
			shoot_control.shoot_mode_R = SHOOT_STOP;
			shoot_control.key_Q_cnt = 0;
		}
		else
		{
//			shoot_control.auto_rst_signal = 0;
		}
		
		//重新启动计时 - 条件为两电机均在线 且由重启信号
		if(shoot_control.auto_rst_signal == 1 && shoot_control.trigger_motor17mm_L_is_online && shoot_control.trigger_motor17mm_R_is_online)
		{
			shoot_control.rst_on_wait_time++;
		}
		else
		{
			shoot_control.rst_on_wait_time = 0;
		}
		
		//电机上线 时间到 开启摩擦轮
		if(shoot_control.rst_on_wait_time > 460)
		{
			shoot_control.shoot_mode_L = SHOOT_READY_FRIC; 
			shoot_control.shoot_mode_R = SHOOT_READY_FRIC; 
			shoot_control.user_fire_ctrl = user_SHOOT_AUTO;//开启摩擦轮 默认auto
			//设置按键
			shoot_control.key_Q_cnt = 1;
		}
		//自动重启 功能结束 ----------------------------------------------------------------------------------------------------------------

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          射击数据更新
	shoot motor 是拨弹电机
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
		// 默认的是Left Barrel
    static fp32 speed_fliter_1_L = 0.0f;
    static fp32 speed_fliter_2_L = 0.0f;
    static fp32 speed_fliter_3_L = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num_L[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
		
		//Right Barrel
		static fp32 speed_fliter_1_R = 0.0f;
    static fp32 speed_fliter_2_R = 0.0f;
    static fp32 speed_fliter_3_R = 0.0f;
    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num_R[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波 Left barrel 拨弹
#if TRIG_MOTOR_TURN_LEFT_BARREL
    speed_fliter_1_L = speed_fliter_2_L;
    speed_fliter_2_L = speed_fliter_3_L;
    speed_fliter_3_L = speed_fliter_2_L * fliter_num_L[0] + speed_fliter_1_L * fliter_num_L[1] - (shoot_control.shoot_motor_L_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num_L[2]; //shoot_motor_measure
    shoot_control.L_barrel_speed = speed_fliter_3_L; //shoot_control.speed = speed_fliter_3;
#else
		speed_fliter_1_L = speed_fliter_2_L;
    speed_fliter_2_L = speed_fliter_3_L;
    speed_fliter_3_L = speed_fliter_2_L * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_L_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.L_barrel_speed = speed_fliter_3_L;
#endif
		//二阶低通滤波 Right barrel 拨弹
#if TRIG_MOTOR_TURN_RIGHT_BARREL
    speed_fliter_1_R = speed_fliter_2_R;
    speed_fliter_2_R = speed_fliter_3_R;
    speed_fliter_3_R = speed_fliter_2_R * fliter_num_R[0] + speed_fliter_1_R * fliter_num_R[1] - (shoot_control.shoot_motor_R_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num_R[2]; //shoot_motor_measure
    shoot_control.R_barrel_speed = speed_fliter_3_R; //shoot_control.speed = speed_fliter_3;
#else
		speed_fliter_1_R = speed_fliter_2_R;
    speed_fliter_2_R = speed_fliter_3_R;
    speed_fliter_3_R = speed_fliter_2_R * fliter_num_R[0] + speed_fliter_1_R * fliter_num_R[1] + (shoot_control.shoot_motor_R_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num_R[2];
    shoot_control.R_barrel_speed = speed_fliter_3_R;
#endif
		
		//电机是否离线检测
		/*只扫描一次按键这个思路*/
		// Left barrel 左拨弹 电机
		if(toe_is_error(TRIGGER_MOTOR17mm_L_TOE))
		{
			shoot_control.trigger_motor17mm_L_is_online = 0x00;
		}
		else
		{
			shoot_control.trigger_motor17mm_L_is_online = 0x01;
		}
		
		// Right barrel 右拨弹 电机
		if(toe_is_error(TRIGGER_MOTOR17mm_R_TOE))
		{
			shoot_control.trigger_motor17mm_R_is_online = 0x00;
		}
		else
		{
			shoot_control.trigger_motor17mm_R_is_online = 0x01;
		}

//    /*
//		别看这行注释哈, 它写错了: 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
//		应该是:
//		这几句话的目的是判断电机方向的, 对码盘值进行积分时, 需要确定那一帧step的方向, 不能用rpm来, rpm是瞬时的
//		*/
//    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count--;
//    }
//    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count++;
//    }

////    if (shoot_control.ecd_count == FULL_COUNT)
//    {
//        shoot_control.ecd_count = -(FULL_COUNT - 1);//-(FULL_COUNT - 1);
//    }
//    else if (shoot_control.ecd_count == -FULL_COUNT)
//    {
//        shoot_control.ecd_count = FULL_COUNT - 1;
//    }
//    //计算输出轴角度 5-19之前
//		//ecd_count 编码器 计数 数的圈数 整数
//		//之前的转了几圈 + 当前的编码器值 将其转换为弧度制
//    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
		
		//添加了码盘值积分后 对拨弹盘angle的计算 SZL 5-19
		//之前的转了几圈 + 当前的编码器值 将其转换为弧度制 马盘值里程计
#if TRIG_MOTOR_TURN_LEFT_BARREL
//		shoot_control.angle = -(shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
		shoot_control.L_barrel_angle = -(shoot_control.shoot_motor_L_measure->total_ecd + shoot_control.shoot_motor_L_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
#else
		shoot_control.L_barrel_angle = (shoot_control.shoot_motor_L_measure->total_ecd + shoot_control.shoot_motor_L_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
		//shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
#endif

#if TRIG_MOTOR_TURN_RIGHT_BARREL
		shoot_control.R_barrel_angle = -(shoot_control.shoot_motor_R_measure->total_ecd + shoot_control.shoot_motor_R_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
#else
		shoot_control.R_barrel_angle = (shoot_control.shoot_motor_R_measure->total_ecd + shoot_control.shoot_motor_R_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
		//shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
#endif

		//其实可以把所有按键相关状态机放到这里 从set mode中移到这里面 虽然会有耦合
		
		//按键V记时, V只是记录了上一次状态, 但是没有计数
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_V)
		{
			if(shoot_control.press_key_V_time < PRESS_LONG_TIME_V)
			{
				shoot_control.press_key_V_time++;
			}
			shoot_control.last_key_V_sts = 1;
		}
		else
		{
			shoot_control.last_key_V_sts = 0;
			shoot_control.press_key_V_time = 0;
		}
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME_L)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME_R)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_control.shoot_mode_L != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])) //shoot_mode
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }
		
		//NOT USED for MD
//		//12-30-2021 SZL 添加 friction 电机 反馈 数据
//		shoot_control.left_fricMotor.fricW_speed = M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN * shoot_control.left_friction_motor_measure->speed_rpm;
//		shoot_control.right_fricMotor.fricW_speed = M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN * shoot_control.right_friction_motor_measure->speed_rpm;
//		
//		//Added for J-scope debug
//		temp_rpm_right = shoot_control.right_friction_motor_measure->speed_rpm;
//		temp_rpm_left = shoot_control.left_friction_motor_measure->speed_rpm;
		
}

/* 6-6-2023 更改为新的绝对角度控制发弹 - 的退弹函数 */
static void L_barrel_trigger_motor_turn_back_17mm(void)
{
//		//老的 - 模糊控制发弹的退弹 - 不用了
//    if( shoot_control.L_barrel_block_time < BLOCK_TIME_L) //block_time
//    {
//        shoot_control.L_barrel_speed_set = shoot_control.L_barrel_trigger_speed_set; //speed_set trigger_speed_set
//    }
//    else
//    {
//        shoot_control.L_barrel_speed_set = -shoot_control.L_barrel_trigger_speed_set; //speed_set trigger_speed_set
//    }

//    if(fabs(shoot_control.L_barrel_speed) < BLOCK_TRIGGER_SPEED_L && shoot_control.L_barrel_block_time < BLOCK_TIME_L) //speed block_time
//    {
//        shoot_control.L_barrel_block_time++; //block_time
//        shoot_control.L_barrel_reverse_time = 0; //reverse_time
//    }
//    else if (shoot_control.L_barrel_block_time == BLOCK_TIME_L && shoot_control.L_barrel_reverse_time < REVERSE_TIME_L) //block_time reverse_time
//    {
//        shoot_control.L_barrel_reverse_time++; //reverse_time
//    }
//    else
//    {
//        shoot_control.L_barrel_block_time = 0; //block_time
//    }

		if( shoot_control.L_barrel_block_time < BLOCK_TIME_L)
    {//未发生堵转
        //shoot_control.speed_set = shoot_control.trigger_speed_set;
				shoot_control.L_barrel_block_flag = 0;
    }
    else
    {		//发生堵转
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_pid);
				shoot_control.L_barrel_block_flag = 1;//block_flag=1表示发生堵转; block_flag=0表示未发生堵转或已完成堵转清除
        shoot_control.L_barrel_speed_set = -shoot_control.L_barrel_trigger_speed_set;
    }

		//检测堵转时间
    if(fabs(shoot_control.L_barrel_speed ) < BLOCK_TRIGGER_SPEED_L && shoot_control.L_barrel_block_time < BLOCK_TIME_L)
    {
        shoot_control.L_barrel_block_time++;//发生堵转开始计时
        shoot_control.L_barrel_reverse_time = 0;
    }
    else if (shoot_control.L_barrel_block_time == BLOCK_TIME_L && shoot_control.L_barrel_reverse_time < REVERSE_TIME_L)
    {
        shoot_control.L_barrel_reverse_time++;//开始反转 开始计时反转时间
    }
    else
    {//完成反转
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.L_barrel_trigger_motor_pid);
				shoot_control.L_barrel_block_flag = 0;
        shoot_control.L_barrel_block_time = 0;	
    }
		
		if(shoot_control.L_barrel_last_block_flag == 1 && shoot_control.L_barrel_block_flag == 0)
		{//完成一次堵转清除
			//放弃当前的打弹请求
			shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
		}
		
		shoot_control.L_barrel_last_block_flag = shoot_control.L_barrel_block_flag;
		/*block_flag = 1发生堵转
			block_flag = 0未发生堵转*/
}

static void R_barrel_trigger_motor_turn_back_17mm(void)
{
//		//老的 - 模糊控制发弹的退弹 - 不用了
//    if( shoot_control.R_barrel_block_time < BLOCK_TIME_R)
//    {
//        shoot_control.R_barrel_speed_set = shoot_control.R_barrel_trigger_speed_set;
//    }
//    else
//    {
//        shoot_control.R_barrel_speed_set = -shoot_control.R_barrel_trigger_speed_set;
//    }

//    if(fabs(shoot_control.R_barrel_speed) < BLOCK_TRIGGER_SPEED_R && shoot_control.R_barrel_block_time < BLOCK_TIME_R)
//    {
//        shoot_control.R_barrel_block_time++;
//        shoot_control.R_barrel_reverse_time = 0;
//    }
//    else if (shoot_control.R_barrel_block_time == BLOCK_TIME_R && shoot_control.R_barrel_reverse_time < REVERSE_TIME_R)
//    {
//        shoot_control.R_barrel_reverse_time++;
//    }
//    else
//    {
//        shoot_control.R_barrel_block_time = 0;
//    }
		
		if( shoot_control.R_barrel_block_time < BLOCK_TIME_R)
    {//未发生堵转
        //shoot_control.speed_set = shoot_control.trigger_speed_set;
				shoot_control.R_barrel_block_flag = 0;
    }
    else
    {		//发生堵转
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_pid);
				shoot_control.R_barrel_block_flag = 1;//block_flag=1表示发生堵转; block_flag=0表示未发生堵转或已完成堵转清除
        shoot_control.R_barrel_speed_set = -shoot_control.R_barrel_trigger_speed_set;
    }

		//检测堵转时间
    if(fabs(shoot_control.R_barrel_speed) < BLOCK_TRIGGER_SPEED_R && shoot_control.R_barrel_block_time < BLOCK_TIME_R)
    {
        shoot_control.R_barrel_block_time++;//发生堵转开始计时
        shoot_control.R_barrel_reverse_time = 0;
    }
    else if (shoot_control.R_barrel_block_time == BLOCK_TIME_R && shoot_control.R_barrel_reverse_time < REVERSE_TIME_R)
    {
        shoot_control.R_barrel_reverse_time++;//开始反转 开始计时反转时间
    }
    else
    {//完成反转
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.R_barrel_trigger_motor_pid);
				shoot_control.R_barrel_block_flag = 0;
        shoot_control.R_barrel_block_time = 0;	
    }
		
		if(shoot_control.R_barrel_last_block_flag == 1 && shoot_control.R_barrel_block_flag == 0)
		{//完成一次堵转清除
			//放弃当前的打弹请求
			shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
		}
		
		shoot_control.R_barrel_last_block_flag = shoot_control.R_barrel_block_flag;
		/*block_flag = 1发生堵转
			block_flag = 0未发生堵转*/
}

/**
  * @brief          老的 - 模糊控制发弹的退弹 - 不用了 左侧枪管 射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void L_barrel_shoot_bullet_control_17mm(void)//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
{
    //每次拨动 1/4PI的角度
    if (shoot_control.L_barrel_move_flag == 0) //move_flag
    {
        shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format //set_angle angle
        shoot_control.L_barrel_move_flag = 1;
    }
		
		/*这段代码的测试是在NewINF v6.4.1 中测试的, 也就是不会出现:(发射机构断电时, shoot_mode状态机不会被置为发射相关状态)
		整体的逻辑是: 如果发射机构断电, shoot_mode状态机不会被置为发射相关状态, 不会进入此函数; 这段代码只是在这里保险
	  电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor17mm_L_is_online == 0x00)
		{
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle; //set_angle  angle
				return;
		}
		
    if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_L = SHOOT_DONE;
    }
    //到达角度判断
    if ((shoot_control.L_barrel_set_angle - shoot_control.L_barrel_angle) > 0.05f)//(rad_format(shoot_control.set_angle - shoot_control.angle) > 0.0005f)//0.15f) //pr改动前为0.05f shooter_rad_format
    {
        //没到达一直设置旋转速度
        shoot_control.L_barrel_trigger_speed_set = TRIGGER_SPEED_L; //trigger_speed_set
        L_barrel_trigger_motor_turn_back_17mm();
    }
    else
    {
        shoot_control.L_barrel_move_flag = 0; // move_flag
			  shoot_control.shoot_mode_L = SHOOT_DONE; //pr test shoot_mode
    }
   
}

/**
  * @brief          左枪管 射击控制，控制拨弹电机角度，完成一次发射, 精确的角度环PID
  * @param[in]      void
  * @retval         void
  */
static void L_barrel_shoot_bullet_control_absolute_17mm(void)
{
	  //每次拨动 120度 的角度
    if (shoot_control.L_barrel_move_flag == 0)
    {
				/*一次只能执行一次发射任务, 第一次发射任务请求完成后, 还未完成时, 请求第二次->不会执行第二次发射
				一次拨一个单位
        */
				shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.L_barrel_move_flag = 1;
    }
		
		/*这段代码的测试是在NewINF v6.4.1 中测试的, 也就是不会出现:(发射机构断电时, shoot_mode状态机不会被置为发射相关状态)
		整体的逻辑是: 如果发射机构断电, shoot_mode状态机不会被置为发射相关状态, 不会进入此函数; 这段代码只是在这里保险
	  电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor17mm_L_is_online == 0x00)
		{
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle; //set_angle  angle
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_L = SHOOT_DONE;
    }
		//还剩余较小角度时, 算到达了
		if(shoot_control.L_barrel_set_angle - shoot_control.L_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.L_barrel_trigger_speed_set = TRIGGER_SPEED_L;
				//用于需要直接速度控制时的控制速度这里是堵转后反转速度 TRIGGER_SPEED符号指明正常旋转方向
				L_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.L_barrel_move_flag = 0;
				shoot_control.shoot_mode_L = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0当前帧发射机构 没有正在执行的发射请求
			shoot_control.move_flag = 1当前帧发射机构 有正在执行的发射请求*/
}

//左枪 连续发弹控制 每秒多少颗; shoot_freq射频
static void L_barrel_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq)
{
		 //if(xTaskGetTickCount() % (1000 / shoot_freq) == 0) //1000为tick++的频率	
		 if( get_time_based_freq_signal(xTaskGetTickCount(), &(shoot_control.L_barrel_last_tick), shoot_freq) )//get_para_hz_time_freq_signal_FreeRTOS(shoot_freq) )
		 {
			 	shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.L_barrel_move_flag = 1; //固定频率连续发射时, move_flag并没有使用, 依靠时间进行角度增加
		 }
		
		/*这段代码的测试是在NewINF v6.4.1 中测试的, 也就是不会出现:(发射机构断电时, shoot_mode状态机不会被置为发射相关状态)
		整体的逻辑是: 如果发射机构断电, shoot_mode状态机不会被置为发射相关状态, 不会进入此函数; 这段代码只是在这里保险
	  电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor17mm_L_is_online == 0x00)
		{
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_L = SHOOT_DONE;
    }
		//还剩余较小角度时, 算到达了
		if(shoot_control.L_barrel_set_angle - shoot_control.L_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.L_barrel_trigger_speed_set = TRIGGER_SPEED_L;
				//用于需要直接速度控制时的控制速度这里是堵转后反转速度 TRIGGER_SPEED符号指明正常旋转方向
				L_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.L_barrel_move_flag = 0;
				shoot_control.shoot_mode_L = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0当前帧发射机构 没有正在执行的发射请求
			shoot_control.move_flag = 1当前帧发射机构 有正在执行的发射请求*/
}

/**
  * @brief          老的 - 模糊控制发弹的退弹 - 不用了 -右侧枪管 射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void R_barrel_shoot_bullet_control_17mm(void)//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
{
    //每次拨动 1/4PI的角度
    if (shoot_control.R_barrel_move_flag == 0)
    {
        shoot_control.R_barrel_set_angle = (shoot_control.R_barrel_angle + PI_TEN_R);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.R_barrel_move_flag = 1;
    }
		
		/*这段代码的测试是在NewINF v6.4.1 中测试的, 也就是不会出现:(发射机构断电时, shoot_mode状态机不会被置为发射相关状态)
		整体的逻辑是: 如果发射机构断电, shoot_mode状态机不会被置为发射相关状态, 不会进入此函数; 这段代码只是在这里保险
	  电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor17mm_R_is_online == 0x00)
		{
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				return;
		}
		
    if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_R = SHOOT_DONE;
    }
    //到达角度判断
    if ((shoot_control.R_barrel_set_angle - shoot_control.R_barrel_angle) > 0.05f)//(rad_format(shoot_control.set_angle - shoot_control.angle) > 0.0005f)//0.15f) //pr改动前为0.05f shooter_rad_format
    {
        //没到达一直设置旋转速度
        shoot_control.R_barrel_trigger_speed_set = TRIGGER_SPEED_R;
        R_barrel_trigger_motor_turn_back_17mm();
    }
    else
    {
        shoot_control.R_barrel_move_flag = 0;
			  shoot_control.shoot_mode_R = SHOOT_DONE; //pr test
    }
   
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射, 精确的角度环PID
  * @param[in]      void
  * @retval         void
  */
static void R_barrel_shoot_bullet_control_absolute_17mm(void)
{
	  //每次拨动 120度 的角度
    if (shoot_control.R_barrel_move_flag == 0)
    {
				/*一次只能执行一次发射任务, 第一次发射任务请求完成后, 还未完成时, 请求第二次->不会执行第二次发射
				一次拨一个单位
        */
				shoot_control.R_barrel_set_angle = (shoot_control.R_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.R_barrel_move_flag = 1;
    }
		
		/*这段代码的测试是在NewINF v6.4.1 中测试的, 也就是不会出现:(发射机构断电时, shoot_mode状态机不会被置为发射相关状态)
		整体的逻辑是: 如果发射机构断电, shoot_mode状态机不会被置为发射相关状态, 不会进入此函数; 这段代码只是在这里保险
	  电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor17mm_R_is_online == 0x00)
		{
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_R = SHOOT_DONE;
    }
		//还剩余较小角度时, 算到达了
		if(shoot_control.R_barrel_set_angle - shoot_control.R_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.R_barrel_trigger_speed_set = TRIGGER_SPEED_R;
				//用于需要直接速度控制时的控制速度这里是堵转后反转速度 TRIGGER_SPEED符号指明正常旋转方向
				R_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.R_barrel_move_flag = 0;
				shoot_control.shoot_mode_R = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0当前帧发射机构 没有正在执行的发射请求
			shoot_control.move_flag = 1当前帧发射机构 有正在执行的发射请求*/
}

//连续发弹控制 每秒多少颗; shoot_freq射频 6-6-2023
static void R_barrel_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq)
{
		 //if(xTaskGetTickCount() % (1000 / shoot_freq) == 0) //1000为tick++的频率
		if( get_time_based_freq_signal(xTaskGetTickCount(), &(shoot_control.R_barrel_last_tick), shoot_freq) )//get_para_hz_time_freq_signal_FreeRTOS(shoot_freq) )
		{
			shoot_control.R_barrel_set_angle = (shoot_control.R_barrel_angle + PI_TEN_R);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
			shoot_control.R_barrel_move_flag = 1; //固定频率连续发射时, move_flag并没有使用, 依靠时间进行角度增加
		}
		
		/*这段代码的测试是在NewINF v6.4.1 中测试的, 也就是不会出现:(发射机构断电时, shoot_mode状态机不会被置为发射相关状态)
		整体的逻辑是: 如果发射机构断电, shoot_mode状态机不会被置为发射相关状态, 不会进入此函数; 这段代码只是在这里保险
	  电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor17mm_R_is_online == 0x00)
		{
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode_R = SHOOT_DONE;
    }
		//还剩余较小角度时, 算到达了
		if(shoot_control.R_barrel_set_angle - shoot_control.R_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.R_barrel_trigger_speed_set = TRIGGER_SPEED_R;
				//用于需要直接速度控制时的控制速度这里是堵转后反转速度 TRIGGER_SPEED符号指明正常旋转方向
				R_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.R_barrel_move_flag = 0;
				shoot_control.shoot_mode_R = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0当前帧发射机构 没有正在执行的发射请求
			shoot_control.move_flag = 1当前帧发射机构 有正在执行的发射请求*/
}

static void L_R_barrel_alternate_shoot_bullet_control_17mm_timer_reset(uint32_t phase_diff_ms)
{
	//shoot_control.R_barrel_alternate_shoot_last_tick = shoot_control.L_barrel_alternate_shoot_last_tick + phase_diff_ms;// = xTaskGetTickCount();
	xTaskGetTickCount();
}

/*
左右交替发射, 一边打一颗子弹, 绝对位置控制的角度环
uint8_t shoot_freq -> 发射频率
uint16_t phase_diff_ms -> 交替相位差

uint32_t lastTick1 = 0;
uint32_t lastTick2 = 100;  // 使得huart2与huart1有100ms的相位差

while(1) {
  if(HAL_GetTick() - lastTick1 >= 1000) {
    lastTick1 = HAL_GetTick();
    HAL_UART_Transmit(&huart1, (uint8_t*)"One second passed\n", 17, 100);
  }
  if(HAL_GetTick() - lastTick2 >= 1000) {
    lastTick2 = HAL_GetTick();
    HAL_UART_Transmit(&huart2, (uint8_t*)"One second passed\n", 17, 100);
  }
}

*/
static void L_R_barrel_alternate_shoot_bullet_control_continuous_17mm(uint8_t shoot_freq, uint32_t phase_diff_ms)
{
		if( get_time_based_freq_signal(xTaskGetTickCount(), &(shoot_control.L_barrel_alternate_shoot_last_tick), shoot_freq) )//get_para_hz_time_freq_signal_FreeRTOS(shoot_freq) )
		{
//			shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
//			shoot_control.L_barrel_move_flag = 1;
			
//			shoot_control.R_barrel_alternate_shoot_last_tick = shoot_control.L_barrel_alternate_shoot_last_tick + phase_diff_ms;
//		}
			 
//		if( get_time_based_freq_signal(xTaskGetTickCount(), &(shoot_control.R_barrel_alternate_shoot_last_tick), shoot_freq) ) //( xTaskGetTickCount() - shoot_control.R_barrel_alternate_shoot_last_tick >= phase_diff_ms)
			if( generate_signal_pwm(shoot_control.L_barrel_alternate_shoot_last_tick, phase_diff_ms, 0.5f) )
			{ 
				if(shoot_control.L_barrel_overheat_stop == 0) //超热量不发弹
				{
					//左枪管 发射控制
					shoot_control.L_barrel_set_angle = (shoot_control.L_barrel_angle + PI_TEN_L);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
					shoot_control.L_barrel_move_flag = 1;
				}
			}
			else
			{
	//			shoot_control.R_barrel_alternate_shoot_last_tick = xTaskGetTickCount();
				if(shoot_control.R_barrel_overheat_stop == 0) //超热量不发弹
				{
					//右枪管 发射控制
					shoot_control.R_barrel_set_angle = (shoot_control.R_barrel_angle + PI_TEN_R);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
					shoot_control.R_barrel_move_flag = 1;
				}
			}
		
		}
		
		//左枪管 发射控制-------------------------------------------------------------------------------
		/*不会进入此函数; 这段代码只是在这里保险; 电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor17mm_L_is_online == 0x00)
		{
				shoot_control.L_barrel_set_angle = shoot_control.L_barrel_angle;
				shoot_control.L_barrel_move_flag = 0;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
		{
				shoot_control.shoot_mode_L = SHOOT_DONE;
		}
		//还剩余较小角度时, 算到达了
		if(shoot_control.L_barrel_set_angle - shoot_control.L_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.L_barrel_trigger_speed_set = TRIGGER_SPEED_L;
				//用于需要直接速度控制时的控制速度这里是堵转后反转速度 TRIGGER_SPEED符号指明正常旋转方向
				L_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.L_barrel_move_flag = 0;
				shoot_control.shoot_mode_L = SHOOT_DONE; 
		}
		
		//右枪管 发射控制-------------------------------------------------------------------------------
		/*不会进入此函数; 这段代码只是在这里保险; 电机掉线, 即发射机构断电特征出现时, 放弃当前发射请求*/
		if(shoot_control.trigger_motor17mm_R_is_online == 0x00)
		{
				shoot_control.R_barrel_set_angle = shoot_control.R_barrel_angle;
			  shoot_control.R_barrel_move_flag = 0;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
		{
				shoot_control.shoot_mode_R = SHOOT_DONE;
		}
		//还剩余较小角度时, 算到达了
		if(shoot_control.R_barrel_set_angle - shoot_control.R_barrel_angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.R_barrel_trigger_speed_set = TRIGGER_SPEED_R;
				//用于需要直接速度控制时的控制速度这里是堵转后反转速度 TRIGGER_SPEED符号指明正常旋转方向
				R_barrel_trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.R_barrel_move_flag = 0;
				shoot_control.shoot_mode_R = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0当前帧发射机构 没有正在执行的发射请求
			shoot_control.move_flag = 1当前帧发射机构 有正在执行的发射请求*/

}
//static void L_R_barrel_alternate_shoot_bullet_control_absolute_17mm()
//{
//	
//}

/*
4-16-2023 目前未使用 卡尔曼滤波 这个函数是直接赋值 也就是调整ramp.max_value 
*/
static void snail_fric_wheel_kalman_adjustment(ramp_function_source_t *fric1, ramp_function_source_t *fric2)
{
	if(fric1 == NULL || fric2 == NULL)
	{
		return;
	}
	
	// 处理左枪管
	if(fric1 == &shoot_control.L_barrel_fric1_ramp && fric2 == &shoot_control.L_barrel_fric2_ramp)
	{
		fric1->max_value = fric1->max_value_constant;
		fric2->max_value = fric2->max_value_constant;
	}
	else if(fric1 == &shoot_control.R_barrel_fric3_ramp && fric2 == &shoot_control.R_barrel_fric4_ramp)
	{ //处理右枪管
		fric1->max_value = fric1->max_value_constant;
		fric2->max_value = fric2->max_value_constant;
	}
	else
	{
		return; // no val change
	}
	
}

const shoot_control_t* get_robot_shoot_control()
{
	return &shoot_control;
}

/* ---------- getter method 获取数据 ---------- */
shoot_mode_e get_shoot_mode()
{
	return shoot_control.shoot_mode_L;
}

user_fire_ctrl_e get_user_fire_ctrl()
{
	return shoot_control.user_fire_ctrl;
}

uint8_t get_ammoBox_sts()
{
	return shoot_control.ammoBox_sts;
}
/* ---------- getter method end ---------- */

/*
发射机构 拨弹电机 自己的PID, 需要使用积分分离 阈值取决于设备本身
*/
void shoot_PID_init(shoot_pid_t *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 shoot_PID_calc(shoot_pid_t *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
		
		pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

		//积分分离算法
    pid->Pout = pid->Kp * pid->error[0];
		
		if(pid->mode == SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS)
		{
				if(fabs(pid->error[0]) < PID_TRIG_POSITION_INTEGRAL_THRESHOLD)
				{//在范围内, 对此时的值进行积分
					pid->Iout += pid->Ki * pid->error[0];
				}
				else
				{//不在范围内, 此时不计分
					pid->Iout = pid->Iout;
				}

				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				abs_limit(&pid->Iout, pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				abs_limit(&pid->out, pid->max_out);
		}
		else
		{
				if(fabs(pid->error[0]) < PID_TRIG_SPEED_INTEGRAL_THRESHOLD)
				{//在范围内, 对此时的值进行积分
					pid->Iout += pid->Ki * pid->error[0];
				}
				else
				{//不在范围内, 此时不计分
					pid->Iout = pid->Iout;
				}

				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				abs_limit(&pid->Iout, pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				abs_limit(&pid->out, pid->max_out);
		}

    return pid->out;
}

//重置PID
void shoot_PID_clear(shoot_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat)
{
	if(!toe_is_error(REFEREE_TOE))
  {
		 //ID1 的就是左枪管
		 get_shooter_id1_17mm_heat_limit_and_heat(&shoot_heat->L_barrel_heat_limit, &shoot_heat->L_barrel_heat);
		 shoot_heat->L_barrel_local_heat_limit = shoot_heat->L_barrel_heat_limit;
		 shoot_heat->L_barrel_local_cd_rate = get_shooter_id1_17mm_cd_rate();
		
		 //ID2 的就是右枪管
		 get_shooter_id2_17mm_heat_limit_and_heat(&shoot_heat->R_barrel_heat_limit, &shoot_heat->R_barrel_heat);
		 shoot_heat->R_barrel_local_heat_limit = shoot_heat->R_barrel_heat_limit;
		 shoot_heat->R_barrel_local_cd_rate = get_shooter_id2_17mm_cd_rate();
  }
	else
	{
		 //裁判系统离线时 hard code 一个默认的冷却和上限
		 shoot_heat->L_barrel_local_heat_limit = LOCAL_HEAT_LIMIT_SAFE_VAL;
		 shoot_heat->L_barrel_local_cd_rate = LOCAL_CD_RATE_SAFE_VAL;
		 shoot_heat->R_barrel_local_heat_limit = LOCAL_HEAT_LIMIT_SAFE_VAL;
		 shoot_heat->R_barrel_local_cd_rate = LOCAL_CD_RATE_SAFE_VAL;
	}
	
	//用函数10Hz + 里程计信息算
	//热量增加计算
	if( get_para_hz_time_freq_signal_HAL(10) )
	{
		//左枪管 ID1枪管 开始 -------------------------------
		/*当发射机构断电时, 也就是当拨弹电机断电时, 热量不会增加, 只考虑冷却*/
		if(shoot_control.trigger_motor17mm_L_is_online)
		{ //发射机构未断电
#if TRIG_MOTOR_TURN_LEFT_BARREL
			shoot_heat->L_barrel_rt_odom_angle = -(get_L_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE;
//		shoot_heat->L_barrel_rt_odom_angle = -(shoot_heat->L_barrel_angle);
#else
			shoot_heat->L_barrel_rt_odom_angle = (get_L_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE; //TODO 里程计 初始值是负数 - 排除问题
//		shoot_heat->L_barrel_rt_odom_angle = (shoot_heat->L_barrel_angle);
#endif

//		shoot_heat->rt_odom_local_heat = (fp32)(shoot_heat->rt_odom_angle - shoot_heat->last_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC) * ONE17mm_BULLET_HEAT_AMOUNT; //不这样算
	
			//用当前发弹量来计算热量
			shoot_heat->L_barrel_rt_odom_total_bullets_fired = ((fp32)shoot_heat->L_barrel_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC);
			shoot_heat->L_barrel_rt_odom_local_heat[0] += (fp32)abs( ((int32_t)shoot_heat->L_barrel_rt_odom_total_bullets_fired) - ((int32_t)shoot_heat->L_barrel_rt_odom_calculated_bullets_fired) ) * (fp32)ONE17mm_BULLET_HEAT_AMOUNT;
			
			//update last
			shoot_heat->L_barrel_rt_odom_calculated_bullets_fired = shoot_heat->L_barrel_rt_odom_total_bullets_fired;
			shoot_heat->L_barrel_last_rt_odom_angle = shoot_heat->L_barrel_rt_odom_angle;
		}
		else
		{ //发射机构断电 - TODO 是否加一个时间上的缓冲
			shoot_heat->L_barrel_rt_odom_calculated_bullets_fired = shoot_heat->L_barrel_rt_odom_total_bullets_fired;
			shoot_heat->L_barrel_last_rt_odom_angle = shoot_heat->L_barrel_rt_odom_angle;
		}
		
		//冷却
		shoot_heat->L_barrel_rt_odom_local_heat[0] -= (fp32)((fp32)shoot_heat->L_barrel_local_cd_rate / 10.0f);
		if(shoot_heat->L_barrel_rt_odom_local_heat[0] < 0.0f)
		{
			shoot_heat->L_barrel_rt_odom_local_heat[0] = 0.0f;
		}
			 
		//更新时间戳
		shoot_control.L_barrel_local_last_cd_timestamp = xTaskGetTickCount();
		
		//融合裁判系统的heat信息, 修正本地的计算 --TODO 测试中
//		if( abs( ((int32_t)shoot_control.local_last_cd_timestamp) - ((int32_t)get_last_robot_state_rx_timestamp()) ) > 200 )
//		{
//			shoot_heat->rt_odom_local_heat = shoot_heat->heat;
//		}
//		 fp32 delta_heat = shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat); // fabs(shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat));
//		 if(delta_heat > 12.0f) //差不多一发的热量 fabs(delta_heat) 
//		 {
//			 shoot_heat->rt_odom_local_heat[0] -= delta_heat;
//		 }
		
		//local heat限度
		shoot_heat->L_barrel_rt_odom_local_heat[0] = fp32_constrain(shoot_heat->L_barrel_rt_odom_local_heat[0], MIN_LOCAL_HEAT, (fp32)shoot_heat->L_barrel_local_heat_limit*2.0f); //MAX_LOCAL_HEAT); //(fp32)shoot_heat->local_heat_limit
		
		//存过去的
		shoot_heat->L_barrel_rt_odom_local_heat[3] = shoot_heat->L_barrel_rt_odom_local_heat[2];
		shoot_heat->L_barrel_rt_odom_local_heat[2] = shoot_heat->L_barrel_rt_odom_local_heat[1];
		shoot_heat->L_barrel_rt_odom_local_heat[1] = shoot_heat->L_barrel_rt_odom_local_heat[0];
		//----section end----
		//左枪管 ID1枪管 结束 -------------------------------
		
		//右枪管 ID1枪管 开始 -------------------------------
		/*当发射机构断电时, 也就是当拨弹电机断电时, 热量不会增加, 只考虑冷却*/
		if(shoot_control.trigger_motor17mm_R_is_online)
		{ //发射机构未断电
#if TRIG_MOTOR_TURN_RIGHT_BARREL
			shoot_heat->R_barrel_rt_odom_angle = -(get_R_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE;
//		shoot_heat->R_barrel_rt_odom_angle = -(shoot_heat->R_barrel_angle);
#else
			shoot_heat->R_barrel_rt_odom_angle = (get_R_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE; //TODO 里程计 初始值是负数 - 排除问题
//		shoot_heat->R_barrel_rt_odom_angle = (shoot_heat->R_barrel_angle);
#endif

//		shoot_heat->rt_odom_local_heat = (fp32)(shoot_heat->rt_odom_angle - shoot_heat->last_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC) * ONE17mm_BULLET_HEAT_AMOUNT; //不这样算
	
			//用当前发弹量来计算热量
			shoot_heat->R_barrel_rt_odom_total_bullets_fired = ((fp32)shoot_heat->R_barrel_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC);
			shoot_heat->R_barrel_rt_odom_local_heat[0] += (fp32)abs( ((int32_t)shoot_heat->R_barrel_rt_odom_total_bullets_fired) - ((int32_t)shoot_heat->R_barrel_rt_odom_calculated_bullets_fired) ) * (fp32)ONE17mm_BULLET_HEAT_AMOUNT;
			
			//update last
			shoot_heat->R_barrel_rt_odom_calculated_bullets_fired = shoot_heat->R_barrel_rt_odom_total_bullets_fired;
			shoot_heat->R_barrel_last_rt_odom_angle = shoot_heat->R_barrel_rt_odom_angle;
		}
		else
		{ //发射机构断电 - TODO 是否加一个时间上的缓冲
			shoot_heat->R_barrel_rt_odom_calculated_bullets_fired = shoot_heat->R_barrel_rt_odom_total_bullets_fired;
			shoot_heat->R_barrel_last_rt_odom_angle = shoot_heat->R_barrel_rt_odom_angle;
		}
		
		//冷却
		shoot_heat->R_barrel_rt_odom_local_heat[0] -= (fp32)((fp32)shoot_heat->R_barrel_local_cd_rate / 10.0f);
		if(shoot_heat->R_barrel_rt_odom_local_heat[0] < 0.0f)
		{
			shoot_heat->R_barrel_rt_odom_local_heat[0] = 0.0f;
		}
			 
		//更新时间戳
		shoot_control.R_barrel_local_last_cd_timestamp = xTaskGetTickCount();
		
		//融合裁判系统的heat信息, 修正本地的计算 --TODO 测试中
//		if( abs( ((int32_t)shoot_control.local_last_cd_timestamp) - ((int32_t)get_last_robot_state_rx_timestamp()) ) > 200 )
//		{
//			shoot_heat->rt_odom_local_heat = shoot_heat->heat;
//		}
//		 fp32 delta_heat = shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat); // fabs(shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat));
//		 if(delta_heat > 12.0f) //差不多一发的热量 fabs(delta_heat) 
//		 {
//			 shoot_heat->rt_odom_local_heat[0] -= delta_heat;
//		 }
		
		//local heat限度
		shoot_heat->R_barrel_rt_odom_local_heat[0] = fp32_constrain(shoot_heat->R_barrel_rt_odom_local_heat[0], MIN_LOCAL_HEAT, (fp32)shoot_heat->R_barrel_local_heat_limit*2.0f); //MAX_LOCAL_HEAT); //(fp32)shoot_heat->local_heat_limit
		
		//存过去的
		shoot_heat->R_barrel_rt_odom_local_heat[3] = shoot_heat->R_barrel_rt_odom_local_heat[2];
		shoot_heat->R_barrel_rt_odom_local_heat[2] = shoot_heat->R_barrel_rt_odom_local_heat[1];
		shoot_heat->R_barrel_rt_odom_local_heat[1] = shoot_heat->R_barrel_rt_odom_local_heat[0];
		//----section end----
		//右枪管 ID1枪管 结束 -------------------------------
		
	}
	
	return 0;
}

/* 此函数会阻塞
替换原来calibrate task中, CAN_cmd_chassis_reset_ID()函数 依靠摇杆值的输入, 校准PWM
左侧遥感 竖直方向 ch[3]拨到最下面(摇杆最小值 -660.0f)为 pwm max90%, 拨到最上面(摇杆最大值 660.0f)为pwm min 10%
FRIC_OFF 1000 对应 5% pwm
*/
void L_R_barrel_all_fric_esc_pwm_calibration()
{
//	//挂起其它所有可能产生影响的任务
	osThreadSuspend(gimbalTaskHandle); //文件 最前面 声明 用于挂起和恢复
	osThreadSuspend(chassisTaskHandle);
	
	//初始化为最大值
	L_barrel_fric1_on(17999);
	L_barrel_fric2_on(17999);
	R_barrel_fric3_on(17999);
	R_barrel_fric4_on(17999);
	vTaskDelay(200);
	
	while(1)
	{
		if(shoot_control.shoot_rc->rc.ch[3] < (int16_t)(-330))
		{
			//开启蜂鸣器
			buzzer_on(1, 500);  //buzzer_on(50, 19999); //头文件声明最前面
			//摇杆向下拨
			L_barrel_fric1_on(2199);
			L_barrel_fric2_on(2199);
			R_barrel_fric3_on(2199);
			R_barrel_fric4_on(2199); //17999
		}
		else if(shoot_control.shoot_rc->rc.ch[3] > (int16_t)(330))
		{
			//开启蜂鸣器
			buzzer_on(10, 500); //buzzer_on(70, 19999); //头文件声明最前面
			
			//摇杆向上拨
			L_barrel_fric1_on(1000); //1999
			L_barrel_fric2_on(1000);
			R_barrel_fric3_on(1000);
			R_barrel_fric4_on(1000);
		}
		
		//检测左侧开关不为最下的情况退出循环
		if(!( switch_is_down(shoot_control.shoot_rc->rc.s[1]) ))
		{
			break;
		}
	}
	
//	//resume 之前挂起的任务
	osThreadResume(gimbalTaskHandle);
	osThreadResume(chassisTaskHandle);
}
