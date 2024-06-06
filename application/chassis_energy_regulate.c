/**
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  * @file       chassis_energy_regulate.c/h
  * @brief      chassis energy regulate 底盘功率 能量调控
  * @note       Based on strategy, adjust chassis energy usage by speed: chassis energy regulate.
  *             This program mainly adjust the chassis spinning speed based on drivers input.
	* 						Main feedbacks are super cap remaining percentage, 
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     Zelin Shen      1. add chassis energy regulate
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  */
#include "chassis_energy_regulate.h"
#include "referee.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "detect_task.h"
#include "SuperCap_comm.h"
#include "chassis_task.h"
#include "referee_interact_task.h"

#define RAD_PER_SEC_FROM_RPM(rpm) ( (float)(((float)rpm) * 2.0f * PI / 60.0f) ) // 将RPM转换为弧度每秒


void chassis_energy_regulate(chassis_move_t *chassis_energy)
{
	if (chassis_energy == NULL) {
		return;
	}
	
	// 按键输入 初步设置底盘功率模式
	uint32_t current_time = xTaskGetTickCount();
	
	switch (chassis_energy->chassis_RC[TEMP].key[KEY_PRESS].ctrl)
	{
		case 0:
			if(chassis_energy->last_chassis_energy_mode == CHASSIS_CHARGE)
			{
				chassis_energy->chassis_energy_mode = CHASSIS_NORMAL;
			}
		break;

		// 按下
		case 1:
			chassis_energy->chassis_energy_mode = CHASSIS_CHARGE;
		break;

		default:
			// does nothings
		break;
	}
	
	switch (chassis_energy->chassis_RC[TEMP].key[KEY_PRESS].shift)
	{
		case 0:
			// 刚松开shift时, 有一个延时, 平滑过渡 --> 再小陀螺减速
			if(chassis_energy->last_chassis_energy_mode == CHASSIS_BOOST && current_time - chassis_energy->shift_pressed_timestamp >= 1000)
			{
				chassis_energy->chassis_energy_mode = CHASSIS_NORMAL;
			}
		break;
		
		case 1:
			chassis_energy->chassis_energy_mode = CHASSIS_BOOST;
			chassis_energy->shift_pressed_timestamp = current_time;
		break;

		default:
			// does nothings
		break;
	}
	
	// 超级电容能量低时, 可以override用户操作离开BOOST模式
	if(chassis_energy->chassis_energy_mode == CHASSIS_BOOST && (cer_get_current_cap_relative_pct() < cer_get_current_cap_boost_mode_pct_threshold()) )
	{
		chassis_energy->chassis_energy_mode = CHASSIS_NORMAL;
	}
	
	// 刷新UI
	if(chassis_energy->last_chassis_energy_mode != chassis_energy->chassis_energy_mode)
	{
		set_interactive_flag_chassis_energy_mode_flag(1);
	}
	chassis_energy->last_chassis_energy_mode = chassis_energy->chassis_energy_mode;
	
	// 按照最终确定的模式来判断小陀螺转速
	switch (chassis_energy->chassis_energy_mode)
	{
		case CHASSIS_BOOST:
			// 当有按键输入时, 降低小陀螺转速
			if (fabs(chassis_energy->vx_set) > 0.01 || fabs(chassis_energy->vy_set) > 0.01)
			{
				chassis_energy->spin_speed = RAD_PER_SEC_FROM_RPM(70);
				chassis_energy->moving_timestamp = current_time;
			} else {
				// 底盘刚停下in place时, 有一个延时, 平滑过渡 --> 再小陀螺加速
				if(current_time - chassis_energy->moving_timestamp >= 500)
				{
					chassis_energy->spin_speed = RAD_PER_SEC_FROM_RPM(120); // 100
				} else {
					chassis_energy->spin_speed = RAD_PER_SEC_FROM_RPM(70);
				}
			}
		break;
		
		case CHASSIS_NORMAL:
			chassis_energy->spin_speed = RAD_PER_SEC_FROM_RPM(70);
		break;
		
		case CHASSIS_CHARGE:
			chassis_energy->spin_speed = RAD_PER_SEC_FROM_RPM(50);
		break;

		default:
			// as normal
			chassis_energy->spin_speed = RAD_PER_SEC_FROM_RPM(70);
		break;
	}
}
