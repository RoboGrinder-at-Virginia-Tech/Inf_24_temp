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
#ifndef CHASSIS_ENERGY_REGULATE_H
#define CHASSIS_ENERGY_REGULATE_H

typedef enum
{
	CHASSIS_CHARGE,
	CHASSIS_NORMAL,
  CHASSIS_BOOST,
} chassis_energy_mode_e;

struct chassis_move_t;
extern void chassis_energy_regulate(struct chassis_move_t *chassis_energy);

#endif
