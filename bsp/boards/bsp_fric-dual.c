#include "bsp_fric.h"
#include "main.h"
#include "shoot.h"
#include "arm_math.h"

extern TIM_HandleTypeDef htim1;
extern shoot_control_t shoot_control;
/*
SZL 4-15-2023 添加 MD 炮塔的另外两个 摩擦轮
fric1_on(..) 变为 L_barrel_fric1_on(..)
fric2_on(..) 变为 L_barrel_fric2_on(..)
__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd)相关函数; cmd设置CCR比较值寄存器, CNT计数值单片机数数用的
cmd数值最小值和最大值, 最小值为0 - 0%占空比, 最大值为19999(或20000) 100%占空比, AutoReload Regiter(ARR重载值) 配置=19999

htim1所有channel 1 ~ 4 模式pwm mode 1 递增 CNT<CCR, 通道CH为有效, 否则为无效(即CNT>=CCR)

19999 * 10% = 1999; 19999 * 90% = 17999
*/

void L_barrel_fric_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, FRIC_OFF);
}
void L_barrel_fric1_on(uint16_t cmd) // fric1_on
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
}
void L_barrel_fric2_on(uint16_t cmd) // fric2_on
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
}

void R_barrel_fric_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, FRIC_OFF);
}
void R_barrel_fric3_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, cmd);
}
void R_barrel_fric4_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, cmd);
}



///*
//SZL 12-30-2021 M3508 摩擦轮 射击速度 控制
//4-15-2023 NOT needed for MD
//*/
//void M3508_fric_wheel_spin_control(fp32 left_fric_speed, fp32 right_fric_speed)
//{
//		//fp32 temp = 0.0f;
//		//fp32 vector_rate = 0.0f;
//		shoot_control.left_fricMotor.fricW_speed_set = left_fric_speed;
//		shoot_control.right_fricMotor.fricW_speed_set = right_fric_speed;
//	
//		//添加M3508_fric_wheel_spin_control 限制幅度 按照Wheel speed限制幅度
//		/*
//		temp = fabs(shoot_control.left_fricMotor.fricW_speed_set);
//		if(temp > shoot_control.currentLIM_shoot_speed)
//		{
//			shoot_control.left_fricMotor.fricW_speed_set = (-shoot_control.currentLIM_shoot_speed);
//			shoot_control.right_fricMotor.fricW_speed_set = shoot_control.currentLIM_shoot_speed;
//		}
//		
//		temp = shoot_control.right_fricMotor.fricW_speed_set;
//		if(temp > shoot_control.currentLIM_shoot_speed)
//		{
//			shoot_control.left_fricMotor.fricW_speed_set = (-shoot_control.currentLIM_shoot_speed);
//			shoot_control.right_fricMotor.fricW_speed_set = shoot_control.currentLIM_shoot_speed;
//		}
//		*/
//	
//		PID_calc(&shoot_control.left_fric_motor_pid, shoot_control.left_fricMotor.fricW_speed, left_fric_speed); //left_fric_speed 改
//		PID_calc(&shoot_control.right_fric_motor_pid, shoot_control.right_fricMotor.fricW_speed, right_fric_speed);
//	
//		shoot_control.left_fricMotor.fricW_given_current = shoot_control.left_fric_motor_pid.out;
//		shoot_control.right_fricMotor.fricW_given_current = shoot_control.right_fric_motor_pid.out;
//	
//		CAN_cmd_friction_wheel(shoot_control.left_fricMotor.fricW_given_current, shoot_control.right_fricMotor.fricW_given_current);

//}

