/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "shoot.h"
#include "referee_ui.h"
#include "referee_interact_task.h"


extern shoot_control_t shoot_control;

/*
SZL MG-995舵机
初始值: min500 max2500
举例:
最小值: 500
程序上把Min 设置成500 按住递减 减到Min (=500)状态机为弹舱关闭 <=> 因为得100%确定关闭

最大值 2000
程序上把Max 设置为 2000 按住按键递增 增加到接近max (>1500)判断为弹舱盖 开启

标定:
Hero的弹舱关闭 PWM = 655 = Min; MAX2000

步兵 PWM = 1500 关
		 PWM = 600 开
		 
2000 - 接近180度
*/

#define SERVO_MIN_PWM   600 //655 //开启
#define SERVO_MAX_PWM   1500 //1800 //1575//关

#define AMMO_BOX_COVER_CLOSE_STATE SERVO_MAX_PWM
#define AMMO_BOX_COVER_OPEN_STATE (SERVO_MIN_PWM)

#define PWM_DETAL_VALUE 10 //10

//#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
//#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
//#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
//#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

//#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

#define INF_AMMO_BOX_ADD_PWM_KEY KEY_PRESSED_OFFSET_Z
#define SERVO_ADD_PWM_KEY KEY_PRESSED_OFFSET_CTRL

const RC_ctrl_t *servo_rc;
//const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[4] = {SERVO_MAX_PWM, SERVO_MAX_PWM, SERVO_MAX_PWM, SERVO_MAX_PWM};
ui_ammoBox_sts_e last_ui_ammoBox_sts = ammoOFF;

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
		uint32_t dial_ccw_set_val_time = 0;
		uint32_t dial_cw_set_val_time = 0;

    while(1)
    {
        for(uint8_t i = 0; i < 4; i++)
        {
//            if(servo_rc->key.v & KEY_PRESSED_OFFSET_Z)
//            {
//                servo_pwm[i] -= PWM_DETAL_VALUE;
//            }
//            else if( (servo_rc->key.v & SERVO_ADD_PWM_KEY) && (servo_rc->key.v & KEY_PRESSED_OFFSET_Z))
//            {
//                servo_pwm[i] += PWM_DETAL_VALUE;
//            }
					
					// 遥控器滚轮滤波时间
					if(servo_rc[TEMP].rc.dial > 500)
					{
						dial_ccw_set_val_time++;
					}
					else
					{
						dial_ccw_set_val_time = 0;
					}
					
					if(servo_rc[TEMP].rc.dial < -500)
					{
						dial_cw_set_val_time++;
					}
					else
					{
						dial_cw_set_val_time = 0;
					}
						
						if(servo_rc[TEMP].key[KEY_PRESS_WITH_CTRL].b || dial_ccw_set_val_time > 50)
            {
                servo_pwm[i] += PWM_DETAL_VALUE;
            }
            else if(servo_rc[TEMP].key[KEY_PRESS].b || dial_cw_set_val_time > 50)
            {
                servo_pwm[i] -= PWM_DETAL_VALUE;
            }

            //limit the pwm
           //限制pwm
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }
						
						//判断 Ammo Box Cover FSM
						if((servo_pwm[i] < AMMO_BOX_COVER_OPEN_STATE) || (servo_pwm[i] == AMMO_BOX_COVER_OPEN_STATE))
						{//弹舱开
							// 后台更新
							set_ui_ammoBox_sts(ammoOPEN);
							
							// 刷新一次
							if (last_ui_ammoBox_sts != ammoOPEN)
							{
								set_interactive_flag_ammo_box_cover_sts_flag(1);
							}
							last_ui_ammoBox_sts = ammoOPEN; // update last
						}
						else if((servo_pwm[i] > AMMO_BOX_COVER_CLOSE_STATE) || (servo_pwm[i] == AMMO_BOX_COVER_CLOSE_STATE))
						{//弹舱关
							// 后台更新
							set_ui_ammoBox_sts(ammoOFF);
							
							// 刷新一次
							if (last_ui_ammoBox_sts != ammoOFF)
							{
								set_interactive_flag_ammo_box_cover_sts_flag(1);
							}
							last_ui_ammoBox_sts = ammoOFF; // update last
						}

            servo_pwm_set(servo_pwm[i], i);
        }
        osDelay(10);
    }
}


