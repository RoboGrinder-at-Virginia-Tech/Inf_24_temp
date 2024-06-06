/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled屏幕显示错误码
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "oled_task.h"
#include "main.h"
#include "oled.h"

#include "cmsis_os.h"
#include "detect_task.h"
#include "voltage_task.h"

#include "SuperCap_comm.h"
#include "prog_msg_utility.h"
#include "stdio.h"

#define OLED_CONTROL_TIME 10 //100 //10
#define REFRESH_RATE    10

const error_t *error_list_local;

uint8_t other_toe_name0[5][5] = {"yaw\0","pL\0","pR\0","tL\0","tR\0"}; //{"yaw\0","ptL\0","ptR\0","trL\0","trR\0"};
uint8_t other_toe_name[4][4] = {"GYR\0","ACC\0","MAG\0","REF\0"};

uint8_t last_oled_error = 0;
uint8_t now_oled_errror = 0;
static uint8_t refresh_tick = 0;
uint32_t oled_dynamic_init_TimeStamp=0;
uint32_t oled_refresh_TimeStamp=0;

/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void const * argument)
{
    uint8_t i;
    uint8_t show_col, show_row;
    error_list_local = get_error_list_point();
		OLED_com_reset();//
    osDelay(1000);
//	  OLED_com_reset();//
    OLED_init();
    OLED_LOGO();
    i = 100;
    while(i--)
    {
        if(OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }
        osDelay(10);
    }
    while(1)
    {
        //use i2c ack to check the oled
        if(OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }

        now_oled_errror = toe_is_error(OLED_TOE);
        //oled init
        if(last_oled_error == 1 && now_oled_errror == 0)
        {
            OLED_init();
        }
				
				if(toe_is_error(OLED_TOE))
        {
            OLED_init();
        }
				
				
        if(now_oled_errror == 0)//now_oled_errror == 0)
        {
            refresh_tick++;
            //10Hz refresh
            if(refresh_tick > configTICK_RATE_HZ / (OLED_CONTROL_TIME * REFRESH_RATE))
            {
//					  if(xTaskGetTickCount() - oled_refresh_TimeStamp >= 100)//100
//						{
							  oled_refresh_TimeStamp = xTaskGetTickCount();
                refresh_tick = 0;
                OLED_operate_gram(PEN_CLEAR);
                OLED_show_graphic(0, 1, &battery_box);

                if(get_battery_percentage() < 10)
                {
                    OLED_printf(9, 2, "%d", get_battery_percentage());
                }
                else if(get_battery_percentage() < 100)
                {
                    OLED_printf(6, 2, "%d", get_battery_percentage());
                }
                else
                {
                    OLED_printf(3, 2, "%d", get_battery_percentage());
                }

//                OLED_show_string(90, 27, (uint8_t*)"DBUS");
//                OLED_show_graphic(115, 27, &check_box[error_list_local[DBUS_TOE].error_exist]);
								//显示在靠近右下角
								OLED_show_string(90, 50, (uint8_t*)"DBUS");
                OLED_show_graphic(115, 50, &check_box[error_list_local[DBUS_TOE].error_exist]);
								
								//添加两种超级电容在线状态框和电压 - 在DBUS旁边 - TODO: cap电压数据限制
								char cap_vol_str[20];
								sprintf(cap_vol_str, "%.1f", ui_get_current_cap_voltage());
								OLED_show_string(30, 50, (uint8_t*)"SP");
								OLED_show_string(30+15, 50, (uint8_t*)cap_vol_str); //30+20, 50
                OLED_show_graphic(50+25, 50, &check_box[all_superCap_is_error() ? 1 : 0]); //error_list_local[GEN3CAP_TOE].error_exist
								
								//在电池旁边 显示Fric L和Fric R的连接状态
								/*电池logo 估计长度: 32; 宽度: 15*/
								//Fric L
								OLED_show_string(32, 0, (uint8_t*)"FL");
								OLED_show_graphic(32+12, 0, &check_box[error_list_local[SHOOT_FRIC_L_TOE].error_exist]);
								
								//Fric R
								OLED_show_string(64, 0, (uint8_t*)"FR");
								OLED_show_graphic(64+12, 0, &check_box[error_list_local[SHOOT_FRIC_R_TOE].error_exist]);
								
								//szl 3-28 add 1s tick time display
								OLED_show_string(96, 0, (uint8_t*)"T");
								OLED_show_string(96+6, 0, (uint8_t*)util_1s_time_cnt_toString());
								
                for(i = CHASSIS_MOTOR1_TOE; i < CHASSIS_MOTOR4_TOE + 1; i++) //for(i = CHASSIS_MOTOR1_TOE; i < TRIGGER_MOTOR_TOE + 1; i++)
                {		//第一排的4个设备
                    show_col = ((i-1) * 32) % 128;
                    show_row = 15 + (i-1) / 4 * 12;
                    OLED_show_char(show_col, show_row, 'M');
                    OLED_show_char(show_col + 6, show_row, '0'+i);
                    OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                }
								
								for(i = YAW_GIMBAL_MOTOR_TOE; i < TRIGGER_MOTOR17mm_R_TOE + 1; i++)
								{
										show_col = ((i-1) * 32) % 128; //(i * 32) % 128;
                    show_row = 15 + (i-1) / 4 * 12; //15 + i / 4 * 12;
                    OLED_show_string(show_col, show_row, other_toe_name0[i - YAW_GIMBAL_MOTOR_TOE]);
                    OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);
								}

                for(i = BOARD_GYRO_TOE; i < REFEREE_TOE + 1; i++)
                {
                    show_col = ((i-1) * 32) % 128; //(i * 32) % 128;
                    show_row = 15 + (i-1) / 4 * 12; //15 + i / 4 * 12;
                    OLED_show_string(show_col, show_row, other_toe_name[i - BOARD_GYRO_TOE]);
                    OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);

                }
								

                OLED_refresh_gram();
            }//
        }
//以1Hz 频率初始化
//				if(xTaskGetTickCount() - 2000 > oled_dynamic_init_TimeStamp)
//				{
//						oled_dynamic_init_TimeStamp = xTaskGetTickCount(); //更新时间戳 
////						OLED_operate_gram(PEN_CLEAR);
////						OLED_com_reset();
//						OLED_init();
//				}
//				

        last_oled_error = now_oled_errror;
        vTaskDelay(OLED_CONTROL_TIME); //osDelay(OLED_CONTROL_TIME);
    }
}

