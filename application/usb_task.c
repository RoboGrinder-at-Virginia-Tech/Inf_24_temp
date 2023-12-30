/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
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
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"

#include "miniPC_comm_task.h"
#include "miniPC_msg.h"

#include "prog_msg_utility.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_task_high_water;
#endif

extern pc_cmd_gimbal_ctrl_t pc_cmd_gimbal_ctrl_aid;
extern pc_cmd_gimbal_ctrl_t pc_cmd_gimbal_ctrl_full;

extern pc_comm_unpack_data_t pc_comm_unpack_data_obj;


//static void usb_printf(const char *fmt,...);

//static uint8_t usb_buf[336]; //256 这就是越界的指针 change to 512 加上换行符336
uint8_t usb_buf[512]; //256 这就是越界的指针 change to 512 加上换行符336
//static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;



void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();


    while(1)
    {
        osDelay(1000);
//        usb_printf(
//"******************************\r\n\
//voltage percentage:%d%% \r\n\
//DBUS:%s\r\n\
//chassis motor1:%s\r\n\
//chassis motor2:%s\r\n\
//chassis motor3:%s\r\n\
//chassis motor4:%s\r\n\
//yaw motor:%s\r\n\
//pitch motor:%s\r\n\
//trigger motor:%s\r\n\
//gyro sensor:%s\r\n\
//accel sensor:%s\r\n\
//mag sensor:%s\r\n\
//referee usart:%s\r\n\
//******************************\r\n",
//            get_battery_percentage(), 
//            status[error_list_usb_local[DBUS_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
//            status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
//            status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
//            status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
//            status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
//            status[error_list_usb_local[REFEREE_TOE].error_exist]);

///* -------------- CV communication tests -------------- */
//				usb_printf(
//"******************************\r\n\
//miniPC comm:%s\r\n\
//pc_cmd_gimbal_ctrl_aid.yaw: %x\r\n\
//pc_cmd_gimbal_ctrl_aid.pitch: %x\r\n\
//pc_cmd_gimbal_ctrl_aid.is_detect: %x\r\n\
//pc_cmd_gimbal_ctrl_aid.shoot: %x\r\n\
//----\r\n\
//pc_comm_unpack_data_obj.frame_len: %x\r\n\
//pc_comm_unpack_data_obj.cmd_id: %x\r\n\
//pc_comm_unpack_data_obj.unpack_step: %x\r\n\
//******************************\r\n",
//						status[error_list_usb_local[PC_TOE].error_exist],
//						pc_cmd_gimbal_ctrl_aid.yaw,
//						pc_cmd_gimbal_ctrl_aid.pitch,
//						pc_cmd_gimbal_ctrl_aid.is_detect,
//						pc_cmd_gimbal_ctrl_aid.shoot,
//						
//						pc_comm_unpack_data_obj.frame_len,
//						pc_comm_unpack_data_obj.cmd_id,
//						pc_comm_unpack_data_obj.unpack_step);
///* -------------- CV communication tests Ends -------------- */

		/* ---------- RTOS tasks stats and info - prog_msg_utility ---------- */
			  CPU_info_to_usb();
		/* ---------- RTOS tasks stats and info - prog_msg_utility Ends ---------- */
#if INCLUDE_uxTaskGetStackHighWaterMark
    usb_task_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif	
    }

}

void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(usb_buf, len);
}

//static void usb_printf(const char *fmt,...)
//{
//    static va_list ap;
//    uint16_t len = 0;

//    va_start(ap, fmt);

//    len = vsprintf((char *)usb_buf, fmt, ap);

//    va_end(ap);


//    CDC_Transmit_FS(usb_buf, len);
//}

//int fputc(int ch,FILE *f)
//{
//    //uint8_t temp[1]={ch};
//    //HAL_UART_Transmit(&huart6,temp,1,2);//初始化USART时 设置8bits
//	
//		uint8_t localBuf[1] = {ch};
//		CDC_Transmit_FS(localBuf, 1);
//		
//		return NULL;
//}
