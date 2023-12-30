/**
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  * @file       usb_ano_task.c/h
  * @brief      usb_ano_task. tasks that handles AnoPTv8 app
  * @note       Using USB, output data to AnoPTv8 app
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     April-1-2023     Zelin Shen     basic utility functions
	*
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  */
#include "usb_ano_task.h"
#include "usb_task.h"
#include "usbd_cdc_if.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "INS_task.h"
#include "AnoPTv8.h"

extern int32_t parListForTest[ANOPTV8_PARNUM];

usb_ano_gimbal_info_t usb_ano_gimbal_info;

/*
Based on the communication doc, mode 1 use Euler angle
*/
void AnoPTv8TxFrameF3_mode1(void)
{
	uint8_t databuf[8];
	
	databuf[0] = 0x01; //mpde 1 use Euler angle
	
	//[0]ROLL
	databuf[1] = BYTE0(usb_ano_gimbal_info.angle[0]);
	databuf[2] = BYTE1(usb_ano_gimbal_info.angle[0]);
	
	//[1]PIT
	databuf[3] = BYTE0(usb_ano_gimbal_info.angle[1]);
	databuf[4] = BYTE1(usb_ano_gimbal_info.angle[1]);
	
	//[2]YAW
	databuf[5] = BYTE0(usb_ano_gimbal_info.angle[2]);
	databuf[6] = BYTE1(usb_ano_gimbal_info.angle[2]);
	
	databuf[7] = 0x00; //not used
	
	//send
	AnoPTv8SendBuf(ANOPTV8_SWJID, 0x03, databuf, 8);
}

void usb_ano_task(void const * argument)
{
	
  MX_USB_DEVICE_Init();
	memset(&usb_ano_gimbal_info, 0, sizeof(usb_ano_gimbal_info_t));
	
	usb_ano_gimbal_info.gimbal_INS_angle_ptr = get_INS_angle_point(); //get_INS_gimbal_angle_point();
	usb_ano_gimbal_info.gimbal_INS_gyro_ptr = get_INS_gyro_point(); //get_INS_gimbal_gyro_point();
	
  while(1)
  {
		//update from our on board sensor
		for(uint8_t i = 0; i < 3; i++)
		{
			usb_ano_gimbal_info.angle[i] = usb_ano_gimbal_info.gimbal_INS_angle_ptr[i];
			usb_ano_gimbal_info.gyro_w[i] = usb_ano_gimbal_info.gimbal_INS_gyro_ptr[i];
		}
		
		//copy data to send; convert to int16
		//[0]ROLL [1]PIT [2]YAW by ano protocol 180/pi = 57.29577951f; - sign here is to temp fix the direction; 
		// TODO: maybe use another rotation matrix rotate to their device's orientation coordinate
		usb_ano_gimbal_info.angle[0] = (int16_t)( ( *(usb_ano_gimbal_info.gimbal_INS_angle_ptr + INS_gimbal_angle_ROLL_ADDRESS_OFFSET) * 57.29577951f ) * 100.0f );
		usb_ano_gimbal_info.angle[1] = (int16_t)( ( *(usb_ano_gimbal_info.gimbal_INS_angle_ptr + INS_gimbal_angle_PITCH_ADDRESS_OFFSET) * -57.29577951f ) * 100.0f );
		usb_ano_gimbal_info.angle[2] = (int16_t)( ( *(usb_ano_gimbal_info.gimbal_INS_angle_ptr + INS_gimbal_angle_YAW_ADDRESS_OFFSET) * -57.29577951f ) * 100.0f );
		
		//pack the data
		AnoPTv8TxFrameF3_mode1();
		
		//send
		AnoPTv8HwTrigger1ms(); //AnoPTv8TxRunThread1ms();
		
		vTaskDelay(1);
	}
}
