#ifndef __USB_ANO_TASK_H__
#define __USB_ANO_TASK_H__
#include "struct_typedef.h"


typedef __packed struct
{
	//offset is in INS_task.h
	const fp32 * gimbal_INS_angle_ptr; // point to our INS gimbal Euler angle
	const fp32 * gimbal_INS_gyro_ptr; // point to our INS task
	
	//the following message will be sent to ano
	//[0]ROLL [1]PIT [2]YAW
	int16_t angle[3]; 
	int16_t gyro_w[3];
	
} usb_ano_gimbal_info_t;	

extern void usb_ano_task(void const * argument);

#endif
