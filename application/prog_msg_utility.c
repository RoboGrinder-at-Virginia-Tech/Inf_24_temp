/**
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  * @file       prog_msg_utility.c/h
  * @brief      prog_msg_utility. program utility relared functions
  * @note       This file provide functions related to debug(CPU usage), 
	*							message information about miniPC 
	*							<-> TypeC board communication(Tx and Rx) package freq and package loss rate
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-28-2023     Zelin Shen      basic utility functions
	*
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  */
	
#include "prog_msg_utility.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "miniPC_comm_task.h"
//#include "string.h"
#include "stdio.h"
//#include "CRC8_CRC16.h"
//#include "detect_task.h"
//#include "SuperCap_comm.h"
//#include "referee.h"
//#include "arm_math.h"

//use
#include "FreeRTOS.h"
#include "tim.h"
#include "struct_typedef.h"
#include "usb_task.h"

/* RTOS based Run time stats gathering functions.
From: https://blog.csdn.net/liuyi_lab/article/details/112833260
From: https://doc.embedfire.com/linux/stm32mp1/freertos/zh/latest/application/cpu_usage_rate.html
Follow the 2nd instruciton:
--------------------------------------------------------------------
In file FreeRTOS.h the following changes will be made by CubeMX
 #define configGENERATE_RUN_TIME_STATS            1
 #define configUSE_TRACE_FACILITY                 1
 #define configUSE_STATS_FORMATTING_FUNCTIONS     1
which enabled RTOS to help us do run time stats

Then implement the following functions:

--需要在main中调用此初始化函数init_prog_msg_utility()
*/

//volatile unsigned long util_CPU_RunTime = 0UL;
//volatile uint64_t util_CPU_RunTime = 0UL;
//note that we have to use unsigned long because this is the return type in RTOS related function API
volatile unsigned long util_CPU_RunTime = 0UL;
volatile uint32_t util_1s_time_cnt = 0UL;

//extern TIM_HandleTypeDef htim14; this is the timer for CPU runtime stat

uint8_t CPU_RunInfo[512];           //保存任务运行时间信息

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14 )
  { //20000HZ 中断频率
      util_CPU_RunTime++;
  }
	
	if(util_CPU_RunTime % 20000 == 0)
	{
		util_1s_time_cnt++;
	}
	
//	if(htim == &htim14 )
//  {
//      util_CPU_RunTime++;
//  }
}

void configureTimerForRunTimeStats(void)
{
	util_CPU_RunTime = 0UL;
}

unsigned long getRunTimeCounterValue(void)
{
	return util_CPU_RunTime;
}

uint32_t get_util_1s_time_cnt(void)
{
	return util_1s_time_cnt;
}

const char* util_1s_time_cnt_toString(void)
{
	static char temp_toString[20];
	sprintf(temp_toString, "%d", (util_1s_time_cnt%1000));
	return temp_toString;
}

void init_prog_msg_utility()
{
	//启动定时器
  HAL_TIM_Base_Start_IT(&htim14);
	
	memset(CPU_RunInfo,0,sizeof(CPU_RunInfo)); //信息缓冲区清零
}

/*
This function output the CPU information to usb, call this funciton only in the usb task
*/
void CPU_info_to_usb(void)
{
//	memset(CPU_RunInfo,0,400);   //信息缓冲区清零
//	vTaskList((char *)&CPU_RunInfo);  //获取任务运行时间信息
//	usb_printf(
//"---------------------------------------------\r\n\
// 任务名    任务状态 优先级     剩余栈     任务序号\r\n\
// %s\
// ---------------------------------------------\r\n",
//  CPU_RunInfo);
	
	memset(CPU_RunInfo,0,sizeof(CPU_RunInfo)); //信息缓冲区清零
  vTaskGetRunTimeStats((char *)&CPU_RunInfo);
	usb_printf(
"---------------------------------------------\r\n\
任务名             运行计数            使用率\r\n\
%s\
---------------------------------------------\r\n",
  CPU_RunInfo);
}

/*
对相关时间的封装 - 获取程序Tick运行时间
*/
uint32_t get_prog_system_time_HAL()
{
	return HAL_GetTick();
}

fp32 get_prog_system_time_HAL_1s()
{
	return (fp32)(HAL_GetTick() / Tick_INCREASE_FREQ_HAL_BASED);
}

bool_t get_para_hz_time_freq_signal_HAL(uint8_t freq)
{
	return ( HAL_GetTick() % (Tick_INCREASE_FREQ_HAL_BASED / freq) == 0 );
}

//-------

uint32_t get_prog_system_time_FreeRTOS()
{
	return xTaskGetTickCount();
}

fp32 get_prog_system_time_FreeRTOS_1s()
{
	return (fp32)(xTaskGetTickCount() / Tick_INCREASE_FREQ_FREE_RTOS_BASED);
}

bool_t get_para_hz_time_freq_signal_FreeRTOS(uint8_t freq)
{
		return ( xTaskGetTickCount() % (Tick_INCREASE_FREQ_HAL_BASED / freq) == 0 );
}

/*
需要更加复杂的一种方法: 对于shoot函数, 按键刚按下时就进, 已这个时间为准, 之后固定频率(固定时间间隔) 产生true信号

uint32_t lastTick = 0;

while(1) {
  if(HAL_GetTick() - lastTick >= 1000) {
    lastTick = HAL_GetTick();
    HAL_UART_Transmit(&huart1, (uint8_t*)"One second passed\n", 17, 100);
  }
}

使用: last_tick需由caller准备好, 并初始化好; current_tick=xTaskGetTickCount() or HAL_GetTick(); freq为频率
*/
bool_t get_time_based_freq_signal(uint32_t current_tick, uint32_t* last_tick, uint8_t freq)
{
	if( current_tick - *last_tick >= (uint32_t)(1.0f / ((fp32)freq) * ((fp32)Tick_INCREASE_FREQ_HAL_BASED)) )
	{
		*last_tick = current_tick;
		return 1; //返回true
	}
	else
	{
		return 0; //返回false
	}
}

/*
生成PWM类型的信号 - 当前时间, 周期, 占空比
*/
bool_t generate_signal_pwm(uint32_t time, uint32_t period, fp32 dutyCycle)
{
    uint32_t currentTime = time; //HAL_GetTick();
    uint32_t timeInPeriod = currentTime % period; // 当前时刻在一个周期中的时间
    uint32_t onTime = period * dutyCycle; // 占空比对应的时间

    if (timeInPeriod < onTime) {
        return 1;
    } else {
        return 0;
    }
}
