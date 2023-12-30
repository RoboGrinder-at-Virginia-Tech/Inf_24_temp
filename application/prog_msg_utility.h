#ifndef __PROG_MSG_UTILITY__
#define __PROG_MSG_UTILITY__

#include "prog_msg_utility.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "struct_typedef.h"

/*
对相关时间的封装 - 获取程序Tick运行时间
*/
//millisecond; 1000Hz - 每秒增加1000
#define Tick_INCREASE_FREQ_HAL_BASED 1000

//millisecond; 1000Hz - 每秒增加1000
#define Tick_INCREASE_FREQ_FREE_RTOS_BASED 1000

uint32_t get_prog_system_time_HAL(void);
bool_t get_para_hz_time_freq_signal_HAL(uint8_t freq);
fp32 get_prog_system_time_HAL_1s(void);

uint32_t get_prog_system_time_FreeRTOS(void);
bool_t get_para_hz_time_freq_signal_FreeRTOS(uint8_t freq);
fp32 get_prog_system_time_FreeRTOS_1s(void);

void CPU_info_to_usb(void);
void init_prog_msg_utility(void);
uint32_t get_util_1s_time_cnt(void);
const char* util_1s_time_cnt_toString(void);
bool_t get_time_based_freq_signal(uint32_t current_tick, uint32_t* last_tick, uint8_t freq);
bool_t generate_signal_pwm(uint32_t time, uint32_t period, fp32 dutyCycle);

#endif
