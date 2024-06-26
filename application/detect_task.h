/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      detect error task, judged by receiving data time. provide detect
                hook function, error exist function.
  *             检测错误任务， 通过接收数据时间来判断.提供 检测钩子函数,错误存在函数.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *
  @verbatim
  ==============================================================================
    add a sensor 
    1. in detect_task.h, add the sensor name at the end of errorList,like
    enum errorList
    {
        ...
        XXX_TOE,    //new sensor
        ERROR_LIST_LENGHT,
    };
    2.in detect_init function, add the offlineTime, onlinetime, priority params,like
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3. if XXX_TOE has data_is_error_fun ,solve_lost_fun,solve_data_error_fun function, 
        please assign to function pointer.
    4. when XXX_TOE sensor data come, add the function detect_hook(XXX_TOE) function.
    如果要添加一个新设备
    1.第一步在detect_task.h，添加设备名字在errorList的最后，像
    enum errorList
    {
        ...
        XXX_TOE,    //新设备
        ERROR_LIST_LENGHT,
    };
    2.在detect_init函数,添加offlineTime, onlinetime, priority参数
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.如果有data_is_error_fun ,solve_lost_fun,solve_data_error_fun函数，赋值到函数指针
    4.在XXX_TOE设备数据来的时候, 添加函数detect_hook(XXX_TOE).
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"


#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//错误码以及对应设备顺序
enum errorList
{
    DBUS_TOE = 0,
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
    YAW_GIMBAL_MOTOR_TOE,
    PITCH_GIMBAL_MOTOR_L_TOE, // old PITCH_GIMBAL_MOTOR_TOE
	  PITCH_GIMBAL_MOTOR_R_TOE,
    TRIGGER_MOTOR17mm_L_TOE, // old TRIGGER_MOTOR_TOE
	  TRIGGER_MOTOR17mm_R_TOE,
    BOARD_GYRO_TOE,
    BOARD_ACCEL_TOE,
    BOARD_MAG_TOE,
    REFEREE_TOE,
    RM_IMU_TOE,
    OLED_TOE,
	  PC_TOE, //SZL 1-18-2023 change MINIPC_TOE to PC_TOE
		ZIDACAP_TOE, //SZL 2-12-2022 添加
	  GEN2CAP_TOE, //SZL 12-27-2022 添加 新的超级电容
	  GEN3CAP_TOE, // szl 3-19-2024 第三代超级电容
	  WULIECAP_TOE, //雾列超级电容 TOE
		SHOOT_FRIC_L_TOE,
		SHOOT_FRIC_R_TOE,
    ERROR_LIST_LENGHT,
};

typedef __packed struct
{
    uint32_t new_time;
    uint32_t last_time;
    uint32_t lost_time;
    uint32_t work_time;
    uint16_t set_offline_time : 12;
    uint16_t set_online_time : 12;
    uint8_t enable : 1;
    uint8_t priority : 4;
    uint8_t error_exist : 1;
    uint8_t is_lost : 1;
    uint8_t data_is_error : 1;

    fp32 frequency;
    bool_t (*data_is_error_fun)(void);//检查data是否出错
    void (*solve_lost_fun)(void);//解决设备离线
    void (*solve_data_error_fun)(void);//解决data出错
} error_t;
/*
//SZL 3-12-2022注释: set online time 定义后只有这里用了; 
new_time 通信中断发生后 赋值为 xTaskGetTickCount() 即当前系统时间;
lost_time detect task中 如果确定掉线 就把当前system_time赋值给它
work_time设备上一次正常工作的系统时间 
set_offline_time; 设置离线时间 system_time - error_list[i].new_time 大于此时间时 判定为设备离线
set_online_time; 设备上电后 或接入后 需要这么多时间才能开始正常工作
-> else if (system_time - error_list[i].work_time < error_list[i].set_online_time) -> 刚刚上线，可能存在数据不稳定，只记录不丢失 -> .is_lost = 0; .error_exist = 1;
几个用在detect task中的逻辑关系: 
1) 设备offline时，error一定exist; 但当error_exist = 1时 设备也可能在线 就是单纯数据出错了
2) 当error_exist = 0, 即设备数据不出错时 该设备一定online 

is_lost = 1的时候是设备离线

priority用于 判断错误优先级， 保存优先级最高的错误码, 优先级最高的保留到error_num_display
*/

/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          检测任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void detect_task(void const *pvParameters);

/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
/**
  * @brief          获取设备对应的错误状态
  * @param[in]      toe:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
extern bool_t toe_is_error(uint8_t err);

/**
  * @brief          record the time
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
extern void detect_hook(uint8_t toe);

/**
  * @brief          get error list
  * @param[in]      none
  * @retval         the point of error_list
  */
/**
  * @brief          得到错误列表
  * @param[in]      none
  * @retval         error_list的指针
  */
extern const error_t *get_error_list_point(void);

#endif
