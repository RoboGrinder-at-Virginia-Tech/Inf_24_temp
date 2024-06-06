#ifndef RAMP_FUNCTION_H
#define RAMP_FUNCTION_H

#include "struct_typedef.h"


#pragma pack(1) // 压缩结构体,取消字节对齐

typedef struct
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
		fp32 max_value_constant; // 限幅最大值的常量, 限幅最大值可能会动态的变动, 取决于应用场景
    fp32 frame_period; //时间间隔
} ramp_function_source_t;

#pragma pack()

//斜波函数初始化
extern void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//斜波函数计算
extern void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

#endif
