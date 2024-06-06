#ifndef LINEAR_THROTTLE_H
#define LINEAR_THROTTLE_H

#include "struct_typedef.h"


#pragma pack(1) // 压缩结构体,取消字节对齐

typedef struct
{
	fp32 step;        // 输入数据 每秒希望增长数值 - 油门步进
	fp32 abs_init;				// 限幅最小值, 降低加速迟滞 - 初始加速值
  fp32 abs_target;      // 限幅最大值, 最终速度 - 全油门速度
	
  fp32 frame_period; //时间间隔
	
	fp32 out;
	
} linear_throttle_t;

#pragma pack()

//斜波函数初始化
extern void linear_throttle_init(linear_throttle_t *linear_throttle, fp32 frame_period, fp32 abs_target, fp32 abs_init);

//斜波函数计算
extern void linear_throttle_calc(linear_throttle_t *linear_throttle, fp32 step);

extern void linear_throttle_clear_out(linear_throttle_t *linear_throttle);

#endif
