#include "linear_throttle.h"
#include "arm_math.h"
#include "struct_typedef.h"

void linear_throttle_init(linear_throttle_t *linear_throttle, fp32 frame_period, fp32 abs_target, fp32 abs_init)
{
    linear_throttle->frame_period = frame_period;
    linear_throttle->abs_target = abs_target;
    linear_throttle->abs_init = abs_init;
    linear_throttle->step = 0.0f;
    linear_throttle->out = 0.0f;
}

//判断符号位
static fp32 sign(fp32 value)
{
	if (value >= 0.0f)
	{
		return 1.0f;
	}
	else
	{
		return -1.0f;
	}
}

//限幅函数
static fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

void linear_throttle_calc(linear_throttle_t *linear_throttle, fp32 step)
{
  linear_throttle->step = step;
	
	//由step符号判断 init和target的符号
	if(sign(linear_throttle->step) > 0.0f)
	{
		// 往正方向
		
//		if (linear_throttle->out > linear_throttle->abs_target)
//    {
//      linear_throttle->out = linear_throttle->abs_target;
//    }
//    else if (linear_throttle->out < linear_throttle->abs_init)
//    {
//      linear_throttle->out = linear_throttle->abs_init;
//    }
		
		// 前序判断
		linear_throttle->out = fp32_constrain(linear_throttle->out, linear_throttle->abs_init,  linear_throttle->abs_target);
		
		// 按频率累加
		linear_throttle->out += linear_throttle->step * linear_throttle->frame_period;
		
		// 后置判断
		linear_throttle->out = fp32_constrain(linear_throttle->out, linear_throttle->abs_init,  linear_throttle->abs_target);
	}
	else
	{
		// 往负方向
		
//		if (linear_throttle->out < -linear_throttle->abs_target)
//    {
//      linear_throttle->out = -linear_throttle->abs_target;
//    }
//    else if (linear_throttle->out > -linear_throttle->abs_init)
//    {
//      linear_throttle->out = -linear_throttle->abs_init;
//    }
		
		// 前序判断
		linear_throttle->out = fp32_constrain(linear_throttle->out, -linear_throttle->abs_target,  -linear_throttle->abs_init);
		
		// 按频率累加
		linear_throttle->out += linear_throttle->step * linear_throttle->frame_period;
		
		// 后置判断
		linear_throttle->out = fp32_constrain(linear_throttle->out, -linear_throttle->abs_target,  -linear_throttle->abs_init);
	}
}

void linear_throttle_clear_out(linear_throttle_t *linear_throttle)
{
    linear_throttle->out = 0.0f;
}
