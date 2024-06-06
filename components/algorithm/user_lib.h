#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"
#include "arm_math.h"

#define DEFINE_ARM_MATRIX_MACRO_IN_USER_LIB_H 1

#if DEFINE_ARM_MATRIX_MACRO_IN_USER_LIB_H == 1
#define mat         arm_matrix_instance_f32 
#define mat_64      arm_matrix_instance_f64
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64
#endif

//快速开方
extern fp32 invSqrt_user_lib(fp32 num);
//绝对限制
extern void abs_limit(fp32 *num, fp32 Limit);
//判断符号位
extern fp32 sign(fp32 value);
//浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
extern uint8_t uint8_constrain(uint8_t Value, uint8_t minValue, uint8_t maxValue);
//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);

extern uint16_t min_uint16(uint16_t a, uint16_t b); //比较min

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

//Shooter SZL 5-19-2022
//弧度格式化为-PI~PI
#define shooter_rad_format(Ang) loop_fp32_constrain((Ang), 0, 2*PI)

#endif
