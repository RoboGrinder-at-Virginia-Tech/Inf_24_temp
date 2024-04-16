/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      ��̬�����м�㣬Ϊ��̬�����ṩ��غ���
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "arm_math.h"
#include <math.h>
#include "main.h"
/**
 * @brief          ���ڻ�ȡ��ǰ�߶�
 * @author         RM
 * @param[in]      �߶ȵ�ָ�룬fp32
 * @retval         ���ؿ�
 */

void AHRS_get_height(fp32* high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

/**
 * @brief          ���ڻ�ȡ��ǰγ��
 * @author         RM
 * @param[in]      γ�ȵ�ָ�룬fp32
 * @retval         ���ؿ�
 */

void AHRS_get_latitude(fp32* latitude)
{
    if (latitude != NULL)
    {
        *latitude = 22.0f;
    }
}

/**
 * @brief          ���ٿ���������
 * @author         RM
 * @param[in]      ������Ҫ�����ĸ�������fp32
 * @retval         ����1/sqrt ������ĵ���
 */

fp32 AHRS_invSqrt(fp32 num)
{
    return 1/sqrtf(num);

//    fp32 halfnum = 0.5f * num;
//    fp32 y = num;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(fp32*)&i;
//    y = y * (1.5f - (halfnum * y * y));
//    y = y * (1.5f - (halfnum * y * y));
//    return y;
}

/**
 * @brief          sin����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�sinֵ
 */

fp32 AHRS_sinf(fp32 angle)
{
    return arm_sin_f32(angle);
}
/**
 * @brief          cos����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�cosֵ
 */

fp32 AHRS_cosf(fp32 angle)
{
    return arm_cos_f32(angle);
}

/**
 * @brief          tan����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�tanֵ
 */

fp32 AHRS_tanf(fp32 angle)
{
    return tanf(angle);
}
/**
 * @brief          ����32λ�������ķ����Ǻ��� asin����
 * @author         RM
 * @param[in]      ����sinֵ�����1.0f����С-1.0f
 * @retval         ���ؽǶ� ��λ����
 */

fp32 AHRS_asinf(fp32 sin)
{

    return asinf(sin);
}

/**
 * @brief          �����Ǻ���acos����
 * @author         RM
 * @param[in]      ����cosֵ�����1.0f����С-1.0f
 * @retval         ���ض�Ӧ�ĽǶ� ��λ����
 */

fp32 AHRS_acosf(fp32 cos)
{

    return acosf(cos);
}

/**
 * @brief          �����Ǻ���atan����
 * @author         RM
 * @param[in]      ����tanֵ�е�yֵ ����������С������
 * @param[in]      ����tanֵ�е�xֵ ����������С������
 * @retval         ���ض�Ӧ�ĽǶ� ��λ����
 */

fp32 AHRS_atan2f(fp32 y, fp32 x)
{
    return atan2f(y, x);
}

/*
11-11:
Operating on quaternion or euler angle
*/

// ��Ԫ��תŷ����
Euler Quaternion_to_Euler(Quaternion q) {
    Euler angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysignf(PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);

    return angles;
}

// ŷ����ת��Ԫ��
Quaternion Euler_to_Quaternion(Euler angles) {
    Quaternion q;
    float cy = cosf(angles.yaw * 0.5f);
    float sy = sinf(angles.yaw * 0.5f);
    float cp = cosf(angles.pitch * 0.5f);
    float sp = sinf(angles.pitch * 0.5f);
    float cr = cosf(angles.roll * 0.5f);
    float sr = sinf(angles.roll * 0.5f);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// ����pitch�Ƕȵĺ���
Quaternion RemovePitch(Quaternion q) {
    Euler angles = Quaternion_to_Euler(q);

    // ��pitch�Ƕ�����Ϊ0
    angles.pitch = 0;

    // ���޸ĺ��ŷ����ת������Ԫ��
    return Euler_to_Quaternion(angles);
}

