#ifndef RAMP_FUNCTION_H
#define RAMP_FUNCTION_H

#include "struct_typedef.h"


#pragma pack(1) // ѹ���ṹ��,ȡ���ֽڶ���

typedef struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
		fp32 max_value_constant; // �޷����ֵ�ĳ���, �޷����ֵ���ܻᶯ̬�ı䶯, ȡ����Ӧ�ó���
    fp32 frame_period; //ʱ����
} ramp_function_source_t;

#pragma pack()

//б��������ʼ��
extern void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
extern void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

#endif
