#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "struct_typedef.h"


#pragma pack(1) // ѹ���ṹ��,ȡ���ֽڶ���

typedef struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num[1];       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;

#pragma pack()

//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);

#endif
