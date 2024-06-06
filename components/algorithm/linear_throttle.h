#ifndef LINEAR_THROTTLE_H
#define LINEAR_THROTTLE_H

#include "struct_typedef.h"


#pragma pack(1) // ѹ���ṹ��,ȡ���ֽڶ���

typedef struct
{
	fp32 step;        // �������� ÿ��ϣ��������ֵ - ���Ų���
	fp32 abs_init;				// �޷���Сֵ, ���ͼ��ٳ��� - ��ʼ����ֵ
  fp32 abs_target;      // �޷����ֵ, �����ٶ� - ȫ�����ٶ�
	
  fp32 frame_period; //ʱ����
	
	fp32 out;
	
} linear_throttle_t;

#pragma pack()

//б��������ʼ��
extern void linear_throttle_init(linear_throttle_t *linear_throttle, fp32 frame_period, fp32 abs_target, fp32 abs_init);

//б����������
extern void linear_throttle_calc(linear_throttle_t *linear_throttle, fp32 step);

extern void linear_throttle_clear_out(linear_throttle_t *linear_throttle);

#endif
