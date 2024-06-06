/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

// ����ң�������ݶ�ȡ,ң����������һ����СΪ2������
#define LAST 1
#define TEMP 0

// ��ȡ��������
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// ������ֵ�Ƿ����
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // ��������ʱ��ֵ
#define RC_SW_MID ((uint16_t)3)  // �����м�ʱ��ֵ
#define RC_SW_DOWN ((uint16_t)2) // ��������ʱ��ֵ
// �����жϿ���״̬�ĺ�
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// ��Ӧkey[x][0~16],��ȡ��Ӧ�ļ�;����ͨ��key[KEY_PRESS][Key_W]��ȡW���Ƿ���,������Ϊλ���ɾ��
#define Key_W 0
#define Key_S 1
#define Key_A 2
#define Key_D 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

/* ----------------------- Data Struct ------------------------------------- */
#pragma anon_unions
// �����Ե�λ��ṹ��,���Լ������������ٶ�
typedef union
{
    struct // ���ڷ��ʼ���״̬
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // ����memcpy������Ҫ����ǿ������ת��
} Key_t;

// @todo ��ǰ�ṹ��Ƕ�׹���,��Ҫ�����Ż�
typedef struct
{
    struct
    {
        int16_t rocker_l_; // ��ˮƽ
        int16_t rocker_l1; // ����ֱ
        int16_t rocker_r_; // ��ˮƽ
        int16_t rocker_r1; // ����ֱ
        int16_t dial;      // ��߲���

        uint8_t switch_left;  // ��࿪��
        uint8_t switch_right; // �Ҳ࿪��
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // ��Ϊλ���ļ�������,�ռ����8��,�ٶ�����16~��

    uint8_t key_count[3][16];
} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t *sbus);
#endif
