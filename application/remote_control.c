/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"



//ң����������������
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


//ȡ������
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//ң�������Ʊ���
// ң��������
static RC_ctrl_t rc_ctrl[2];     //[0]:��ǰ����TEMP,[1]:��һ�ε�����LAST.���ڰ����������º��л����ж�

//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return rc_ctrl;
}

//�ж�ң���������Ƿ����
uint8_t RC_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (RC_abs(rc_ctrl[TEMP].rc.rocker_r_) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl[TEMP].rc.rocker_r1) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl[TEMP].rc.rocker_l_) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl[TEMP].rc.rocker_l1) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl[TEMP].rc.switch_left == 0)
    {
        goto error;
    }
    if (rc_ctrl[TEMP].rc.switch_right == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl[TEMP].rc.rocker_r_ = 0;
    rc_ctrl[TEMP].rc.rocker_r1 = 0;
    rc_ctrl[TEMP].rc.rocker_l_ = 0;
    rc_ctrl[TEMP].rc.rocker_l1 = 0;
    rc_ctrl[TEMP].rc.dial = 0;
    rc_ctrl[TEMP].rc.switch_left = RC_SW_MID;
    rc_ctrl[TEMP].rc.switch_right = RC_SW_DOWN;
    rc_ctrl[TEMP].mouse.x = 0;
    rc_ctrl[TEMP].mouse.y = 0;
    rc_ctrl[TEMP].mouse.z = 0;
    rc_ctrl[TEMP].mouse.press_l = 0;
    rc_ctrl[TEMP].mouse.press_r = 0;
    rc_ctrl[TEMP].key[0].keys = 0;
		rc_ctrl[TEMP].key[1].keys = 0;
		rc_ctrl[TEMP].key[2].keys = 0;
		memset(rc_ctrl[TEMP].key_count, 0, sizeof(rc_ctrl[TEMP].key_count));
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//�����ж�
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], rc_ctrl);
                //��¼���ݽ���ʱ��
                detect_hook(DBUS_TOE);
                //sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_rc(sbus_rx_buf[1], rc_ctrl);
                //��¼���ݽ���ʱ��
                detect_hook(DBUS_TOE);
                //sbus_to_usart1(sbus_rx_buf[1]); //PR test 5/22
            }
        }
    }

}

//ȡ������
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
 * @brief ����ң����ҡ�˵�ֵ,����660����С��-660��ֵ����Ϊ����Чֵ,��0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (RC_abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
		if (sbus_buf == NULL || rc_ctrl == NULL)
		{
			return;
		}
	
		// ҡ��,ֱ�ӽ���ʱ��ȥƫ��
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // ��ದ��
    RectifyRCjoystick();
    // ����,0��1��
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // ������
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.z = (sbus_buf[10] | (sbus_buf[11] << 8)); //!< Mouse Z axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  λ��İ���ֵ����,ֱ��memcpy����,ע��С�˵��ֽ���ǰ,��lsb�ڵ�һλ,msb�����
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl������
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift������
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // ��ǰ�����Ƿ���
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // ��һ�ΰ����Ƿ���
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // ��ǰctrl��ϼ��Ƿ���
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  ��ǰshift��ϼ��Ƿ���
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // ��һ��ctrl��ϼ��Ƿ���
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // ��һ��shift��ϼ��Ƿ���

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5λΪctrl��shift,ֱ������
            continue;
        // �����ǰ��������,��һ�ΰ���û�а���,��ctrl��shift��ϼ�û�а���,�򰴼����¼�����1(��⵽������)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // ��ǰctrl��ϼ�����,��һ��ctrl��ϼ�û�а���,��ctrl��ϼ����¼�����1(��⵽������)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // ��ǰshift��ϼ�����,��һ��shift��ϼ�û�а���,��shift��ϼ����¼�����1(��⵽������)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // ������һ�ε�����,���ڰ����������º��л����ж�
}

/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          ͨ��usart1����sbus����,��usart3_IRQHandle����
  * @param[in]      sbus: sbus����, 18�ֽ�
  * @retval         none
  */
void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

