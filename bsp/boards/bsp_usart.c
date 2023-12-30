#include "bsp_usart.h"
#include "main.h"
#include "miniPC_comm_task.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx; //PR test 2021/2/27
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;


//PR test 2021/3/13
/*
USART 1 - miniPC comm
USART 6 - referee */

/* ---------------------------------------------- USART 1 ---------------------------------------------- */

/*
SZL 1-21-2023 modified for miniPC comm uart dma send
*/

void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;

//		//configure call back funciton 配置中断回调函数
//		hdma_usart1_tx.XferCpltCallback = &uart1_tx_dma_done_isr;

}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
//    //disable DMA
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_usart1_tx);

//    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_tx);
//    }

//    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

//    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
//    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

//    __HAL_DMA_ENABLE(&hdma_usart1_tx);
//		
//		//是否需要等待 DMA数据流有效
		
		//版本2 2-3-2023
		//HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
		//disable DMA
		__HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
		
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
		
		HAL_UART_Transmit_DMA(&huart1, data, len);
}


/*PRtest 2021/2/27通讯*/
void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF2);

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

		//是否需要等待 DMA数据流有效
		/*
		  //等待DMA数据流有效
			while(DMA_GetCmdStatus(DEBUG_USART_DMA_STREAM) != ENABLE)
			{
			}   
			stm32f4xx_dma.c
		*/
		
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
}

/* ---------------------------------------------- USART 6 ---------------------------------------------- */

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
		
		//是否需要等待 DMA数据流有效

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

/* -------------------------------------------------- New DMA -------------------------------------------------- */
/*
SZL 1-20-2023 Board support package for DMA uart send

Draft: HAL_DMA_STATE_READY

Note: the order to call and use these functions:
usart1_init(...)
usart1_tx_dual_buff_dma_init_no_enable(...)
usart1_tx_dma_dual_buff_enable(....)

更具研究 双DMA发送方式并不可取 以下函数均未使用

*/

/*
SZL 1-21-2023
Add dual buffer UART Tx DMA
Note that user need to call this function after regular usart1_init(...)
*/
void usart1_tx_dual_buff_dma_init_no_enable(uint8_t *tx1_buf, uint8_t *tx2_buf, uint16_t dma_tx_buf_num)
{
	  //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
		//1-21-23 add
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_LISR_TCIF2);
		
    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
		//memory buffer 1
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(tx1_buf); //(NULL);
		//memory buffer 2
		hdma_usart1_tx.Instance->M1AR = (uint32_t)(tx2_buf);
		
//    hdma_usart1_tx.Instance->NDTR = 0;
		//data length
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, dma_tx_buf_num);
		
		//enable double memory buffer
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
}

/*
Enable send with para. memory address and size
*/
void usart1_tx_dual_buff_dma_enable(uint8_t *tx1_buf, uint8_t *tx2_buf, uint16_t dma_tx_buf_num) //(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    //hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
		//memory buffer 1
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(tx1_buf); //(NULL);
		//memory buffer 2
		hdma_usart1_tx.Instance->M1AR = (uint32_t)(tx2_buf);
		
    //__HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);
		//data length
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, dma_tx_buf_num);
		
		//Enable double memory buffer
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
		//是否需要等待 DMA数据流有效
}

/*
Send function, to send a fixed length of data
This function will find the avaliable DMA, and write data to that dma buff
*/
void usart1_tx_dual_buff_dma_cmd_send(uint8_t *tx_buf, uint16_t len)
{
	
}

/* -------- Other Support Functions -------- */
/* This function to get the usart1 dma states

in file: stm32f4xx.h
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

usart1_get_tx_dma_tc_state() = 0RESET trans not completed; = 1SET completed */
FlagStatus usart1_get_tx_dma_tc_state(void)
{
		//HAL_DMA_GetState(&hdma_usart1_rx) == HAL_DMA_STATE_READY;
	  if(__HAL_DMA_GET_FLAG(&hdma_usart1_tx, DMA_FLAG_TCIF3_7) == SET)
		{
			__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_FLAG_TCIF3_7);
			return SET; //1;
		}
		else
		{
			return RESET; //0;
		}
	
//    if(DMA_GetFlagStatus(DMA1_FLAG_TC4) == SET)
//    {
//        DMA_ClearFlag(DMA1_FLAG_TC4);
//        
//        return true;
//    }
//    else
//    {
//        return false;
//    }
} 
