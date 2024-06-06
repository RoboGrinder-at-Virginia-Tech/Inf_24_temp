/**
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  * @file       miniPC_comm_task.c/h
  * @brief      miniPC_comm_task.  miniPC communication task and functions
  * @note       The communication task that handles the communication between miniPC <-> TypeC board
	*							both Tx and Rx. 
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-16-2023     Zelin Shen      basic comminication functions
	*
  *
  @verbatim
  ==============================================================================
  Communication between miniPC <-> TypeC board; both Tx and Rx. 
	This task outlines the overall structural logic for communication. 
	General frame structure and unpack mechanism is also defined in this file.
	Specific datas, msg are defined and handled in other files.
	File dependency: this file is used with miniPC_msg, fifo data structure file, 
	RM CRC8_CRC16 files, lower level USART drivers
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 RoboGrinder at Virginia Tech****************************
  */

#include "miniPC_comm_task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "bsp_usart.h"
#include "detect_task.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "detect_task.h"
#include "arm_math.h"
#include "user_lib.h"

#include "miniPC_msg.h"
	
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t embed_receive_comm_task_high_water;
uint32_t embed_send_comm_task_high_water;
#endif

extern UART_HandleTypeDef huart1;

//DMA related
extern DMA_HandleTypeDef hdma_usart1_tx;
//DMA_HandleTypeDef hdma_usart1_rx;
//DMA_HandleTypeDef hdma_usart3_rx;
//DMA_HandleTypeDef hdma_usart6_rx;
//DMA_HandleTypeDef hdma_usart6_tx;

void pc_unpack_fifo_data(void);

	
fifo_s_t pc_comm_fifo;
uint8_t pc_comm_fifo_buf[MINIPC_COMM_RX_FIFO_BUF_LENGTH];
uint8_t pc_comm_usart1_buf[2][MINIPC_COMM_UART_DMA_RX_BUF_LENGHT];
pc_comm_unpack_data_t pc_comm_unpack_data_obj;

//send related var
pc_comm_embed_uart_send_data_t embed_send;
uint8_t embed_send_fifo_buf[MINIPC_COMM_TX_FIFO_BUF_LENGTH];
//uint8_t embed_send_usart1_buf[2][MINIPC_COMM_UART_DMA_TX_BUF_LENGHT];
uint8_t embed_send_usart1_buf[MINIPC_COMM_UART_DMA_TX_BUF_LENGHT];

/**pc_communication_task; this is the main task
This task is for receiving the information from cv; pc_to_embed
pc -> embed
we call this the "Main" task becase embed receive is more important based on the functionnality

*/
void embed_receive_Main_communication_task(void const *pvParameters)
{
	init_pc_to_embed_Main_comm_struct_data();
	fifo_s_init(&pc_comm_fifo, pc_comm_fifo_buf, MINIPC_COMM_RX_FIFO_BUF_LENGTH);
	
	//the is needed for both tasks
	usart1_init(pc_comm_usart1_buf[0], pc_comm_usart1_buf[1], MINIPC_COMM_UART_DMA_RX_BUF_LENGHT);
	
	//miniPC_info.miniPC_connection_status = miniPC_offline;//init connection status
	
	while(1)
	{
		//PC-->Embeded; received data unpack
		pc_unpack_fifo_data();
		
		//osDelay(10);
		vTaskDelay(100);
		
		//record high water mark
#if INCLUDE_uxTaskGetStackHighWaterMark
        embed_receive_comm_task_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

/**pc_communication_task
This task is for sending the information from cv; embed_to_pc
embed -> pc
*/
void embed_send_communication_task(void const *pvParameters)
{
	
	fifo_s_init(&embed_send.tx_fifo, embed_send_fifo_buf, MINIPC_COMM_TX_FIFO_BUF_LENGTH);
	
	//need to wait for the main comm task to finish init, wait for usart1_init(...) which is in main task
	//usart1_tx_dma_init was called in main
//	vTaskDelay(300); 
	
	//send msg data struct init
	init_embed_to_pc_comm_struct_data();
	
	embed_send.tx_dma_buf = &embed_send_usart1_buf[0];
	embed_send.tx_dma_buf_size = sizeof(embed_send_usart1_buf);
	embed_send.status = 0;
	
	vTaskDelay(1000);  //wait here not above - 300多等一会儿保证gimbal和其它task已经启动
	
	while(1)
	{

		//Embeded-->PC; send data to PC
		embed_send_data_to_pc_loop();
		
		//osDelay(10);
		//osDelay(4);
		//osDelay(100);
		//vTaskDelay(1000);
		vTaskDelay(2); // 4
		
		//record high water mark
#if INCLUDE_uxTaskGetStackHighWaterMark
        embed_send_comm_task_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}


void pc_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = PC_HEADER_SOF;
  pc_comm_unpack_data_t *p_obj = &pc_comm_unpack_data_obj;

  while ( fifo_s_used(&pc_comm_fifo) )
  {
    byte = fifo_s_get(&pc_comm_fifo);
    switch(p_obj->unpack_step)
    {
      case PC_COMM_STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = PC_COMM_STEP_FRAME_LENGTH;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
			
			//data_len - uint8_t for pc comm
			case PC_COMM_STEP_FRAME_LENGTH:
			{
				p_obj->frame_len = byte;
				p_obj->protocol_packet[p_obj->index++] = byte;
				
				//check data_len to avoid out of bound array ptr
				if(p_obj->frame_len < PC_PROTOCOL_FRAME_MAX_SIZE) //(PC_PROTOCOL_FRAME_MAX_SIZE - PC_HEADER_CRC_CMDID_LEN))
        {
					//also check current index pos, current index position = array index for next seq field
					//DJI referee did not check this current index position
					if(p_obj->index == PC_HEADER_INDEX_FOR_SEQ)
					{
						p_obj->unpack_step = PC_COMM_STEP_SEQ; //PC_COMM_STEP_END_CRC16; //PC_COMM_STEP_CMDID;
					}
					else
					{
						p_obj->unpack_step = PC_COMM_STEP_HEADER_SOF;
						p_obj->index = 0;
					}
        }
        else
        {
          p_obj->unpack_step = PC_COMM_STEP_HEADER_SOF;
          p_obj->index = 0;
        }
			}break;
			
			//unpack sequence information
			case PC_COMM_STEP_SEQ:
			{
				p_obj->protocol_packet[p_obj->index++] = byte;
				p_obj->unpack_step = PC_COMM_STEP_CRC8;
			}break;
			
			//check for header CRC8
			case PC_COMM_STEP_CRC8:
			{
				p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == PC_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, PC_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = PC_COMM_STEP_CMDID_LOW;
          }
          else
          {
            p_obj->unpack_step = PC_COMM_STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
			}break;
			
			//unpack cmd_id information for debug
			case PC_COMM_STEP_CMDID_LOW:
			{
				p_obj->cmd_id = byte;
				p_obj->protocol_packet[p_obj->index++] = byte;
				p_obj->unpack_step = PC_COMM_STEP_CMDID_HIGH;
			}break;
			
			case PC_COMM_STEP_CMDID_HIGH:
			{
				p_obj->cmd_id |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;
				p_obj->unpack_step = PC_COMM_STEP_END_CRC16;
			}break;
			
			case PC_COMM_STEP_END_CRC16:
			{
				if (p_obj->index < p_obj->frame_len) //(PC_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
					 //copy data from(pop) fifo and put into unpack array-ram buffer
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= p_obj->frame_len) //(PC_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = PC_COMM_STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, p_obj->frame_len) ) //verify_CRC16_check_sum(p_obj->protocol_packet, PC_HEADER_CRC_CMDID_LEN + p_obj->data_len)
          {
            pc_comm_data_solve(p_obj->protocol_packet); //solve the data with detail data sturct
						//detect_hook(PC_TOE); in the other func
          }
        }
			}break;

      default:
      {
        p_obj->unpack_step = PC_COMM_STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

/*
Summer 2022 SZL 更改了几个对 flag的识别方式
Spring 2023 SZL upgrades miniPC comm, IRQ also need to handle tx 
*/
void USART1_IRQHandler(void)
{		
		//SZL 1-20-23 IRQ need to handle DMA tx
		if((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) || (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)))
		{
				HAL_UART_IRQHandler(&huart1);
		}
		
		if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))//huart1.Instance->SR & UART_FLAG_RXNE)//data msg received
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1); //SZL 5-30-2022
				__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
    }
    else if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        static uint16_t this_time_rx_len = 0;

        //__HAL_UART_CLEAR_PEFLAG(&huart1); //SZL 5-30-2022
				__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			
        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = MINIPC_COMM_UART_DMA_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

            //reset set_data_lenght
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = MINIPC_COMM_UART_DMA_RX_BUF_LENGHT;

            //set memory buffer 1
            //设定缓冲区1
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
						
						//SZL 1-20 removed this func
						//HAL_UART_Receive_DMA(&huart1,(uint8_t*)pc_rx_buf[0], this_time_rx_len);
						//pc_command_unpack((uint8_t*)pc_rx_buf[0], this_time_rx_len);
						fifo_s_puts(&pc_comm_fifo, (char*)pc_comm_usart1_buf[0], this_time_rx_len);
						detect_hook(PC_TOE); 
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
		  			//MINIPC_COMM_UART_DMA_RX_BUF_LENGHT - huart1.hdmarx->Instance->NDTR; //SZL change calc on RHS
            this_time_rx_len = MINIPC_COMM_UART_DMA_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

            //reset set_data_lenght
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = MINIPC_COMM_UART_DMA_RX_BUF_LENGHT;

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);

						//SZL 1-20 removed this func
						//HAL_UART_Receive_DMA(&huart1,(uint8_t*)pc_rx_buf[1], this_time_rx_len);
						//pc_command_unpack((uint8_t*)pc_rx_buf[1], this_time_rx_len);
						fifo_s_puts(&pc_comm_fifo, (char*)pc_comm_usart1_buf[1], this_time_rx_len);
						detect_hook(PC_TOE); 
        }
    } 	
		//HAL_UART_IRQHandler(&huart1);
}

/* -------------------------------- USART SEND -------------------------------- */
/**
 * @brief  串口dma发送完成中断处理 Serial port dma sending completes interrupt isr
 * for
 * @param  
 * @retval 
 */
void uart1_tx_dma_done_isr() //(struct __DMA_HandleTypeDef * hdma)
{
 	embed_send.status = 0;	//DMA send in idle
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		uart1_tx_dma_done_isr();
	}
}

/**
 * @brief  循环从串口发送fifo读出数据，放置于dma发送缓存，并启动dma传输
 *					The loop sends the fifo read-out data from the serial port, 
 *					places it in the dma send cache, and initiates the dma transfer.
 * DMA2 Stream 7 hdma->XferM1CpltCallback(hdma);
 *  void (* XferCpltCallback)(struct __DMA_HandleTypeDef * hdma);  DMA transfer complete callback 
 * @param  
 * @retval 
 */
uint8_t uart1_poll_dma_tx()
{
	int size = 0;
	
	if (embed_send.status == 0x01)
  {
        return 1; //did not sent out the data
  }
//	size = fifo_read(&s_uart_dev[uart_id].tx_fifo, s_uart_dev[uart_id].dmatx_buf,
//					 s_uart_dev[uart_id].dmatx_buf_size);
	
	size = fifo_s_gets(&embed_send.tx_fifo, (char *)embed_send.tx_dma_buf, embed_send.tx_dma_buf_size);
	embed_send.debug_fifo_size = size; //update debug buff size cnt
	
	if (size != 0 && size > 0)
	{	
    embed_send.debug_UartTxCount += size; //update tx count
		
		/* DMA发送状态,必须在使能DMA传输前置位，否则有可能DMA已经传输并进入中断 */
		embed_send.status = 0x01;
		usart1_tx_dma_enable(embed_send.tx_dma_buf, size);
		
	}
	return 0; //successfully enable send or no data to send
}

/**
 * @brief  串口驱动映射 Serial drive overwrite
 * overwrite to put msg in fifo, producer will call this function in their function
 * @param  
 * @retval 
 */
void uart1_embed_send_byte(uint8_t ch) //(unsigned char ch)
{
	fifo_s_puts(&embed_send.tx_fifo, (char*)&ch, 1);
}

uint8_t get_uart1_embed_send_status()
{
	if (embed_send.status == 0x00)
	{
		return 0; //not busy
	}
	else if(embed_send.status == 0x01)
	{
		return 1; //busy
	}
	else
	{
		return 1; //busy
	}
}

///**
// * @brief refresh data struct to ring buffer
// *
// * @param variable lengthed data
// */
//void embed_send_refresh(int cnt,...)
//{
//	
//}

///**
// * @brief 串口设备初始化 The serial port device is init
// * NOT USED, init process in task
// * @param  
// * @retval 
// */
//void uart1_app_tx_init()
//{
//	return;
//}

/* -------------------------------- USART SEND END-------------------------------- */
