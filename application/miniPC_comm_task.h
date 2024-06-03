#ifndef __MINIPC_COMM_TASK__
#define __MINIPC_COMM_TASK__

#include "main.h"
#include "struct_typedef.h"
#include "fifo.h"

//Declare
extern void embed_receive_Main_communication_task(void const *pvParameters);
extern void embed_send_communication_task(void const *pvParameters);
extern void uart1_embed_send_byte(uint8_t ch);
extern uint8_t get_uart1_embed_send_status(void);
extern uint8_t uart1_poll_dma_tx(void);
extern void uart1_tx_dma_done_isr(void); //(struct __DMA_HandleTypeDef * hdma);

/* 
Path for the comm and Flow of data: 
miniPC UART -> UART wires -> Embedded UART Peripherals with DMA-> DMA buff (interupt)-> software fifo -> unpack array-ram buffer
note 1-17-2023: current data size did not exceed 50 bytes
*/

//this value is used for fifo - software fifo buffer
#define MINIPC_COMM_RX_FIFO_BUF_LENGTH 128 //1024

//This is the DMA buff length
#define MINIPC_COMM_UART_DMA_RX_BUF_LENGHT 50 //512

#define PC_HEADER_SOF 0xAF
//size for the protocal unpack array - ram buffer
#define PC_PROTOCOL_FRAME_MAX_SIZE  50 //128

//Information for different packages' size
#define PC_PROTOCOL_HEADER_SIZE            sizeof(pc_comm_frame_header_t) //4
#define PC_PROTOCOL_CMD_SIZE               2   //sizeof(uint16_t) size for the cmd_id
#define PC_PROTOCOL_CRC16_SIZE             2   //sizeof(uint16_t) size for the CRC16
#define PC_HEADER_CRC_LEN                  (PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_CRC16_SIZE)
#define PC_HEADER_CRC_CMDID_LEN            (PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_CRC16_SIZE + PC_PROTOCOL_CMD_SIZE) //(PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define PC_HEADER_CMDID_LEN                (PC_PROTOCOL_HEADER_SIZE + PC_PROTOCOL_CMD_SIZE) //(PC_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#define PC_HEADER_INDEX_FOR_SEQ						2   //this marks the index in protocol_packet[....]

/*
[...] in unit uint8_t
(frame_header = 	Header/SOF[0] +	ver_frame_len[1]  +	seq[2] +	CRC8[3]) + cmd_id/package id[4][5] + Data[TBD] + frame_tail(CRC16)[ver_frame_len-1]
*/

#pragma pack(push, 1)

typedef enum
{
		//Embeded-->PC
    BASE_INFO_CMD_ID = 0x0204,
    GIMBAL_INFO_CMD_ID = 0x0304,
		REFEREE_INFO_CMD_ID = 0xF001,
	
		//PC-->Embeded
    CHASSIS_REL_CTRL_CMD_ID = 0x1206,
    GIMBAL_REL_AID_CTRL_CMD_ID = 0x130A,
    GIMBAL_REL_FULL_CTRL_CMD_ID = 0x130B,
    UI_REL_MSG_CMD_ID = 0x1001,
}pc_cmd_id_t;

typedef  struct
{
  uint8_t SOF;
  uint8_t frame_length; //entire frame length
	uint8_t seq;
	uint8_t CRC8;
  //uint16_t cmd_id; //cmd_id is related to data package 
	
} pc_comm_frame_header_t;

typedef  struct
{
  uint8_t SOF;
  uint8_t frame_length; //entire frame length
	uint8_t seq;
	uint8_t CRC8;
  uint16_t cmd_id; //cmd_id included considered as header for embed send
	
} pc_comm_embed_send_header_t;

typedef enum
{
  PC_COMM_STEP_HEADER_SOF  = 0,
  PC_COMM_STEP_FRAME_LENGTH  = 1,
	PC_COMM_STEP_SEQ = 2,
	PC_COMM_STEP_CRC8 = 3,
  PC_COMM_STEP_CMDID_LOW = 4,
	PC_COMM_STEP_CMDID_HIGH = 5,
  PC_COMM_STEP_END_CRC16  = 6,
} pc_comm_unpack_step_e;

typedef struct
{
  pc_comm_frame_header_t *p_header;
  uint8_t       frame_len; //data_len; This is frame length of entire frame
	uint16_t			cmd_id;
	uint16_t			cmd_id_pc_comm_data_solve_debug; //For Debug
	
  uint8_t        protocol_packet[PC_PROTOCOL_FRAME_MAX_SIZE];
  pc_comm_unpack_step_e  unpack_step;
  uint8_t       index;
} pc_comm_unpack_data_t;

#pragma pack(pop)

/* -------------------------------- USART SEND -------------------------------- */
//this value is used for fifo - software fifo buffer
#define MINIPC_COMM_TX_FIFO_BUF_LENGTH 128 //1024

//This is the DMA buff length
#define MINIPC_COMM_UART_DMA_TX_BUF_LENGHT 128 //512

#define MINIPC_COMM_FRAME_MAX_SIZE 64 //send temp ram buffer -NOT 40 as before --- might cause outo of bound pointer

typedef struct
{
	//ring buffer 环形缓冲区模块
	fifo_s_t tx_fifo; //embed send ring buffer
	
	uint8_t status;	//msg send sts
	
	uint8_t* tx_dma_buf; //ptr to send dma
	uint16_t tx_dma_buf_size; //the size of the send dma array
	
	uint32_t debug_fifo_size;
	uint32_t debug_UartTxCount;
	
} pc_comm_embed_uart_send_data_t;

//协议层
typedef struct
{
	pc_comm_embed_send_header_t *p_header;
	uint8_t send_ram_buff[MINIPC_COMM_FRAME_MAX_SIZE];
	uint32_t index;
	
	//the # of time that send failed
	uint32_t relative_send_fail_cnts; //relative send counts, will use this to enforce dma tx poll
	
	
	uint32_t chassis_info_embed_send_TimeStamp; //current time stamp
	
	uint32_t gimbal_info_embed_send_TimeStamp;

}embed_send_protocol_t;

/* -------------------------------- USART SEND END-------------------------------- */

#endif
