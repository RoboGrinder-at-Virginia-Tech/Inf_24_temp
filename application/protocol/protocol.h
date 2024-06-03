#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATUS_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    //SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103, deleted
    REFEREE_WARNING_CMD_ID            = 0x0104,
		DART_INFO_CMD_ID									= 0x0105, // not used
    ROBOT_STATUS_CMD_ID               = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_CMD_ID                  			= 0x0204,
    AIR_SUPPORT_DATA_CMD_ID        		= 0x0205,
    HURT_DATA_CMD_ID                	= 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    PROJECTILE_ALLOWANCE_CMD_ID       = 0x0208,
	  RFID_STATUS_CMD_ID								= 0x0209, // not used
		DART_CLIENT_CMD_ID								= 0x020A, // not used
		GROUND_ROBOT_POSITION_CMD_ID			= 0x020B, // not used
		RADAR_MARK_DATA_CMD_ID						= 0x020C, // not used
		SENTRY_INFO_CMD_ID								= 0x020D, // not used
		RADAR_INFO_CMD_ID									= 0x020E, // not used
	
    ROBOT_INTERACTION_DATA_CMD_ID   	= 0x0301,
    /* TODO:
			For robot interaction data, the receive is handled in referee rx task,
	    the transmit is handled in UI task(or referee_interaction_task)
		*/
		
		// TODO: 图传链路 等, video communication route, handled in referee rx task by a different uart port
		
		// 0x0302 TODO: 图传链路, 自定义控制器
		
		MAP_COMMAND_CMD_ID								=	0x0303, // not used
		
		// 0x0304 TODO: 图传链路, 键鼠
		
		MAP_ROBOT_DATA_CMD_ID							=	0x0305, // not used
		
		CUSTOM_CLIENT_DATA_CMD_ID					= 0x0306, // N/A - reserved for 自定义控制器->选手端
		// TODO: 自定义控制器由给ref发信息的线程处理
		
		MAP_DATA_CMD_ID										= 0x0307, // not used
		
		CUSTOM_INFO_CMD_ID								= 0x0308, // not used
		
		
}referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
