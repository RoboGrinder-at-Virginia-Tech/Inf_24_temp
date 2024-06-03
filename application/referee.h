#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

// 串口协议附录 https://www.robomaster.com/zh-CN/products/components/referee

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
		RED_DART				= 8,
		RED_RADAR 			= 9,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
		BLUE_DART				= 108,
		BLUE_RADAR 			= 109,
} robot_id_t;

typedef enum
{
    OPERATOR_RED_HERO        = 0x0101,
    OPERATOR_RED_ENGINEER    = 0x0102,
    OPERATOR_RED_STANDARD_1  = 0x0103,
    OPERATOR_RED_STANDARD_2  = 0x0104,
    OPERATOR_RED_STANDARD_3  = 0x0105,
    OPERATOR_RED_AERIAL      = 0x0106,
    
		//
    OPERATOR_BLUE_HERO       = 0x0165,
    OPERATOR_BLUE_ENGINEER   = 0x0166,
    OPERATOR_BLUE_STANDARD_1 = 0x0167,
    OPERATOR_BLUE_STANDARD_2 = 0x0168,
    OPERATOR_BLUE_STANDARD_3 = 0x0169,
    OPERATOR_BLUE_AERIAL     = 0x016A,
    
} operator_robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;

typedef __packed struct // 0x0001
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}game_status_t;

typedef __packed struct // 0x0002
{
 uint8_t winner;
}game_result_t;

typedef __packed struct // 0x0003
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP; // sentry
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
}game_robot_HP_t; 

/* 0x0005 deleted */

typedef __packed struct // 0x0101
{
	uint32_t event_data;
}event_data_t; 


typedef __packed struct // 0x0102
{
	uint8_t reserved;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* 0x0103 deleted */

typedef __packed struct // 0x0104
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
}referee_warning_t; 

typedef __packed struct // 0x0105
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
}dart_info_t; 

typedef __packed struct // 0x0201
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
}robot_status_t; 

/*
下面这个是判断枪口热量用了的:
原先的:
typedef __packed struct //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
} ext_power_heat_data_t;
shooter_heat0 对应 shooter_id1_17mm_cooling_heat
shooter_heat1 对应 shooter_id2_17mm_cooling_heat
*/
typedef __packed struct // 0x0202
{
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t; 


typedef __packed struct // 0x0203
{
	float x;
	float y;
	float angle;
}robot_pos_t; 

typedef __packed struct // 0x0204
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
}buff_t;

typedef __packed struct // 0x0205
{
	uint8_t airforce_status;
	uint8_t time_remain;
}air_support_data_t;

typedef __packed struct // 0x0206
{
	uint8_t armor_id : 4;
	uint8_t HP_deduction_reason : 4;
}hurt_data_t;

typedef __packed struct // 0x0207
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
}shoot_data_t;

typedef __packed struct // 0x0208
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
}projectile_allowance_t;

typedef __packed struct // 0x0209
{
	uint32_t rfid_status;
}rfid_status_t; 

typedef __packed struct // 0x020A
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
}dart_client_cmd_t; 

typedef __packed struct // 0x020B
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float standard_5_x;
	float standard_5_y;
}ground_robot_position_t;

typedef __packed struct // 0x020C
{
	uint8_t mark_hero_progress;
	uint8_t mark_engineer_progress;
	uint8_t mark_standard_3_progress;
	uint8_t mark_standard_4_progress;
	uint8_t mark_standard_5_progress;
	uint8_t mark_sentry_progress;
}radar_mark_data_t;

typedef __packed struct // 0x020D
{
	uint32_t sentry_info;
} sentry_info_t;

typedef __packed struct // 0x020E
{
	uint8_t radar_info;
} radar_info_t; 

typedef __packed struct // 0x0301
{
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t user_data[112];
}robot_interaction_data_t; 

typedef __packed struct // 0x0303
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
}map_command_t;

typedef __packed struct // 0x0305
{
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
}map_robot_data_t; 

typedef __packed struct // 0x0307
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id; 
}map_data_t; 

typedef __packed struct // 0x0308
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void cpc_get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
extern uint16_t get_chassis_buffer_energy(void);

extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);

extern void get_shooter_id1_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1);
extern void get_shooter_id2_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1);
extern uint16_t get_chassis_power_limit(void);
extern uint16_t get_shooter_id1_17mm_speed_limit(void);
extern uint16_t get_shooter_id2_17mm_speed_limit(void);

extern uint16_t get_shooter_id1_17mm_cd_rate(void);

extern uint32_t get_last_robot_state_rx_timestamp(void);

extern uint8_t get_chassis_power_output_status(void);

extern uint8_t get_game_state_game_type(void);
extern uint8_t get_game_state_game_progress(void);
extern uint16_t get_game_state_stage_remain_time(void);
extern void get_current_and_max_hp(uint16_t *current_HP, uint16_t *maximum_HP);
extern game_robot_HP_t* get_game_robot_HP_ptr(void);

#endif
