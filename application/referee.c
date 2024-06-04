#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "cmsis_os.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t game_state;
game_result_t game_result;
game_robot_HP_t game_robot_HP;

event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action;
referee_warning_t referee_warning;


robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t robot_pos;
buff_t robot_buff;
air_support_data_t air_support_data;
hurt_data_t hurt_data;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
robot_interaction_data_t robot_interaction_data;


uint32_t last_robot_state_rx_timestamp; //上一次收到robot_state信息的时间戳

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));


    memset(&field_event, 0, sizeof(event_data_t));
    memset(&supply_projectile_action, 0, sizeof(ext_supply_projectile_action_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));


    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&robot_buff, 0, sizeof(buff_t));
    memset(&air_support_data, 0, sizeof(air_support_data_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));


    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));



}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATUS_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
        }
        break;

        case ROBOT_STATUS_CMD_ID:
        {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
						last_robot_state_rx_timestamp = xTaskGetTickCount();
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
        }
        break;
        case BUFF_CMD_ID:
        {
            memcpy(&robot_buff, frame + index, sizeof(buff_t));
        }
        break;
        case AIR_SUPPORT_DATA_CMD_ID:
        {
            memcpy(&air_support_data, frame + index, sizeof(air_support_data_t));
        }
        break;
        case HURT_DATA_CMD_ID:
        {
            memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
        }
        break;
        case PROJECTILE_ALLOWANCE_CMD_ID:
        {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
        }
        break;
        case ROBOT_INTERACTION_DATA_CMD_ID:
        {
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

void cpc_get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = (fp32) power_heat_data.chassis_power;
    *buffer = (fp32) power_heat_data.buffer_energy;

}

uint16_t get_chassis_buffer_energy(void)
{
	return power_heat_data.buffer_energy;
}

uint8_t get_robot_id(void)
{
    return robot_status.robot_id;
}

uint8_t get_robot_level(void)
{
		return robot_status.robot_level;
}

/*
SZL 5-15-2022更改
*/
/*
heat0_limit; heat0指的id1的17mm shooter
以此类推
*/
void get_shooter_id1_17mm_heat_limit_and_heat(uint16_t *heat0_limit, uint16_t *heat0)
{
	if(heat0_limit == NULL || heat0 == NULL)
	{
		return;
	}
    
	*heat0_limit = robot_status.shooter_barrel_heat_limit;
  *heat0 = power_heat_data.shooter_17mm_1_barrel_heat;
}

void get_shooter_id2_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1)
{
	if(heat1_limit == NULL || heat1 == NULL)
	{
		return;
	}
  
	*heat1_limit = robot_status.shooter_barrel_heat_limit;
  *heat1 = power_heat_data.shooter_17mm_2_barrel_heat;
}

uint16_t get_chassis_power_limit(void)
{
		return robot_status.chassis_power_limit;
}

uint16_t get_shooter_id1_17mm_speed_limit(void)
{
		return 30; //robot_state.shooter_id1_17mm_speed_limit;
}

uint16_t get_shooter_id2_17mm_speed_limit(void)
{
		return 30; //robot_state.shooter_id2_17mm_speed_limit;
}

uint16_t get_shooter_id1_17mm_cd_rate(void)
{
		return robot_status.shooter_barrel_cooling_value;
}

uint32_t get_last_robot_state_rx_timestamp(void)
{
	return last_robot_state_rx_timestamp;
}

uint8_t get_chassis_power_output_status(void)
{
    return robot_status.power_management_chassis_output; //0为无输出, 1为24v
}

uint8_t get_game_state_game_type(void)
{
	return game_state.game_type;
}

uint8_t get_game_state_game_progress(void)
{
	return game_state.game_progress;
}

uint16_t get_game_state_stage_remain_time(void)
{
	return game_state.stage_remain_time;
}

void get_current_and_max_hp(uint16_t *current_HP, uint16_t *maximum_HP)
{
	if(current_HP == NULL || maximum_HP == NULL)
	{
		return;
	}
	
	*current_HP = robot_status.current_HP;
	*maximum_HP = robot_status.maximum_HP;
}

game_robot_HP_t* get_game_robot_HP_ptr(void)
{
	return &game_robot_HP;
}
