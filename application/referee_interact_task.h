#ifndef __REFEREE_INTERACT_TASK__
#define __REFEREE_INTERACT_TASK__

#include "stm32f4xx.h"
#include "user_lib.h"
#include "struct_typedef.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "shoot.h"
#include "miniPC_msg.h"
#include "remote_control.h"

/*
���������, UI��Referee_Interactive_infoģ����ʹ��Ȩ�Ϳ���Ȩ������ cmd task, UI����ָ����cmd task������Referee_Interactive_info_t,
������֤�˶Ա仯��Ӧ��ʵʱ. ����cmd task�л����(����)����chassis_flag��ֱ��ˢ�¸�UI, �򷢲�chassis_mode, Ȼ����UIģ�����ж�Mode_Change_Check
*/

#pragma pack(1)

typedef struct
{
	uint32_t chassis_mode_flag : 1;
	uint32_t chassis_energy_mode_flag : 1;
	uint32_t shoot_mode_flag : 1;
	uint32_t auto_aim_mode_flag : 1;
	uint32_t cv_gimbal_sts_flag : 1;
	
	// �ڲ����
	uint32_t chassis_error_flag : 1;
	uint32_t gimbal_error_flag : 1;
	uint32_t shoot_error_flag : 1;
	uint32_t current_superCap_error_flag : 1;
} Referee_Interactive_Flag_t;

// �˽ṹ�����UI����������˳���ͨ�ŵ���Ҫ�������ǲ���ϵͳ����
typedef struct
{
	const RC_ctrl_t *rc_ctrl_ptr;
	
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	
	// ΪUI�����Լ�������������
	auto_aim_mode_e cv_gimbal_sts; // cv����ģʽ
	auto_aim_mode_e last_cv_gimbal_sts; // cv����ģʽ
	
	bool_t chassis_error;
	bool_t last_chassis_error;
	
	bool_t gimbal_error;
	bool_t last_gimbal_error;
	
	bool_t shoot_error;
	bool_t last_shoot_error;
	
	bool_t current_superCap_error;
	bool_t last_current_superCap_error;

} Referee_Interactive_info_t;

#pragma pack()

extern void referee_interact_task(void const *pvParameters);
extern void set_interactive_flag_chassis_mode_flag(uint8_t set_val);
extern void set_interactive_flag_chassis_energy_mode_flag(uint8_t set_val);
extern void set_interactive_flag_shoot_mode_flag(uint8_t set_val);
extern void set_interactive_flag_auto_aim_mode_flag(uint8_t set_val);
extern void set_interactive_flag_cv_gimbal_sts_flag(uint8_t set_val);

#endif
