#ifndef __SUPERCAP_COMM_H___
#define __SUPERCAP_COMM_H___

#include "struct_typedef.h"
#include "CAN_receive.h"

#define SUPERCAP_CAN hcan1

///*以下两个Texas不使用*/
//#define ICRA_superCap_max_power 65//40
//#define ICRA_superCap_fail_safe_power 65//40

//发送给超级电容数据数值限制
#define CMD_CHARGE_PWR_MAX 160
#define CMD_CHARGE_PWR_MIN 45

/*超级电容电容组数据*/
//2024第三代超级电容
#define CHARACTERISTIC_VOLTAGE_GEN3_CAP 26.5f
#define MIN_VOLTAGE_GEN3_CAP 0.0f
#define CAPACITY_GEN3_CAP 5.0f

//2023新超级电容
#define CHARACTERISTIC_VOLTAGE_23_CAP 26.5f//26.5f //特征电压是充电电路输出电压，对于C620必须低一些，否则会触发保护
#define MIN_VOLTAGE_23_CAP 13.0f
#define CAPACITY_23_CAP 5.0f //6.0f //电容容量，单位法拉

#define CHARACTERISTIC_VOLTAGE_WULIE_CAP 24.0f//26.5f //特征电压是充电电路输出电压，对于C620必须低一些，否则会触发保护
#define MIN_VOLTAGE_WULIE_CAP 13.0f
#define CAPACITY_WULIE_CAP 6.0f //电容容量，单位法拉

#define CHARACTERISTIC_VOLTAGE_ZIDA_CAP 24.0f
#define MIN_VOLTAGE_ZIDA_CAP 13.0f
#define CAPACITY_ZIDA_CAP 6.0f

typedef enum
{
	/*TypeC -> SuperCap 时 CAN报文 即指令的ID
	(1)TypeC -> SuperCapZida (2)TypeC -> gen2Cap
	*/
	 RMTypeC_Master_Command_ID = 0x4FF,
	
	// TypeC到gen3Cap_ID 的CAN报文
	RMTypeC_Master_Command_ID_for_gen3Cap = 0x108, //0x3FF,
	
	//SuperCap -> TypeC时 CAN报文 即反馈报文的ID:
	 ZiDaCap_ID = 0x500,
	
	//gen2Cap 易林超级电容 -> TypeC
	 gen2Cap_ID = 0x501,
	
	 //彭睿 第三代超级电容 -> TypeC
	 gen3Cap_ID = 0x109, //0x503
	
	//雾列的控制板
	//TypeC -> wulie Cap
	RMTypeC_Master_Command_ID_for_WuLie = 0x210,
	
	//wulie Cap -> TypeC
	WuLieCap_CAN_ID = 0x211,
}supercap_can_msg_id_e;

typedef enum
{
	superCap_offline,
	superCap_online
}superCap_connection_status_e;//这两个结构体 不同的超级电容共用

typedef enum
{
	SuperCap_dataIsError,
	SuperCap_dataIsCorrect
}superCap_dataError_status_e;//这两个结构体 不同的超级电容共用

typedef struct
{
	//发给Cap的数据
	uint8_t max_charge_pwr_command;
	uint8_t fail_safe_charge_pwr_command;
	
	//接收到的 经过转换后的数据
	float EBPct_fromCap; //这个是相对于0J 0%-100%
	float VBKelvin_fromCap;//
	superCap_connection_status_e status;
	
//	int32_t EBank;
	fp32 EBank;
	
	superCap_dataError_status_e data_EBPct_status;
	union superCap_msg_u
	{
		uint16_t msg_u;
		/* 小端模式下: msg_u_EBPct高字节位对应的是a[1]; 低字节位对应的是a[0] */
		uint8_t array[2];
	}msg_u_EBPct, msg_u_VBKelvin;
	
	fp32 relative_EBpct; // 相对于最低电压的百分比
	
}zidaCap_info_t; // ZiDa 超级电容

/*12-27-2022新增 易林 超级电容
superCap23_info_t;
*/
typedef struct
{
	//发给cap的数据
	uint8_t max_charge_pwr_from_ref; //中间变量用于调试
	uint8_t fail_safe_charge_pwr_ref; //中间变量用于调试
	
	uint8_t charge_pwr_command;
	uint8_t fail_safe_charge_pwr_command;
	
	//接收到的 经过转换后的数据
	fp32 PowerData[4];
	fp32 Vin_f; //输入电压**
	fp32 Vbank_f; //电容电压**
	fp32 Ibank_f; //电容电流
	fp32 Vchassis_f; //给底盘电压** -6-14不用了 
	fp32 Cap_fb_set_power; //-6-14超级电容返回的当前充电功率
	fp32 Ichassis_f; //底盘电流**
	fp32 Pbank_f; //
	fp32 Energy; //
	uint8_t PMOS_error_flag;
	
	//其它FSM 和 变量
	superCap_connection_status_e status;
	//这个是相对于0J 0%-100%
	fp32 EBPct;
	fp32 EBank;
	
	fp32 relative_EBpct; // 相对于最低电压的百分比
	
}gen2Cap_info_t;

typedef enum 
{
	CAP_NORMAL = 0,    //正常
	CAP_VOLTAGE_SHUTDOWN = 1, //输入电压异常
	CAP_TEMP_SHUTDOWN = 2,       //过热
	CAP_CURRENT_SHUTDOWN = 3, //过流
}cap_Pflag_e;

typedef struct
{
	//发给cap的数据
	uint8_t max_charge_pwr_from_ref; //中间变量用于调试
	uint8_t fail_safe_charge_pwr_ref; //中间变量用于调试
	
	uint8_t charge_pwr_command; //P-target
	uint8_t fail_safe_charge_pwr_command; //P-failsafe
	uint8_t dcdc_enable;
	uint8_t dcdc_mode;
	uint16_t buffer_energy; // 从裁判系统获取的buffer energy
	
	//接收到的 经过转换后的数据
	fp32 PowerData[3];
	fp32 Vbank_f;
	fp32 Pchassis_f;
	fp32 Pmax_f;
	cap_Pflag_e Pflag;
	
	//其它FSM 和 变量
	superCap_connection_status_e status;
	//这个是相对于0J 0%-100%
	fp32 EBPct;
	fp32 EBank;
	
	fp32 relative_EBpct; // 相对于最低电压的百分比
	
	//裁判系统
	uint8_t power_management_chassis_output; // 0为无输出, 1为24v
	
}gen3Cap_info_t; // PR 第三代超级电容

typedef struct
{
	//发给cap的数据
	uint16_t max_charge_pwr_from_ref;
	uint16_t charge_pwr_command;
	
	//接受到的 经过转换后的数据
	fp32 PowerData[4];
	fp32 input_voltage;//输入电压
  fp32 cap_voltage;//电容电压
	fp32 input_current; //输入电流
	fp32 set_power;//设定功率
	
	//其它FSM 和 变量
	superCap_connection_status_e status;
	//这个是相对于0J 0%-100%
	fp32 EBPct;
	fp32 EBank;
	
	fp32 relative_EBpct; // 相对于最低电压的百分比

}wulieCap_info_t;

extern zidaCap_info_t zidaCap_info;

extern fp32 cal_capE_relative_pct(fp32 curr_vol, fp32 min_vol, fp32 max_vol);

extern void CAN_command_zidaCap(uint8_t max_pwr, uint8_t fail_safe_pwr);
extern void CAN_command_gen2Cap(uint8_t max_pwr, uint8_t fail_safe_pwr);
extern void CAN_command_gen3Cap(uint8_t max_pwr, uint8_t fail_safe_pwr, uint8_t dcdc_enable, uint8_t dcdc_mode, uint16_t buffer_energy);
extern void CAN_command_wulieCap(uint16_t temPower);
extern void zidaCap_offline_proc(void);
extern bool_t zidaCap_is_data_error_proc(void);
extern void zidaCap_solve_data_error_proc(void);
extern void superCap_comm_bothway_init(void);
extern void superCap_control_loop(void);

extern void wulieCap_offline_proc(void);
extern bool_t wulieCap_is_data_error_proc(void);

extern bool_t current_superCap_is_offline(void);
extern bool_t all_superCap_is_error(void);

extern void gen2Cap_offline_proc(void);
extern bool_t gen2Cap_is_data_error_proc(void);

extern void gen3Cap_offline_proc(void);
extern bool_t gen3Cap_is_data_error_proc(void);

extern void cpc_get_superCap_vol_and_energy(fp32* cap_voltage, fp32* EBank);
extern uint16_t cpc_get_superCap_charge_pwr(void);
extern fp32 cpc_get_gen3Cap_Pmax(void);
extern cap_Pflag_e get_gen3Cap_P_Flag(void);

extern fp32 cer_get_current_cap_boost_mode_pct_threshold(void);
extern fp32 cer_get_current_cap_relative_pct(void);

extern fp32 ui_get_current_cap_voltage(void);
extern fp32 simple_get_current_cap_pct(void);

extern fp32 ui_get_current_cap_relative_pct(void);
extern supercap_can_msg_id_e get_current_superCap(void);

#endif /*__SUPERCAP_COMM_H___*/
