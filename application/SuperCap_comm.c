//3-10-2022增加Cap和C板的通信 Can 1 经过滑环

#include "SuperCap_comm.h"
#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "referee.h"
#include "user_lib.h"
#include "detect_task.h"
#include "chassis_power_control.h"
#include "arm_math.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void superCap_offline_proc(void);

//static CAN_TxHeaderTypeDef  superCap_tx_message;
static uint8_t              superCap_can_send_data[8];
static uint8_t              wulieCap_can_send_data[8];
static uint8_t              sCap23_can_send_data[8];

superCap_info_t superCap_info1;
superCap_info_t superCap_info2;
superCap_info_t superCap_info3;//<----越界指针边界 大致位置
superCap_info_t superCap_info4;
superCap_info_t superCap_info;//用的这一个 3 4和这个是正常的
wulieCap_info_t wulie_Cap_info;//雾列超级电容控制板的结构体
sCap23_info_t sCap23_info; //新的超级电容控制板

CAN_TxHeaderTypeDef  superCap_tx_message;
CAN_TxHeaderTypeDef  wulie_Cap_tx_message;
CAN_TxHeaderTypeDef  sCap23_tx_message;

supercap_can_msg_id_e current_superCap; //表明当前使用的是哪一个超级电容

uint8_t debug_max_pwr;
uint8_t debug_fail_safe_pwr;
//uint8_t debug_a=0;
//uint8_t debug_b;
//uint8_t debug_c;

uint32_t any_Cap_can_msg_send_TimeStamp = 0;
const uint16_t any_Cap_can_msg_send_sendFreq = 100;

void superCap_comm_bothway_init()
{
	/*
	初始化以下:
	1 CAN发送数据
	2 CAN接收数据
	*/
	//1初始化发送
	superCap_info.max_charge_pwr_command = 0;
	superCap_info.fail_safe_charge_pwr_command = 0;
	
	//2初始化接收
	superCap_info.EBPct_fromCap = 0.0f;
	superCap_info.VBKelvin_fromCap = 0.0f;
	superCap_info.status = superCap_offline;
	//superCap_info.data_EBPct_status = SuperCap_dataIsError;
	superCap_info.msg_u_EBPct.array[0] = 0;
	superCap_info.msg_u_EBPct.array[1] = 0;
	superCap_info.a = 0;
	superCap_info.b = 0;
	superCap_info.c = 0;
	
	current_superCap = sCap23_ID; //SuperCap_ID;//SuperCap_ID wulie_Cap_CAN_ID
}

uint16_t temp_pwr_command=0;
void superCap_control_loop()
{
	//发送任务计时, 时间到了开始一次发送
	if(xTaskGetTickCount() - any_Cap_can_msg_send_sendFreq > any_Cap_can_msg_send_TimeStamp)
	{
		any_Cap_can_msg_send_TimeStamp = xTaskGetTickCount(); //更新时间戳 
			
		if(current_superCap == SuperCap_ID)
		{//紫达控制板
			//Texas 调试
			temp_pwr_command = get_chassis_power_limit();
			
//			superCap_info.max_charge_pwr_command = get_chassis_power_limit() - 2.5f;
//			//这时fail safe该不该继续按照当前允许的最大功率来SZL 5-16-2022 还是应该按等级信息来算?
//			superCap_info.fail_safe_charge_pwr_command = get_chassis_power_limit() - 2.5f;
			
			//用temp_pwr_command来判断一个不合理数值
			if(temp_pwr_command > 110)
			{
//				superCap_info.max_charge_pwr_command = 60 - 3;
//				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
					temp_pwr_command = INITIAL_STATE_CHASSIS_POWER_LIM;
			}
			
			/*功率自适应 分档 标定
			max_charge_pwr_command; fail_safe_charge_pwr_command; 是 uint8_t
			*/
			if(temp_pwr_command == 40)
			{
				superCap_info.max_charge_pwr_command = 40 - 3;//2.5f;// 40 档 offset 2.5f
				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 60)
			{
				superCap_info.max_charge_pwr_command = 60 - 3;//2.5f;// 60 档 offset 2.5f
				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 80 )
			{
				superCap_info.max_charge_pwr_command = 80 - 5;
				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 100)
			{
				superCap_info.max_charge_pwr_command = 80;
				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 45)
			{
				superCap_info.max_charge_pwr_command = 45 - 3;//2.5f;// 40 档 offset 2.5f
				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 50)
			{
				superCap_info.max_charge_pwr_command = 50 - 3;//2.5f;// 40 档 offset 2.5f
				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 55)
			{
				superCap_info.max_charge_pwr_command = 55 - 3;//2.5f;// 40 档 offset 2.5f
				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
			}
			else
			{
				superCap_info.max_charge_pwr_command = 40 - 3;//2.5f;// 40 档 offset 2.5f
				superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
			}
			
//			superCap_info.max_charge_pwr_command = 70.0f;
//			superCap_info.fail_safe_charge_pwr_command = 40.0f;
				
//			if(superCap_info.max_charge_pwr_command >= 101.0f)
//			{
//				superCap_info.max_charge_pwr_command = 40;
//			}
//				
//			if(superCap_info.fail_safe_charge_pwr_command >= 101.0f)
//			{
//				superCap_info.fail_safe_charge_pwr_command = 40;
//			}
			
			//TODO: remove debug 并添加限制幅度
			
			//For Debug Only------------------------------------------------------------------------------------------------------------------------------------
			superCap_info.max_charge_pwr_command = 10;//2.5f;// 40 档 offset 2.5f
			superCap_info.fail_safe_charge_pwr_command = superCap_info.max_charge_pwr_command;
				
			CAN_command_superCap(superCap_info.max_charge_pwr_command, superCap_info.fail_safe_charge_pwr_command);	
		}
		else if(current_superCap == sCap23_ID)
		{//sCap23易林超级电容控制板
			//计算max_charge_pwr_from_ref
			sCap23_info.max_charge_pwr_from_ref = get_chassis_power_limit() - 0.0f; //2.5f
			
			if(sCap23_info.max_charge_pwr_from_ref > CMD_CHARGE_PWR_MAX)//101.0f)
			{
				sCap23_info.max_charge_pwr_from_ref = 40;
			}
			
//			//Only for Debug
//			sCap23_info.max_charge_pwr_from_ref = 41; //66;
			//--------------------------------------------
			
			//计算fail_safe_charge_pwr_ref 修改成用ifelse标定等级标定fail safe, 这个的目的是 比赛中可能有临时的底盘充电增益, fail safe表示当前的一个安全数值
			sCap23_info.fail_safe_charge_pwr_ref = sCap23_info.max_charge_pwr_from_ref; //40; // 60; // = sCap23_info.max_charge_pwr_from_ref;
		
			sCap23_info.charge_pwr_command = sCap23_info.max_charge_pwr_from_ref;
			sCap23_info.fail_safe_charge_pwr_command = sCap23_info.fail_safe_charge_pwr_ref;
			
			//限制幅度
			sCap23_info.charge_pwr_command = uint8_constrain(sCap23_info.charge_pwr_command, CMD_CHARGE_PWR_MIN, CMD_CHARGE_PWR_MAX);
			sCap23_info.fail_safe_charge_pwr_command = uint8_constrain(sCap23_info.fail_safe_charge_pwr_command, CMD_CHARGE_PWR_MIN, CMD_CHARGE_PWR_MAX);
			
			CAN_command_sCap23(sCap23_info.charge_pwr_command, sCap23_info.fail_safe_charge_pwr_command);
		}
		else //if(current_superCap == wulie_Cap_CAN_ID)
		{//雾列控制板
			wulie_Cap_info.max_charge_pwr_from_ref = get_chassis_power_limit() - 2.5f;
				
			if(wulie_Cap_info.max_charge_pwr_from_ref > CMD_CHARGE_PWR_MAX)//101.0f)
			{
				wulie_Cap_info.max_charge_pwr_from_ref = 40;
			}
			
			//TODO: remove debug 并添加限制幅度
			
			//Only for Debug
			wulie_Cap_info.max_charge_pwr_from_ref = 30;
				
			wulie_Cap_info.charge_pwr_command = wulie_Cap_info.max_charge_pwr_from_ref * 100.f;
			CAN_command_wulie_Cap(wulie_Cap_info.charge_pwr_command);
		}
	}
}
//	//ICRA only
//	superCap_info.max_charge_pwr_command = ICRA_superCap_max_power; //=65w
//	superCap_info.fail_safe_charge_pwr_command = ICRA_superCap_fail_safe_power; //=65w
//	CAN_command_superCap(superCap_info.max_charge_pwr_command, superCap_info.fail_safe_charge_pwr_command);	

/*
根据目前使用的超级电容 返回电容组电压和剩余能量
这里要做的就是返回合理的 传感器数据 不需要在这里考虑掉线
*/
void get_superCap_vol_and_energy(fp32* cap_voltage, fp32* EBank) //仅功率控制使用
{
	fp32 temp_EBank=0, temp_cap_voltage=0;
	if(current_superCap == SuperCap_ID)
	{
		temp_EBank = superCap_info.EBank;
		temp_cap_voltage = superCap_info.VBKelvin_fromCap;
		
		temp_EBank = fp32_constrain(temp_EBank, 0.0f, 2106.75f);//确保数据的正确和合理性
		temp_cap_voltage = fp32_constrain(temp_cap_voltage, 0.0f, 26.5f);
		
		*EBank = temp_EBank;
		*cap_voltage = temp_cap_voltage;
		return;
	}
	else if(current_superCap == sCap23_ID)
	{
		temp_EBank = sCap23_info.EBank;
		temp_cap_voltage = sCap23_info.Vbank_f;
		
		temp_EBank = fp32_constrain(temp_EBank, 0.0f, 2106.75f);//确保数据的正确和合理性
		temp_cap_voltage = fp32_constrain(temp_cap_voltage, 0.0f, 28.5f);
		
		*EBank = temp_EBank;
		*cap_voltage = temp_cap_voltage;
		return;
	}
	else
	{
		temp_EBank = wulie_Cap_info.EBank;
		temp_cap_voltage = wulie_Cap_info.cap_voltage;
		
		temp_EBank = fp32_constrain(temp_EBank, 0.0f, 2106.75f);//确保数据的正确和合理性
		temp_cap_voltage = fp32_constrain(temp_cap_voltage, 0.0f, 26.5f);
		
		*EBank = temp_EBank;
		*cap_voltage = temp_cap_voltage;
		return;
	}
}

/*
返回超级电容充电功率
*/
uint16_t get_superCap_charge_pwr() //仅功率控制使用
{
	fp32 temp_charge_pwr=0;
	if(current_superCap == SuperCap_ID)
	{
		temp_charge_pwr = superCap_info.max_charge_pwr_command;
		temp_charge_pwr = fp32_constrain(temp_charge_pwr, 0.0f, (fp32)CMD_CHARGE_PWR_MAX);//确保数据的正确和合理性
		
		return (uint16_t)temp_charge_pwr;
	}
	else if(current_superCap == sCap23_ID)
	{
		temp_charge_pwr = sCap23_info.charge_pwr_command;
		temp_charge_pwr = fp32_constrain(temp_charge_pwr, 0.0f, (fp32)CMD_CHARGE_PWR_MAX);//确保数据的正确和合理性
		
		return (uint16_t)temp_charge_pwr;
	}
	else
	{
		temp_charge_pwr = wulie_Cap_info.max_charge_pwr_from_ref;
		temp_charge_pwr = fp32_constrain(temp_charge_pwr, 0.0f, (fp32)CMD_CHARGE_PWR_MAX);//确保数据的正确和合理性
		
		return (uint16_t)temp_charge_pwr;
	}
}

/*下面两个函数; 0->normal/online; 1->error/offline*/
bool_t current_superCap_is_offline()
{
	if(current_superCap == sCap23_ID) //SuperCap_ID
	{
		return toe_is_error(SCAP_23_TOR); //SUPERCAP_TOE
	}
	else if(current_superCap == wulie_Cap_CAN_ID) //wulie_Cap_CAN_ID
	{
		return toe_is_error(WULIE_CAP_TOE);
	}
	else
	{
		return toe_is_error(SUPERCAP_TOE); //sCap23_ID
	}
}

bool_t all_superCap_is_offline()
{
	return toe_is_error(SUPERCAP_TOE) && toe_is_error(WULIE_CAP_TOE) && toe_is_error(SCAP_23_TOR);
}

/*
SZL 3-10-2022 下发到SuperCap的数据
SZL 12-27-2022 新增YiLin超级电容
*/
void CAN_command_sCap23(uint8_t max_pwr, uint8_t fail_safe_pwr)
{
		uint32_t send_mail_box;
    sCap23_tx_message.StdId = RMTypeC_Master_Command_ID;
    sCap23_tx_message.IDE = CAN_ID_STD;
    sCap23_tx_message.RTR = CAN_RTR_DATA;
    sCap23_tx_message.DLC = 0x08;
    sCap23_can_send_data[0] = max_pwr;
    sCap23_can_send_data[1] = fail_safe_pwr;
    sCap23_can_send_data[2] = 0;
    sCap23_can_send_data[3] = 0;
    sCap23_can_send_data[4] = 0; 
    sCap23_can_send_data[5] = 0; 
    sCap23_can_send_data[6] = 0; 
    sCap23_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&SCAP23_CAN, &sCap23_tx_message, sCap23_can_send_data, &send_mail_box);
}

void CAN_command_superCap(uint8_t max_pwr, uint8_t fail_safe_pwr)
{
		uint32_t send_mail_box;
    superCap_tx_message.StdId = RMTypeC_Master_Command_ID;
    superCap_tx_message.IDE = CAN_ID_STD;
    superCap_tx_message.RTR = CAN_RTR_DATA;
    superCap_tx_message.DLC = 0x08;
    superCap_can_send_data[0] = max_pwr;
    superCap_can_send_data[1] = fail_safe_pwr;
    superCap_can_send_data[2] = 0;
    superCap_can_send_data[3] = 0;
    superCap_can_send_data[4] = 0; 
    superCap_can_send_data[5] = 0; 
    superCap_can_send_data[6] = 0; 
    superCap_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&SUPERCAP_CAN, &superCap_tx_message, superCap_can_send_data, &send_mail_box);
}

void CAN_command_wulie_Cap(uint16_t temPower)
{
		uint32_t send_mail_box;
    wulie_Cap_tx_message.StdId = RMTypeC_Master_Command_ID_for_WuLie;
    wulie_Cap_tx_message.IDE = CAN_ID_STD;
    wulie_Cap_tx_message.RTR = CAN_RTR_DATA;
    wulie_Cap_tx_message.DLC = 0x08;
    wulieCap_can_send_data[0] = temPower >> 8;
    wulieCap_can_send_data[1] = temPower;
    wulieCap_can_send_data[2] = 0;
    wulieCap_can_send_data[3] = 0;
    wulieCap_can_send_data[4] = 0; 
    wulieCap_can_send_data[5] = 0; 
    wulieCap_can_send_data[6] = 0; 
    wulieCap_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&hcan1, &wulie_Cap_tx_message, wulieCap_can_send_data, &send_mail_box);
}

//返回数据相关
void superCap_offline_proc()
{
		superCap_info.status = superCap_offline;
		
	
}

bool_t superCap_is_data_error_proc()
{
		superCap_info.status = superCap_online;
	
		/*ICRA
			只判断EBPct_fromCap，因为只用这个, 这个是0% 这种 并不是0.0 - 100.0
			注意EBPct是可能超过100%的所以暂时把检查数据是否出错改成这个样子 -100.0 ~ 200.0
			下面的superCap_solve_data_error_proc也有更改
			修改意见: superCap_info 这种原始数据 最好不要动它 如果需要识别不和里数据的地方, 新建一个变量 然后赋值给那个变量
	  */
		if(superCap_info.EBPct_fromCap < -100.0f || superCap_info.EBPct_fromCap > 200.0f)
		{
			superCap_info.data_EBPct_status = SuperCap_dataIsError;
			return 0;
			
		}
		else
		{
			superCap_info.data_EBPct_status = SuperCap_dataIsCorrect;
			return 0;
		}
}

void superCap_solve_data_error_proc()
{
		//因为其数值可能超过100 所以暂时把这个功能屏蔽掉 ICRA
//		if(superCap_info.data_EBPct_status == SuperCap_dataIsError)
//		{
//			if(superCap_info.EBPct_fromCap < 0.0f)
//				superCap_info.EBPct_fromCap = 0.0f;
//			if(superCap_info.EBPct_fromCap > 100.0f)
//				superCap_info.EBPct_fromCap = 100.0f;
//		}
		return;
}

//以下为易林超级电容相关
void sCap23_offline_proc()
{
		sCap23_info.status = superCap_offline;
}

bool_t sCap23_is_data_error_proc()
{
		sCap23_info.status = superCap_online;
		//永远 return 0;
		return 0;
//		//ICRA
//		//只判断EBPct_fromCap，因为只用这个, 这个是0% 这种 并不是0.0 - 100.0
//		//注意EBPct是可能超过100%的所以暂时把检查数据是否出错改成这个样子 -100.0 ~ 200.0
//		//下面的superCap_solve_data_error_proc也有更改
//		if(superCap_info.EBPct_fromCap < -100.0f || superCap_info.EBPct_fromCap > 200.0f)
//		{
//			superCap_info.data_EBPct_status = SuperCap_dataIsError;
//			return 1;
//			
//		}
//		else
//		{
//			superCap_info.data_EBPct_status = SuperCap_dataIsCorrect;
//			return 0;
//		}
}


//以下为雾列相关的
void wulie_Cap_offline_proc()
{
		wulie_Cap_info.status = superCap_offline;
}

bool_t wulie_Cap_is_data_error_proc()
{
		wulie_Cap_info.status = superCap_online;
		//永远 return 0;
		return 0;
//		//ICRA
//		//只判断EBPct_fromCap，因为只用这个, 这个是0% 这种 并不是0.0 - 100.0
//		//注意EBPct是可能超过100%的所以暂时把检查数据是否出错改成这个样子 -100.0 ~ 200.0
//		//下面的superCap_solve_data_error_proc也有更改
//		if(superCap_info.EBPct_fromCap < -100.0f || superCap_info.EBPct_fromCap > 200.0f)
//		{
//			superCap_info.data_EBPct_status = SuperCap_dataIsError;
//			return 1;
//			
//		}
//		else
//		{
//			superCap_info.data_EBPct_status = SuperCap_dataIsCorrect;
//			return 0;
//		}
}

//API for UI and other
fp32 get_current_cap_voltage()
{
	//即插即用的超级电容控制板 判断
	 if(current_superCap == SuperCap_ID)
	 {
		 if(toe_is_error(SUPERCAP_TOE))
		 {
				return 0.0f;
		 }
		 else
		 {
			 //ui_info.cap_pct = superCap_info.EBPct_fromCap;
			 return superCap_info.VBKelvin_fromCap;
		 }
	 }
	 else if(current_superCap == sCap23_ID)
	 {
		 if(toe_is_error(SCAP_23_TOR))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 //ui_info.cap_pct = sCap23_info.EBPct;
		   return sCap23_info.Vbank_f;
		 }
	 }
	 else
	 {
		 if(toe_is_error(WULIE_CAP_TOE))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 //ui_info.cap_pct = wulie_Cap_info.EBPct;
		   return wulie_Cap_info.cap_voltage;
		 }
	 }
}

fp32 get_current_cap_pct()
{
	//即插即用的超级电容控制板 判断
	 if(current_superCap == SuperCap_ID)
	 {
		 if(toe_is_error(SUPERCAP_TOE))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 return superCap_info.EBPct_fromCap;
			 //ui_info.cap_volt = superCap_info.VBKelvin_fromCap;
		 }
	 }
	 else if(current_superCap == sCap23_ID)
	 {
		 if(toe_is_error(SCAP_23_TOR))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 return sCap23_info.EBPct;
		   //ui_info.cap_volt = sCap23_info.Vbank_f;
		 }
	 }
	 else
	 {
		 if(toe_is_error(WULIE_CAP_TOE))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 return wulie_Cap_info.EBPct;
		   //ui_info.cap_volt = wulie_Cap_info.cap_voltage;
		 }
	 }
}

///* 获得 当前在线的超级电容, 电压百分比, 13v时为0%
//*/
//fp32 get_current_cap_voltage_pct()
//{
//}
extern RC_ctrl_t rc_ctrl;
/* 计算 获得 当前在线的超级电容, 能量(焦耳)百分比, 13v时为0%
*/
//call时 确保 参数的正确性
fp32 cal_capE_relative_pct(fp32 curr_vol, fp32 min_vol, fp32 max_vol)
{
	return fp32_constrain( ( (curr_vol - min_vol) * (curr_vol - min_vol) ) / ( (max_vol - min_vol) * (max_vol - min_vol) ), 0.0f, 1.0f);
}

fp32 get_current_capE_relative_pct()
{
//		return fp32_constrain( fabs((fp32) rc_ctrl.rc.ch[3]) / 660.0f, 0.0f, 1.0f);
		//即插即用的超级电容控制板 判断
		if(current_superCap == SuperCap_ID)
		{
			 if(toe_is_error(SUPERCAP_TOE))
			 {
				 return 0.0f;
			 }
			 else
			 {
				 return superCap_info.relative_EBpct;
			 }
		 }
		 else if(current_superCap == sCap23_ID)
		 {
			 if(toe_is_error(SCAP_23_TOR))
			 {
				 return 0.0f;
			 }
			 else
			 {
				 return sCap23_info.relative_EBpct;
			 }
		 }
		 else
		 {
			 if(toe_is_error(WULIE_CAP_TOE))
			 {
				 return 0.0f;
			 }
			 else
			 {
				 return wulie_Cap_info.relative_EBpct;
			 }
		 }
}

supercap_can_msg_id_e get_current_superCap()
{
		return current_superCap;
}
