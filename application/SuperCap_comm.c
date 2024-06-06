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

void zidaCap_offline_proc(void);

//static CAN_TxHeaderTypeDef  superCap_tx_message;
static uint8_t              zidaCap_can_send_data[8];
static uint8_t              wulieCap_can_send_data[8];
static uint8_t              gen2Cap_can_send_data[8];
static uint8_t              gen3Cap_can_send_data[8];

zidaCap_info_t zidaCap_info;// ZiDa 超级电容
wulieCap_info_t wulieCap_info;//  雾列超级电容控制板的结构体
gen2Cap_info_t gen2Cap_info; // PR YiLin 2023 Seattle 超级电容控制板
gen3Cap_info_t gen3Cap_info; // 第三代超级电容

CAN_TxHeaderTypeDef  zidaCap_tx_message;
CAN_TxHeaderTypeDef  wulieCap_tx_message;
CAN_TxHeaderTypeDef  gen2Cap_tx_message;
CAN_TxHeaderTypeDef  gen3Cap_tx_message;

supercap_can_msg_id_e current_superCap; // 表明当前使用的是哪一个超级电容

uint32_t any_Cap_can_msg_send_TimeStamp = 0;
const uint16_t any_Cap_can_msg_send_sendPeriod = 50;

/*  功能 计算 辅助函数 */

/* 计算 获得 当前在线的超级电容, 能量(焦耳)百分比, 13v时为0%
且确保 参数的正确性 */
fp32 cal_capE_relative_pct(fp32 curr_vol, fp32 min_vol, fp32 max_vol)
{
	return fp32_constrain( ( (curr_vol - min_vol) * (curr_vol - min_vol) ) / ( (max_vol - min_vol) * (max_vol - min_vol) ), 0.0f, 1.0f);
}

void superCap_comm_bothway_init()
{
	/*
	初始化以下:
	1 CAN发送数据
	2 CAN接收数据
	*/
	//1初始化发送
	zidaCap_info.max_charge_pwr_command = 0;
	zidaCap_info.fail_safe_charge_pwr_command = 0;
	
	//2初始化接收
	zidaCap_info.EBPct_fromCap = 0.0f;
	zidaCap_info.VBKelvin_fromCap = 0.0f;
	zidaCap_info.status = superCap_offline;
	//zidaCap_info.data_EBPct_status = SuperCap_dataIsError;
	zidaCap_info.msg_u_EBPct.array[0] = 0;
	zidaCap_info.msg_u_EBPct.array[1] = 0;
	
	current_superCap = gen2Cap_ID; //SuperCap_ID;//SuperCap_ID wulie_Cap_CAN_ID
}

void superCap_control_loop()
{
	//发送任务计时, 时间到了开始一次发送
	if(xTaskGetTickCount() - any_Cap_can_msg_send_TimeStamp > any_Cap_can_msg_send_sendPeriod)
	{
		any_Cap_can_msg_send_TimeStamp = xTaskGetTickCount(); //更新时间戳 
			
		if(current_superCap == ZiDaCap_ID)
		{
			uint16_t temp_pwr_command=0;
			
			//紫达控制板
			//Texas 调试
			temp_pwr_command = get_chassis_power_limit();
			
//			zidaCap_info.max_charge_pwr_command = get_chassis_power_limit() - 2.5f;
//			//这时fail safe该不该继续按照当前允许的最大功率来SZL 5-16-2022 还是应该按等级信息来算?
//			zidaCap_info.fail_safe_charge_pwr_command = get_chassis_power_limit() - 2.5f;
			
			//用temp_pwr_command来判断一个不合理数值
			if(temp_pwr_command > 110)
			{
//				zidaCap_info.max_charge_pwr_command = 60 - 3;
//				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
					temp_pwr_command = INITIAL_STATE_CHASSIS_POWER_LIM;
			}
			
			/*功率自适应 分档 标定
			max_charge_pwr_command; fail_safe_charge_pwr_command; 是 uint8_t
			*/
			if(temp_pwr_command == 40)
			{
				zidaCap_info.max_charge_pwr_command = 40 - 3;//2.5f;// 40 档 offset 2.5f
				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 60)
			{
				zidaCap_info.max_charge_pwr_command = 60 - 3;//2.5f;// 60 档 offset 2.5f
				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 80 )
			{
				zidaCap_info.max_charge_pwr_command = 80 - 5;
				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 100)
			{
				zidaCap_info.max_charge_pwr_command = 80;
				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 45)
			{
				zidaCap_info.max_charge_pwr_command = 45 - 3;//2.5f;// 40 档 offset 2.5f
				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 50)
			{
				zidaCap_info.max_charge_pwr_command = 50 - 3;//2.5f;// 40 档 offset 2.5f
				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
			}
			else if(temp_pwr_command == 55)
			{
				zidaCap_info.max_charge_pwr_command = 55 - 3;//2.5f;// 40 档 offset 2.5f
				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
			}
			else
			{
				zidaCap_info.max_charge_pwr_command = 40 - 3;//2.5f;// 40 档 offset 2.5f
				zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
			}
			
//			zidaCap_info.max_charge_pwr_command = 70.0f;
//			zidaCap_info.fail_safe_charge_pwr_command = 40.0f;
				
//			if(zidaCap_info.max_charge_pwr_command >= 101.0f)
//			{
//				zidaCap_info.max_charge_pwr_command = 40;
//			}
//				
//			if(zidaCap_info.fail_safe_charge_pwr_command >= 101.0f)
//			{
//				zidaCap_info.fail_safe_charge_pwr_command = 40;
//			}
			
			//TODO: remove debug 并添加限制幅度
			
			//For Debug Only------------------------------------------------------------------------------------------------------------------------------------
			zidaCap_info.max_charge_pwr_command = 10;//2.5f;// 40 档 offset 2.5f
			zidaCap_info.fail_safe_charge_pwr_command = zidaCap_info.max_charge_pwr_command;
				
			CAN_command_zidaCap(zidaCap_info.max_charge_pwr_command, zidaCap_info.fail_safe_charge_pwr_command);	
		}
		else if(current_superCap == gen3Cap_ID)
		{
			gen3Cap_info.max_charge_pwr_from_ref = get_chassis_power_limit() - 0.0f;
			if(gen3Cap_info.max_charge_pwr_from_ref > CMD_CHARGE_PWR_MAX)//101.0f)
			{
				gen3Cap_info.max_charge_pwr_from_ref = 155; //按最大功率
			}
			
			//计算fail_safe_charge_pwr_ref 修改成用ifelse标定等级标定fail safe, 这个的目的是 
			// TODO 比赛中可能有临时的底盘充电增益, fail safe表示当前的一个安全数值
			gen3Cap_info.fail_safe_charge_pwr_ref = gen3Cap_info.max_charge_pwr_from_ref;
		
			gen3Cap_info.charge_pwr_command = gen3Cap_info.max_charge_pwr_from_ref;
			gen3Cap_info.fail_safe_charge_pwr_command = gen3Cap_info.fail_safe_charge_pwr_ref;
			
			//限制幅度
			gen3Cap_info.charge_pwr_command = uint8_constrain(gen3Cap_info.charge_pwr_command, CMD_CHARGE_PWR_MIN, CMD_CHARGE_PWR_MAX);
			gen3Cap_info.fail_safe_charge_pwr_command = uint8_constrain(gen3Cap_info.fail_safe_charge_pwr_command, CMD_CHARGE_PWR_MIN, CMD_CHARGE_PWR_MAX);
			
			
			// 模式 TODO
			gen3Cap_info.dcdc_mode = 0; //目前自动
			
			// 处理底盘开关机情况
			if(toe_is_error(REFEREE_TOE))
			{
				gen3Cap_info.power_management_chassis_output = 1;// 默认开机 此变量用于调试
				gen3Cap_info.buffer_energy = 0;
			} else {
				gen3Cap_info.power_management_chassis_output = get_chassis_power_output_status();
				gen3Cap_info.buffer_energy = get_chassis_buffer_energy();
			}
			
			if(gen3Cap_info.power_management_chassis_output)
			{
				gen3Cap_info.dcdc_enable = 1;
				CAN_command_gen3Cap(gen3Cap_info.charge_pwr_command, gen3Cap_info.fail_safe_charge_pwr_command, gen3Cap_info.dcdc_enable, gen3Cap_info.dcdc_mode, gen3Cap_info.buffer_energy);
			} else {
				gen3Cap_info.dcdc_enable = 0;
				CAN_command_gen3Cap(gen3Cap_info.charge_pwr_command, gen3Cap_info.fail_safe_charge_pwr_command, gen3Cap_info.dcdc_enable, gen3Cap_info.dcdc_mode, gen3Cap_info.buffer_energy);
			}
		}
		else if(current_superCap == gen2Cap_ID)
		{// gen2Cap 易林超级电容控制板
			//计算max_charge_pwr_from_ref
			gen2Cap_info.max_charge_pwr_from_ref = get_chassis_power_limit() - 0.0f; //2.5f
			
			if(gen2Cap_info.max_charge_pwr_from_ref > CMD_CHARGE_PWR_MAX)//101.0f)
			{
				gen2Cap_info.max_charge_pwr_from_ref = 40;
			}
			
//			//Only for Debug
//			gen2Cap_info.max_charge_pwr_from_ref = 41; //66;
			//--------------------------------------------
			
			//计算fail_safe_charge_pwr_ref 修改成用ifelse标定等级标定fail safe, 这个的目的是 比赛中可能有临时的底盘充电增益, fail safe表示当前的一个安全数值
			gen2Cap_info.fail_safe_charge_pwr_ref = gen2Cap_info.max_charge_pwr_from_ref; //40; // 60; // = gen2Cap_info.max_charge_pwr_from_ref;
		
			gen2Cap_info.charge_pwr_command = gen2Cap_info.max_charge_pwr_from_ref;
			gen2Cap_info.fail_safe_charge_pwr_command = gen2Cap_info.fail_safe_charge_pwr_ref;
			
			//限制幅度
			gen2Cap_info.charge_pwr_command = uint8_constrain(gen2Cap_info.charge_pwr_command, CMD_CHARGE_PWR_MIN, CMD_CHARGE_PWR_MAX);
			gen2Cap_info.fail_safe_charge_pwr_command = uint8_constrain(gen2Cap_info.fail_safe_charge_pwr_command, CMD_CHARGE_PWR_MIN, CMD_CHARGE_PWR_MAX);
			
			CAN_command_gen2Cap(gen2Cap_info.charge_pwr_command, gen2Cap_info.fail_safe_charge_pwr_command);
		}
		else //if(current_superCap == WuLieCap_CAN_ID)
		{//雾列控制板
			wulieCap_info.max_charge_pwr_from_ref = get_chassis_power_limit() - 2.5f;
				
			if(wulieCap_info.max_charge_pwr_from_ref > CMD_CHARGE_PWR_MAX)//101.0f)
			{
				wulieCap_info.max_charge_pwr_from_ref = 40;
			}
			
			//TODO: remove debug 并添加限制幅度
			
			//Only for Debug
			wulieCap_info.max_charge_pwr_from_ref = 30;
				
			wulieCap_info.charge_pwr_command = wulieCap_info.max_charge_pwr_from_ref * 100.f;
			CAN_command_wulieCap(wulieCap_info.charge_pwr_command);
		}
	}
}

/* 通用 判断掉线相关 */

/*下面两个函数; 0->normal/online; 1->error/offline*/
bool_t current_superCap_is_offline()
{
	if (current_superCap == gen3Cap_ID)
	{
		return toe_is_error(GEN3CAP_TOE);
	}
	else if(current_superCap == gen2Cap_ID) //SuperCap_ID
	{
		return toe_is_error(GEN2CAP_TOE);
	}
	else if(current_superCap == WuLieCap_CAN_ID)
	{
		return toe_is_error(WULIECAP_TOE);
	}
	else
	{
		return toe_is_error(ZIDACAP_TOE);
	}
}

bool_t all_superCap_is_error()
{
	return toe_is_error(ZIDACAP_TOE) && toe_is_error(WULIECAP_TOE) && toe_is_error(GEN2CAP_TOE) && toe_is_error(GEN3CAP_TOE);
}

supercap_can_msg_id_e get_current_superCap()
{
		return current_superCap;
}

/* can 驱动层 */
/*
SZL 3-10-2022 下发到SuperCap的数据
SZL 12-27-2022 新增YiLin超级电容
*/
void CAN_command_gen2Cap(uint8_t max_pwr, uint8_t fail_safe_pwr)
{
		uint32_t send_mail_box;
    gen2Cap_tx_message.StdId = RMTypeC_Master_Command_ID;
    gen2Cap_tx_message.IDE = CAN_ID_STD;
    gen2Cap_tx_message.RTR = CAN_RTR_DATA;
    gen2Cap_tx_message.DLC = 0x08;
    gen2Cap_can_send_data[0] = max_pwr;
    gen2Cap_can_send_data[1] = fail_safe_pwr;
    gen2Cap_can_send_data[2] = 0;
    gen2Cap_can_send_data[3] = 0;
    gen2Cap_can_send_data[4] = 0; 
    gen2Cap_can_send_data[5] = 0; 
    gen2Cap_can_send_data[6] = 0; 
    gen2Cap_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&SUPERCAP_CAN, &gen2Cap_tx_message, gen2Cap_can_send_data, &send_mail_box);
}

void CAN_command_gen3Cap(uint8_t max_pwr, uint8_t fail_safe_pwr, uint8_t dcdc_enable, uint8_t dcdc_mode, uint16_t buffer_energy)
{
		uint32_t send_mail_box;
    gen3Cap_tx_message.StdId = RMTypeC_Master_Command_ID_for_gen3Cap;
    gen3Cap_tx_message.IDE = CAN_ID_STD;
    gen3Cap_tx_message.RTR = CAN_RTR_DATA;
    gen3Cap_tx_message.DLC = 0x08;
    gen3Cap_can_send_data[0] = max_pwr;
    gen3Cap_can_send_data[1] = fail_safe_pwr;
    gen3Cap_can_send_data[2] = dcdc_enable;
    gen3Cap_can_send_data[3] = dcdc_mode;
	  // 将 buffer_energy 的低字节赋值给数组的第 4 个位置
    gen3Cap_can_send_data[4] = (uint8_t)(buffer_energy & 0xFF);
	  // 将 buffer_energy 的高字节赋值给数组的第 5 个位置
		gen3Cap_can_send_data[5] = (uint8_t)(buffer_energy >> 8);
	 
    gen3Cap_can_send_data[6] = 0; 
    gen3Cap_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&SUPERCAP_CAN, &gen3Cap_tx_message, gen3Cap_can_send_data, &send_mail_box);
}

void CAN_command_zidaCap(uint8_t max_pwr, uint8_t fail_safe_pwr)
{
		uint32_t send_mail_box;
    zidaCap_tx_message.StdId = RMTypeC_Master_Command_ID;
    zidaCap_tx_message.IDE = CAN_ID_STD;
    zidaCap_tx_message.RTR = CAN_RTR_DATA;
    zidaCap_tx_message.DLC = 0x08;
    zidaCap_can_send_data[0] = max_pwr;
    zidaCap_can_send_data[1] = fail_safe_pwr;
    zidaCap_can_send_data[2] = 0;
    zidaCap_can_send_data[3] = 0;
    zidaCap_can_send_data[4] = 0; 
    zidaCap_can_send_data[5] = 0; 
    zidaCap_can_send_data[6] = 0; 
    zidaCap_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&SUPERCAP_CAN, &zidaCap_tx_message, zidaCap_can_send_data, &send_mail_box);
}

void CAN_command_wulieCap(uint16_t temPower)
{
		uint32_t send_mail_box;
    wulieCap_tx_message.StdId = RMTypeC_Master_Command_ID_for_WuLie;
    wulieCap_tx_message.IDE = CAN_ID_STD;
    wulieCap_tx_message.RTR = CAN_RTR_DATA;
    wulieCap_tx_message.DLC = 0x08;
    wulieCap_can_send_data[0] = temPower >> 8;
    wulieCap_can_send_data[1] = temPower;
    wulieCap_can_send_data[2] = 0;
    wulieCap_can_send_data[3] = 0;
    wulieCap_can_send_data[4] = 0; 
    wulieCap_can_send_data[5] = 0; 
    wulieCap_can_send_data[6] = 0; 
    wulieCap_can_send_data[7] = 0; 
    HAL_CAN_AddTxMessage(&hcan1, &wulieCap_tx_message, wulieCap_can_send_data, &send_mail_box);
}

/* TOE 模块层 功能函数*/

//返回数据相关
void zidaCap_offline_proc()
{
		zidaCap_info.status = superCap_offline;
		
	
}

bool_t zidaCap_is_data_error_proc()
{
		zidaCap_info.status = superCap_online;
	
		if(zidaCap_info.EBPct_fromCap < -100.0f || zidaCap_info.EBPct_fromCap > 200.0f)
		{
			zidaCap_info.data_EBPct_status = SuperCap_dataIsError;
			return 0;
			
		}
		else
		{
			zidaCap_info.data_EBPct_status = SuperCap_dataIsCorrect;
			return 0;
		}
}

void zidaCap_solve_data_error_proc()
{
		//因为其数值可能超过100 所以暂时把这个功能屏蔽掉 ICRA
//		if(zidaCap_info.data_EBPct_status == SuperCap_dataIsError)
//		{
//			if(zidaCap_info.EBPct_fromCap < 0.0f)
//				zidaCap_info.EBPct_fromCap = 0.0f;
//			if(zidaCap_info.EBPct_fromCap > 100.0f)
//				zidaCap_info.EBPct_fromCap = 100.0f;
//		}
		return;
}

//以下为易林超级电容相关
void gen2Cap_offline_proc()
{
		gen2Cap_info.status = superCap_offline;
}

bool_t gen2Cap_is_data_error_proc()
{
		gen2Cap_info.status = superCap_online;
		//永远 return 0;
		return 0;
}

//以下为彭睿第三代超级电容相关
void gen3Cap_offline_proc()
{
		gen3Cap_info.status = superCap_offline;
}

bool_t gen3Cap_is_data_error_proc()
{
	gen3Cap_info.status = superCap_online;
	
	// 就判断个是否有error flag - 不是NORMAL即error
	if(gen3Cap_info.Pflag != CAP_NORMAL)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


//以下为雾列相关的
void wulieCap_offline_proc()
{
		wulieCap_info.status = superCap_offline;
}

bool_t wulieCap_is_data_error_proc()
{
		wulieCap_info.status = superCap_online;
		//永远 return 0;
		return 0;
}

/* 给各个使用 supercap 的模块的功能函数 */

/*
根据目前使用的超级电容 返回电容组电压和剩余能量
这里要做的就是返回合理的 传感器数据 不需要在这里考虑掉线
*/
void cpc_get_superCap_vol_and_energy(fp32* cap_voltage, fp32* EBank) //仅功率控制使用
{
	fp32 temp_EBank=0, temp_cap_voltage=0;
	if(current_superCap == ZiDaCap_ID)
	{
		temp_EBank = zidaCap_info.EBank;
		temp_cap_voltage = zidaCap_info.VBKelvin_fromCap;
		
		temp_EBank = fp32_constrain(temp_EBank, 0.0f, 2106.75f);//确保数据的正确和合理性
		temp_cap_voltage = fp32_constrain(temp_cap_voltage, 0.0f, 26.5f);
		
		*EBank = temp_EBank;
		*cap_voltage = temp_cap_voltage;
		return;
	}
	else if(current_superCap == gen3Cap_ID)
	{
		temp_EBank = gen3Cap_info.EBank;
		temp_cap_voltage = gen3Cap_info.Vbank_f;
		
		temp_EBank = fp32_constrain(temp_EBank, 0.0f, 2106.75f);//确保数据的正确和合理性
		temp_cap_voltage = fp32_constrain(temp_cap_voltage, 0.0f, 28.5f);
		
		*EBank = temp_EBank;
		*cap_voltage = temp_cap_voltage;
		return;
	}
	else if(current_superCap == gen2Cap_ID)
	{
		temp_EBank = gen2Cap_info.EBank;
		temp_cap_voltage = gen2Cap_info.Vbank_f;
		
		temp_EBank = fp32_constrain(temp_EBank, 0.0f, 2106.75f);//确保数据的正确和合理性
		temp_cap_voltage = fp32_constrain(temp_cap_voltage, 0.0f, 28.5f);
		
		*EBank = temp_EBank;
		*cap_voltage = temp_cap_voltage;
		return;
	}
	else
	{
		temp_EBank = wulieCap_info.EBank;
		temp_cap_voltage = wulieCap_info.cap_voltage;
		
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
uint16_t cpc_get_superCap_charge_pwr() //仅功率控制使用
{
	fp32 temp_charge_pwr=0;
	
	if(current_superCap == ZiDaCap_ID)
	{
		temp_charge_pwr = zidaCap_info.max_charge_pwr_command;
		temp_charge_pwr = fp32_constrain(temp_charge_pwr, 0.0f, (fp32)CMD_CHARGE_PWR_MAX);//确保数据的正确和合理性
		
		return (uint16_t)temp_charge_pwr;
	}
	else if(current_superCap == gen3Cap_ID)
	{
		temp_charge_pwr = gen3Cap_info.charge_pwr_command;
		temp_charge_pwr = fp32_constrain(temp_charge_pwr, 0.0f, (fp32)CMD_CHARGE_PWR_MAX);//确保数据的正确和合理性
		
		return (uint16_t)temp_charge_pwr;
	}
	else if(current_superCap == gen2Cap_ID)
	{
		temp_charge_pwr = gen2Cap_info.charge_pwr_command;
		temp_charge_pwr = fp32_constrain(temp_charge_pwr, 0.0f, (fp32)CMD_CHARGE_PWR_MAX);//确保数据的正确和合理性
		
		return (uint16_t)temp_charge_pwr;
	}
	else
	{
		temp_charge_pwr = wulieCap_info.max_charge_pwr_from_ref;
		temp_charge_pwr = fp32_constrain(temp_charge_pwr, 0.0f, (fp32)CMD_CHARGE_PWR_MAX);//确保数据的正确和合理性
		
		return (uint16_t)temp_charge_pwr;
	}
}

fp32 cpc_get_gen3Cap_Pmax()
{
	return gen3Cap_info.Pmax_f;
}

cap_Pflag_e get_gen3Cap_P_Flag()
{
	return gen3Cap_info.Pflag;
}

// 给 chassis energy regulate 的函数
fp32 cer_get_current_cap_boost_mode_pct_threshold()
{
	// 特殊情况1: 无超级电容在线 - 用的缓冲能量
	if(all_superCap_is_error())
	{
		return 0.5;
	}
	
	if(current_superCap == ZiDaCap_ID)
	{
		return 0.5f;
	}
	else if(current_superCap == gen3Cap_ID)
	{
		return 0.5f;
	}
	else if(current_superCap == gen2Cap_ID)
	{
		return 0.5f;
	}
	else
	{
		return 0.5f;
	}
}

// Please make sure relative_EBpct are calculated correctly using cal_capE_relative_pct(..)
// 给 chassis energy regulate 的函数 relative指相对于最小可用能量
fp32 cer_get_current_cap_relative_pct()
{
	// 特殊情况1: 无超级电容在线 - 用的缓冲能量
	if(all_superCap_is_error())
	{
		fp32 pct = get_chassis_buffer_energy() / 60.0f; // 缓冲能量 max 最小60J
		return pct;
	}
	
	if(current_superCap == ZiDaCap_ID)
	{
		return zidaCap_info.relative_EBpct;
	}
	else if(current_superCap == gen2Cap_ID)
	{
		return gen2Cap_info.relative_EBpct;
	}
	else if(current_superCap == gen3Cap_ID)
	{
		return gen3Cap_info.relative_EBpct;
	}
	else
	{
		return wulieCap_info.relative_EBpct;
	}
}

fp32 simple_get_current_cap_pct()
{
	//即插即用的超级电容控制板 判断
	 if(current_superCap == ZiDaCap_ID)
	 {
		 if(toe_is_error(ZIDACAP_TOE))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 return zidaCap_info.EBPct_fromCap;
			 //ui_info.cap_volt = zidaCap_info.VBKelvin_fromCap;
		 }
	 }
	 else if(current_superCap == gen2Cap_ID)
	 {
		 if(toe_is_error(GEN2CAP_TOE))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 return gen2Cap_info.EBPct;
		   //ui_info.cap_volt = gen2Cap_info.Vbank_f;
		 }
	 }
	 else
	 {
		 if(toe_is_error(WULIECAP_TOE))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 return wulieCap_info.EBPct;
		   //ui_info.cap_volt = wulieCap_info.cap_voltage;
		 }
	 }
}

//API for UI and other
fp32 ui_get_current_cap_voltage()
{
	//即插即用的超级电容控制板 判断
	 if(current_superCap == ZiDaCap_ID)
	 {
		 if(toe_is_error(ZIDACAP_TOE))
		 {
				return 0.0f;
		 }
		 else
		 {
			 //ui_info.cap_pct = zidaCap_info.EBPct_fromCap;
			 return zidaCap_info.VBKelvin_fromCap;
		 }
	 }
	 else if(current_superCap == gen3Cap_ID)
	 {
		 // toe_is_error(GEN3CAP_TOE) 出现error code时仍显示电压, OLED 也显示电压, 无打勾
		 if(gen3Cap_info.status == superCap_offline)
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 //ui_info.cap_pct = gen2Cap_info.EBPct;
		   return gen3Cap_info.Vbank_f;
		 }
	 }
	 else if(current_superCap == gen2Cap_ID)
	 {
		 if(toe_is_error(GEN2CAP_TOE))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 //ui_info.cap_pct = gen2Cap_info.EBPct;
		   return gen2Cap_info.Vbank_f;
		 }
	 }
	 else
	 {
		 if(toe_is_error(WULIECAP_TOE))
		 {
			 //ui_info.cap_pct = 0.0f;
			 //ui_info.cap_volt = 0.0f;
			 return 0.0f;
		 }
		 else
		 {
			 //ui_info.cap_pct = wulie_Cap_info.EBPct;
		   return wulieCap_info.cap_voltage;
		 }
	 }
}

// for ui, relative_EBpct are calculated correctly using cal_capE_relative_pct(..) relative指相对于最小可用能量
fp32 ui_get_current_cap_relative_pct()
{
//		return fp32_constrain( fabs((fp32) rc_ctrl.rc.ch[3]) / 660.0f, 0.0f, 1.0f);
		//即插即用的超级电容控制板 判断
		if(current_superCap == ZiDaCap_ID)
		{
			 if(toe_is_error(ZIDACAP_TOE))
			 {
				 return 0.0f;
			 }
			 else
			 {
				 return zidaCap_info.relative_EBpct;
			 }
		 }
		 else if(current_superCap == gen2Cap_ID)
		 {
			 if(toe_is_error(GEN2CAP_TOE))
			 {
				 return 0.0f;
			 }
			 else
			 {
				 return gen2Cap_info.relative_EBpct;
			 }
		 }
		 else if(current_superCap == gen3Cap_ID)
		 {
			 // 出现error code时 百分比显0
			 if(toe_is_error(GEN3CAP_TOE))
			 {
				 //ui_info.cap_pct = 0.0f;
				 //ui_info.cap_volt = 0.0f;
				 return 0.0f;
			 }
			 else
			 {
				 //ui_info.cap_pct = gen2Cap_info.EBPct;
				 return gen3Cap_info.relative_EBpct;
			 }
		 }
		 else
		 {
			 if(toe_is_error(WULIECAP_TOE))
			 {
				 return 0.0f;
			 }
			 else
			 {
				 return wulieCap_info.relative_EBpct;
			 }
		 }
}
