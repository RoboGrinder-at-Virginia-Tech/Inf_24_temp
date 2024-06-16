/**
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       Based on competition rules: chassis power control.
  *             Two ways to limit chassis power 
	*							1) The overall program structure is from DJI 2019 example code and file "chassis_power_control"
	*									based on remaining buffer # to regulate and to control the raw esc control current(message send to can bus)
	*             2) Speed adaptive chassis power control; the program will regulate and control the PID speed of each wheel first.
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
	*  V2.0.0     July-20-2022    Zelin Shen      2. re-write basic chassis power control function; support dynamic charging power
	*																								 add speed adaptive chassis power control;
	*  V3.0.0     August-4-2022   Zelin Shen      3. add chassis power & energy control using superCap; It is a speed adaptive chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "SuperCap_comm.h"

#define CPC_MAX(a,b) ( ((a)>(b)) ? (a):(b) )
#define CPC_MIN(a,b) ( ((a)>(b)) ? (b):(a) )

extern  supercap_can_msg_id_e current_superCap;

void update_energy_buffer_debuff_total_current_limit(uint16_t chassis_p_lim, fp32* total_i_lim);
static void map_superCap_charge_pwr_to_debuff_total_current_limit(uint16_t charge_pwr, fp32* total_i_lim);

//������
uint8_t SZL_debug_place=4;
fp32 SZL_debug_chassis_power_buffer = 0;
//static uint8_t robot_id = 0 ; //test
//������END----

cpc_cap_energy_t cpc_cap_energy;
cpc_buffer_energy_t cpc_buffer_energy; //chassis_energy_control_direct_connect;

//regular power control global var - for debug
fp32 current_scale;

//speed adaptive power control global var - for debug
fp32 speed_adp_scale;

// ָ�õڶ����������ݼܹ���
void gen2_superCap_speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
		cpc_cap_energy.robot_id = get_robot_id();
	
		/*---����һ����Ҫ�õ��� ��̬�䶯������---*/
	  cpc_cap_energy.chassis_power_limit = get_chassis_power_limit();
		if(cpc_cap_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (chassis_e_ctrl.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (chassis_e_ctrl.chassis_power_limit <0) )
		{//ʶ�� ������ ��������ֵ
			cpc_cap_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
		}
		
		//�Ӳ���ϵͳ��ȡ��ǰ��������
		cpc_get_chassis_power_and_buffer(&cpc_cap_energy.chassis_power, &cpc_cap_energy.chassis_power_buffer);//����Ҫ
		//ʶ�� ������ chassis_power �� chassis_power_buffer ��������ֵ��--- SZL: ��ʱ������ --- ���������ݶ��г���������˵ ���Ǻ���Ҫ
		
		//�� �������� ��ȡ��ǰʣ������ ��ȡ��ǰʹ�õĳ������ݵ�ʣ������
	  cpc_get_superCap_vol_and_energy(&cpc_cap_energy.superCap_vol, &cpc_cap_energy.superCap_e_buffer);
		cpc_cap_energy.superCap_charge_pwr = (uint16_t)cpc_get_superCap_charge_pwr();
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* VOL_OUTPUT_CUTOFF_POINT = 14.72f; MINIMUM_VOL=15.81f*/
		if(cpc_cap_energy.superCap_vol <= gen2Cap_VOL_OUTPUT_CUTOFF_POINT) //superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//һ������cut off����
			cpc_cap_energy.critical_val = gen2Cap_MINIMUM_VOL; //gen2Cap_MINIMUM_ENERGY_BUFF;
		}
		else if(cpc_cap_energy.superCap_vol >= gen2Cap_MINIMUM_VOL)
		{//һ���ر�cut off����
			cpc_cap_energy.critical_val = gen2Cap_VOL_OUTPUT_CUTOFF_POINT; //gen2Cap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			cpc_cap_energy.critical_val = gen2Cap_VOL_OUTPUT_CUTOFF_POINT;
		}
		
		//�ֲ� ���㵱ǰ���ù�������
		if(cpc_cap_energy.superCap_vol >= gen2Cap_WARNING_VOL)
		{//��������
//			cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
			cpc_cap_energy.p_max = gen2Cap_MAX_POWER_VALUE;
			
			cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen2Cap_MAX_POWER_VALUE);//����ʵ� ����
			//convert p_max to total_current_limit for esc raw values
		  cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		else if(cpc_cap_energy.superCap_vol > gen2Cap_MINIMUM_VOL && cpc_cap_energy.superCap_vol < gen2Cap_WARNING_VOL)
		{//ֱ�ӵ�������; �����ȽϷ���; ����
			/* Ϊ���и���ƽ������������ ͨ����ǰ�������ݵĳ�繦�� ��ӳ��� ��ǰdebuff�������
			*/
//			chassis_e_ctrl.p_max = 200.0f; //(fp32)(0.5f*6.0f*chassis_e_ctrl.superCap_vol*chassis_e_ctrl.superCap_vol - 0.5f*6.0f*superCap_WARNING_VOL*gen2Cap_WARNING_VOL) / CHASSIS_REFEREE_COMM_TIME;
//			chassis_e_ctrl.p_max = fp32_constrain(chassis_e_ctrl.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen2Cap_MAX_POWER_VALUE);
//			
////			fp32 power_scale = chassis_e_ctrl.superCap_vol / gen2Cap_WARNING_VOL;
//			
//			chassis_e_ctrl.total_current_limit = (fp32)chassis_e_ctrl.p_max / 24.0f * 1000.0f;// * power_scale;
//			//���Ÿ��� p_max
//			chassis_e_ctrl.p_max = chassis_e_ctrl.total_current_limit / 1000.0f * 24.0f;
			//-------------------------------------------
			
//			fp32 power_scale = chassis_e_ctrl.superCap_vol / gen2Cap_WARNING_VOL;
////			update_energy_buffer_debuff_total_current_limit(cpc_buffer_energy.chassis_power_limit, &cpc_buffer_energy.buffer_debuff_total_current_limit);
////			cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_debuff_total_current_limit * power_scale;
//			chassis_e_ctrl.total_current_limit = 16000.0f * power_scale; //16000.0f * power_scale;
//			
//			//���Ÿ��� p_max
//			chassis_e_ctrl.p_max = chassis_e_ctrl.total_current_limit / 1000.0f * 24.0f;
			//8-4-2022 �·���---------------------------------------------------------------------------------------------------
			fp32 power_scale = cpc_cap_energy.superCap_vol / gen2Cap_WARNING_VOL;
			map_superCap_charge_pwr_to_debuff_total_current_limit(cpc_cap_energy.superCap_charge_pwr, &(cpc_cap_energy.buffer_debuff_total_current_limit));//fp32 buffer_debuff_total_current_limit;
			cpc_cap_energy.total_current_limit = cpc_cap_energy.buffer_debuff_total_current_limit * power_scale;
			//���Ÿ��� p_max
			cpc_cap_energy.p_max = cpc_cap_energy.total_current_limit / 1000.0f * 24.0f;
		}
		else
		{//��������
			//���������ﵽ����С��Σ��ֵ��, ��֤��ǰ����������� С�ڵ��� ����ϵͳ�Ĺ�������
			cpc_cap_energy.p_max = (fp32)cpc_cap_energy.superCap_charge_pwr - 4.0f; //(fp32)chassis_e_ctrl.chassis_power_limit - 4.0f;
			
			cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen2Cap_MAX_POWER_VALUE);//����ʵ� ����
			//convert p_max to total_current_limit for esc raw values
		  cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		
//		//������ʵ�� ����ʵ� ����
//		if(fabs(cpc_buffer_energy.p_max) > MAX_POWER_VALUE)
//		{
//			cpc_buffer_energy.p_max = MAX_POWER_VALUE;
//		}
		
		/*---��� ��̬�䶯������ �ĸ���---*/
		
		/*�ȴ��� ����ϵͳ���ߵ����---��ֻ�����������*/
		if(current_superCap_is_offline())
		{
			//�Ͱ���һ�����������ƾ�����; �豸����; ��������� �����ݸ���
			cpc_cap_energy.p_max = REFEREE_OFFLINE_POWER_LIM_VAL;
			cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, REFEREE_OFFLINE_POWER_LIM_VAL);
			cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;
			
			//calculate pid
			/*����ʱ���ֵ�һ������: PID�㷨; set=0; fdb=0; error=0; ����I��, Iout=922; out=530.809
			  ��total_current=1500~3000ʱ ���� ���͹���; 
			  ��������: ����ϵͳ����ʱ��� p_max = 100 -> total_current_limit=2083.3; total_current = 3630.90; ����������ʾ��chassis_power = 3.5w;
			
				(���� �ֶγ��������� ���)�ѻ����˼�����, ҡ����ǰ��, ������ǰ��ת������ٶȺ�; PID set �ӽ� fdb; ʹ�� out����
			  ����total_current=1500~3000ʱ �ϸߵ��̹��ʳ���;
				��������: ҡ���Ƶ���ǰ��; total_current = 1295.1; total_current_limit=20833.3(p_max=499.9); ���ݰ����յ��� chassis_power = 49.43w; ʹ�ù��ʼƲ⵽�Ĺ���Ҳ���
				
				------ ��һ��������ô�����? ------ Ŀǰ��ʱ��REFEREE_OFFLINE_POWER_LIM_VAL���
			*/
			for (uint8_t i = 0; i < 4; i++)
			{
					PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
			}
				
			cpc_cap_energy.total_current = 0.0f;
			//calculate the original motor current set
			//����ԭ����������趨
			for(uint8_t i = 0; i < 4; i++)
			{
					cpc_cap_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			
			if(cpc_cap_energy.total_current > cpc_cap_energy.total_current_limit)
			{
				current_scale = cpc_cap_energy.total_current_limit / cpc_cap_energy.total_current;
				chassis_power_control->motor_speed_pid[0].out*=current_scale;
				chassis_power_control->motor_speed_pid[1].out*=current_scale;
				chassis_power_control->motor_speed_pid[2].out*=current_scale;
				chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}/*��ʼ �ֶγ��������� + �ٶ�����Ӧ�Ĺ��ʿ���*/
		else if(cpc_cap_energy.superCap_vol < cpc_cap_energy.critical_val)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			cpc_cap_energy.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			cpc_cap_energy.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
			
			cpc_cap_energy.current_loop_cnt = 0;// init value
			while(1)
			{
				//calculate pid
				for (uint8_t i = 0; i < 4; i++)
				{
						PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
				}
				
				cpc_cap_energy.total_current = 0.0f;
				//calculate the original motor current set
				//����ԭ����������趨
				for(uint8_t i = 0; i < 4; i++)
				{
						cpc_cap_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
				}
				cpc_cap_energy.total_current_unit_amp = cpc_cap_energy.total_current / 1000.0f;//convert esc control value to unit amp current
				
				if(cpc_cap_energy.total_current > cpc_cap_energy.total_current_limit)//cpc_buffer_energy.total_current_unit_amp * 24.0f > cpc_buffer_energy.p_max)
				{
	//				  fp32 speed_adp_scale;
						
						cpc_cap_energy.current_loop_cnt++;
						if(cpc_cap_energy.current_loop_cnt >= 8)
						{
							//�ﵽ�趨ѭ���������� ֱ������Ŀ���������֤
							current_scale = cpc_cap_energy.total_current_limit / cpc_cap_energy.total_current;
							chassis_power_control->motor_speed_pid[0].out*=current_scale;
							chassis_power_control->motor_speed_pid[1].out*=current_scale;
							chassis_power_control->motor_speed_pid[2].out*=current_scale;
							chassis_power_control->motor_speed_pid[3].out*=current_scale;

							cpc_cap_energy.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
							break;
						}
						else
						{
							//adapt speed
							speed_adp_scale = 0.99f; //cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current; //cpc_buffer_energy.p_max / (cpc_buffer_energy.total_current_unit_amp * 24.0f);
							chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
						}
				}
				else
				{
					cpc_cap_energy.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
					break;
				}
			}
		}
		
		//values and FSM for debug regarding speed-adaptive power ctrl algorithm
		if(cpc_cap_energy.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
		{
			cpc_cap_energy.num_loop_limit_reached++;
		}
		else
		{
			if(cpc_cap_energy.current_loop_cnt != 0)
			{
				cpc_cap_energy.num_of_normal_loop++;
			}
			
			if(cpc_cap_energy.current_loop_cnt > cpc_cap_energy.max_speed_adp_loop_cnt)
			{
				cpc_cap_energy.max_speed_adp_loop_cnt = cpc_cap_energy.current_loop_cnt;
			}
		}
		
		//values for debug
		cpc_cap_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		cpc_cap_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		cpc_cap_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		cpc_cap_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		cpc_cap_energy.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
			cpc_cap_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
		}

}

// �������������� ���� ����ϵͳ���ߵ�����¹���
void gen3_superCap_ref_sys_error_case_sacpc(chassis_move_t *chassis_power_control)
{
	/* ��������˲���ϵͳ����, �Ͷ��� �������������ݷ��������ɲ������������
	 Ϊ�˱��ⲻ��Ҫ��ɲ��, Ҳû�е��̹ضϹ��� */
	
	//fp32 current_scale;

	cpc_buffer_energy.robot_id = get_robot_id();

	/*---����һ����Ҫ�õ��� ��̬�䶯������---*/
	cpc_buffer_energy.chassis_power_limit = get_chassis_power_limit();
	if(cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (cpc_buffer_energy.chassis_power_limit <0) )
	{//ʶ�� ������ ��������ֵ
		cpc_buffer_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
	}
	
	//�Ӳ���ϵͳ��ȡ��ǰ��������
	cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
	
	//ʶ�� ������ chassis_power �� chassis_power_buffer ��������ֵ��--- SZL: ��ʱ������ ---
	
	//judge output cut-off point based on remaining energy and set the buffer ene critical val point
	/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3, 6; MINIMUM_ENERGY_BUFF=10, 13*/
	if(cpc_buffer_energy.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
	{//һ������cut off����
		cpc_buffer_energy.critical_power_buffer = MINIMUM_ENERGY_BUFF;
	}
	else if(cpc_buffer_energy.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
	{//һ���ر�cut off����
		cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
	}
	else
	{// default sts
		cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
	}
	
	//������������ ����gen3 �������ݷ����ĵ�ǰ����ܲ���Ĺ�������
	// gen3cap_Pmax_spt ��һ������ֵ���� Vbank * 15
	cpc_cap_energy.gen3cap_Pmax_spt = fp32_constrain(cpc_get_gen3Cap_Pmax(), cpc_buffer_energy.chassis_power_limit, 500.5f);
	cpc_cap_energy.gen3cap_spt_total_current_limit = (fp32)cpc_cap_energy.gen3cap_Pmax_spt / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	
	//�� �������� ��ȡ��ǰʣ������ ��ȡ��ǰʹ�õĳ������ݵ�ʣ������
	cpc_get_superCap_vol_and_energy(&cpc_cap_energy.superCap_vol, &cpc_cap_energy.superCap_e_buffer);
	cpc_cap_energy.superCap_charge_pwr = (uint16_t)cpc_get_superCap_charge_pwr();
	
	//���ݳ������ݵ�ѹ���� ���޵��� �����ۺ� (gen3 �������ݷ����ĵ�ǰ����ܲ���Ĺ������� ����) => total_current_limit
	if(cpc_cap_energy.superCap_vol >= gen3Cap_WARNING_VOL)
	{//��������
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
		cpc_cap_energy.p_max = gen3Cap_LARGE_POWER_VALUE;
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//����ʵ� ����
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// ���ں��� ������Сֵ ���޸�
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
	}
	else if(cpc_cap_energy.superCap_vol > gen3Cap_MINIMUM_VOL && cpc_cap_energy.superCap_vol < gen3Cap_WARNING_VOL)
	{//��������
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
		cpc_cap_energy.p_max = gen3Cap_MEDIUM_POWER_VALUE;
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//����ʵ� ����
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// ���ں��� ������Сֵ ���޸�
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
	}
	else
	{//��������
		// ��Ȼ����һ�� �����������޷� Ŀǰ�Ͱ��ճ��������·�������
		cpc_cap_energy.p_max = gen3Cap_SMALL_POWER_VALUE;
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_limit + 10.0f); // uncomment���������һ�λ�������
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//����ʵ� ����
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// ���ں��� ������Сֵ ���޸�
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
		//cpc_cap_energy.total_current_limit = cpc_cap_energy.cap_vol_cali_total_current_limit; // uncomment���������һ�λ�������
		
//		// 6-1-2024 �������� ��debuff�޷�����
//		//���������ﵽ����С��Σ��ֵ��, debuff��������
//		fp32 power_scale = cpc_cap_energy.superCap_vol / gen3Cap_WARNING_VOL;
//		map_superCap_charge_pwr_to_debuff_total_current_limit(cpc_cap_energy.superCap_charge_pwr, &(cpc_cap_energy.buffer_debuff_total_current_limit));//fp32 buffer_debuff_total_current_limit;
//		cpc_cap_energy.total_current_limit = cpc_cap_energy.buffer_debuff_total_current_limit * power_scale;
//		//���Ÿ��� p_max
//		cpc_cap_energy.p_max = cpc_cap_energy.total_current_limit / 1000.0f * 24.0f;
		
//		// !!�ڶ����������ݹ���!! ��֤��ǰ����������� С�ڵ��� ����ϵͳ�Ĺ�������
//		cpc_cap_energy.p_max = (fp32)cpc_cap_energy.superCap_charge_pwr - 4.0f; //(fp32)chassis_e_ctrl.chassis_power_limit - 4.0f;
//		
//		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//����ʵ� ����
//		//convert p_max to total_current_limit for esc raw values
//		cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	}
	
	// ��ʼ�����ֶγ��������� ���̵�������
	cpc_cap_energy.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
	
	cpc_cap_energy.current_loop_cnt = 0;// init value
	while(1)
	{
		//calculate pid
		for (uint8_t i = 0; i < 4; i++)
		{
				PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
		}
		
		cpc_cap_energy.total_current = 0.0f;
		//calculate the original motor current set
		//����ԭ����������趨
		for(uint8_t i = 0; i < 4; i++)
		{
				cpc_cap_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
		}
		cpc_cap_energy.total_current_unit_amp = cpc_cap_energy.total_current / 1000.0f;//convert esc control value to unit amp current
		
		if(cpc_cap_energy.total_current > cpc_cap_energy.total_current_limit)//cpc_buffer_energy.total_current_unit_amp * 24.0f > cpc_buffer_energy.p_max)
		{
//				  fp32 speed_adp_scale;
				
				cpc_cap_energy.current_loop_cnt++;
				if(cpc_cap_energy.current_loop_cnt >= 8)
				{
					//�ﵽ�趨ѭ���������� ֱ������Ŀ���������֤
					current_scale = cpc_cap_energy.total_current_limit / cpc_cap_energy.total_current;
					chassis_power_control->motor_speed_pid[0].out*=current_scale;
					chassis_power_control->motor_speed_pid[1].out*=current_scale;
					chassis_power_control->motor_speed_pid[2].out*=current_scale;
					chassis_power_control->motor_speed_pid[3].out*=current_scale;

					cpc_cap_energy.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
					break;
				}
				else
				{
					//adapt speed
					speed_adp_scale = 0.99f; //cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current; //cpc_buffer_energy.p_max / (cpc_buffer_energy.total_current_unit_amp * 24.0f);
					chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
					chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
					chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
					chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
				}
		}
		else
		{
			cpc_cap_energy.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
			break;
		}
	}
		
	//values and FSM for debug regarding speed-adaptive power ctrl algorithm
	if(cpc_cap_energy.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
	{
		cpc_cap_energy.num_loop_limit_reached++;
	}
	else
	{
		if(cpc_cap_energy.current_loop_cnt != 0)
		{
			cpc_cap_energy.num_of_normal_loop++;
		}
		
		if(cpc_cap_energy.current_loop_cnt > cpc_cap_energy.max_speed_adp_loop_cnt)
		{
			cpc_cap_energy.max_speed_adp_loop_cnt = cpc_cap_energy.current_loop_cnt;
		}
	}
	
	//values for debug
	cpc_cap_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
	cpc_cap_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
	cpc_cap_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
	cpc_cap_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
	
	cpc_cap_energy.motor_final_total_current = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		cpc_cap_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
	}

}

// �������������� ���������� ���̹��ʿ���
void gen3_superCap_speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
	//fp32 current_scale;

	cpc_buffer_energy.robot_id = get_robot_id();

	/*---����һ����Ҫ�õ��� ��̬�䶯������---*/
	cpc_buffer_energy.chassis_power_limit = get_chassis_power_limit();
	if(cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (cpc_buffer_energy.chassis_power_limit <0) )
	{//ʶ�� ������ ��������ֵ
		cpc_buffer_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
	}
	
	//�Ӳ���ϵͳ��ȡ��ǰ��������
	cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
	
	//ʶ�� ������ chassis_power �� chassis_power_buffer ��������ֵ��--- SZL: ��ʱ������ ---
	
	//judge output cut-off point based on remaining energy and set the buffer ene critical val point
	/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3, 6; MINIMUM_ENERGY_BUFF=10, 13*/
	if(cpc_buffer_energy.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
	{//һ������cut off����
		cpc_buffer_energy.critical_power_buffer = MINIMUM_ENERGY_BUFF;
	}
	else if(cpc_buffer_energy.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
	{//һ���ر�cut off����
		cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
	}
	else
	{// default sts
		cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
	}
	
	//������������ ����gen3 �������ݷ����ĵ�ǰ����ܲ���Ĺ�������
	// gen3cap_Pmax_spt ��һ������ֵ���� Vbank * 15
	cpc_cap_energy.gen3cap_Pmax_spt = fp32_constrain(cpc_get_gen3Cap_Pmax(), cpc_buffer_energy.chassis_power_limit, 500.5f);
	cpc_cap_energy.gen3cap_spt_total_current_limit = (fp32)cpc_cap_energy.gen3cap_Pmax_spt / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	
	//�� �������� ��ȡ��ǰʣ������ ��ȡ��ǰʹ�õĳ������ݵ�ʣ������
	cpc_get_superCap_vol_and_energy(&cpc_cap_energy.superCap_vol, &cpc_cap_energy.superCap_e_buffer);
	cpc_cap_energy.superCap_charge_pwr = (uint16_t)cpc_get_superCap_charge_pwr();
	
	//���ݳ������ݵ�ѹ���� ���޵��� �����ۺ� (gen3 �������ݷ����ĵ�ǰ����ܲ���Ĺ������� ����) => total_current_limit
	if(cpc_cap_energy.superCap_vol >= gen3Cap_WARNING_VOL)
	{//��������
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
		cpc_cap_energy.p_max = gen3Cap_LARGE_POWER_VALUE;
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//����ʵ� ����
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// ���ں��� �������ֵ ���޸�
		cpc_cap_energy.total_current_limit = CPC_MAX(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
	}
	else if(cpc_cap_energy.superCap_vol > gen3Cap_MINIMUM_VOL && cpc_cap_energy.superCap_vol < gen3Cap_WARNING_VOL)
	{//��������
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
		cpc_cap_energy.p_max = gen3Cap_MEDIUM_POWER_VALUE;
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//����ʵ� ����
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// ���ں��� ������Сֵ ���޸�
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
	}
	else
	{//��������
		// ��Ȼ����һ�� �����������޷� Ŀǰ�Ͱ��ճ��������·�������
		cpc_cap_energy.p_max = gen3Cap_SMALL_POWER_VALUE;
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_limit + 10.0f); // uncomment���������һ�λ�������
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//����ʵ� ����
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// ���ں��� ������Сֵ ���޸�
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
		//cpc_cap_energy.total_current_limit = cpc_cap_energy.cap_vol_cali_total_current_limit; // uncomment���������һ�λ�������
		
//		// 6-1-2024 �������� ��debuff�޷�����
//		//���������ﵽ����С��Σ��ֵ��, debuff��������
//		fp32 power_scale = cpc_cap_energy.superCap_vol / gen3Cap_WARNING_VOL;
//		map_superCap_charge_pwr_to_debuff_total_current_limit(cpc_cap_energy.superCap_charge_pwr, &(cpc_cap_energy.buffer_debuff_total_current_limit));//fp32 buffer_debuff_total_current_limit;
//		cpc_cap_energy.total_current_limit = cpc_cap_energy.buffer_debuff_total_current_limit * power_scale;
//		//���Ÿ��� p_max
//		cpc_cap_energy.p_max = cpc_cap_energy.total_current_limit / 1000.0f * 24.0f;
		
//		// !!�ڶ����������ݹ���!! ��֤��ǰ����������� С�ڵ��� ����ϵͳ�Ĺ�������
//		cpc_cap_energy.p_max = (fp32)cpc_cap_energy.superCap_charge_pwr - 4.0f; //(fp32)chassis_e_ctrl.chassis_power_limit - 4.0f;
//		
//		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//����ʵ� ����
//		//convert p_max to total_current_limit for esc raw values
//		cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	}
	
	//����ϵͳ��������: �ֲ� ���ݲ���ϵͳ�������� ���㵱ǰ���ù�������
	if(cpc_buffer_energy.chassis_power_buffer >= WARNING_ENERGY_BUFF)
	{
		//�������� �ϴ��� 
////			cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
//		cpc_buffer_energy.p_max = MAX_POWER_VALUE;
//		
//		cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//����ʵ� ����
//		//convert p_max to total_current_limit for esc raw values
//		cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// ����������ʱ, ʹ����������ĵ�������
		cpc_buffer_energy.total_current_limit = cpc_cap_energy.total_current_limit;
	}
	else if(cpc_buffer_energy.chassis_power_buffer > MINIMUM_ENERGY_BUFF && cpc_buffer_energy.chassis_power_buffer < WARNING_ENERGY_BUFF)
	{//ֱ�ӵ�������; �����ȽϷ���; ����
		fp32 power_scale = cpc_buffer_energy.chassis_power_buffer / WARNING_ENERGY_BUFF;
//			update_energy_buffer_debuff_total_current_limit(cpc_buffer_energy.chassis_power_limit, &cpc_buffer_energy.buffer_debuff_total_current_limit);
//			cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_debuff_total_current_limit * power_scale;
		cpc_buffer_energy.total_current_limit = 16000.0f * power_scale;
		
		//���Ÿ��� p_max
		cpc_buffer_energy.p_max = cpc_buffer_energy.total_current_limit / 1000.0f * 24.0f;
	}
	else
	{//��������
		//���������ﵽ����С��Σ��ֵ��, ��֤��ǰ����������� С�ڵ��� ����ϵͳ�Ĺ�������
		cpc_buffer_energy.p_max = (fp32)cpc_buffer_energy.chassis_power_limit;//-8.0f;
		
		cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//����ʵ� ����
		//convert p_max to total_current_limit for esc raw values
		cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	}
	
//		//������ʵ�� ����ʵ� ����
//		if(fabs(cpc_buffer_energy.p_max) > MAX_POWER_VALUE)
//		{
//			cpc_buffer_energy.p_max = MAX_POWER_VALUE;
//		}
	
	/*---��� ��̬�䶯������ �ĸ���---*/
	
	/*���ﲻ���� ����ϵͳ���ߵ����*/
	/*����ʱ���ֵ�һ������: PID�㷨; set=0; fdb=0; error=0; ����I��, Iout=922; out=530.809
			��total_current=1500~3000ʱ ���� ���͹���; 
			��������: ����ϵͳ����ʱ��� p_max = 100 -> total_current_limit=2083.3; total_current = 3630.90; ����������ʾ��chassis_power = 3.5w;
		
			(���� �ֶγ��������� ���)�ѻ����˼�����, ҡ����ǰ��, ������ǰ��ת������ٶȺ�; PID set �ӽ� fdb; ʹ�� out����
			����total_current=1500~3000ʱ �ϸߵ��̹��ʳ���;
			��������: ҡ���Ƶ���ǰ��; total_current = 1295.1; total_current_limit=20833.3(p_max=499.9); ���ݰ����յ��� chassis_power = 49.43w; ʹ�ù��ʼƲ⵽�Ĺ���Ҳ���
			���������Hero��ͨ�� �ֶγ��������� �Ѿ������7-20֮ǰ���Զ�û����
		
		7-20����: ��һ�β��Բ�����ʱ��, �Ѳ������ڼ�����, δʹ�ó������� ���ӿ�ת ��ǰȫ���� ��ʱ����ֳ����ʿ�Ѫ, ������cut-off����ʱ; ������װ�� ����ϵͳ�������ݹ���ģ��
		������7-21����ͬ�����и����Ⲣδ����
		
		
			------ ��һ��������ô�����? ------ Ŀǰ��ʱ��REFEREE_OFFLINE_POWER_LIM_VAL���
		*/
	
	/*��ʼ �ֶγ��������� + �ٶ�����Ӧ�Ĺ��ʿ���*/
	if(cpc_buffer_energy.chassis_power_buffer < cpc_buffer_energy.critical_power_buffer)
	{//when below critical pt; just cut-off output
		chassis_power_control->motor_speed_pid[0].out = 0.0f;
		chassis_power_control->motor_speed_pid[1].out = 0.0f;
		chassis_power_control->motor_speed_pid[2].out = 0.0f;
		chassis_power_control->motor_speed_pid[3].out = 0.0f;
		
		cpc_buffer_energy.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
	}
	else
	{
		cpc_buffer_energy.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
		
		cpc_buffer_energy.current_loop_cnt = 0;// init value
		while(1)
		{
			//calculate pid
			for (uint8_t i = 0; i < 4; i++)
			{
					PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
			}
			
			cpc_buffer_energy.total_current = 0.0f;
			//calculate the original motor current set
			//����ԭ����������趨
			for(uint8_t i = 0; i < 4; i++)
			{
					cpc_buffer_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			cpc_buffer_energy.total_current_unit_amp = cpc_buffer_energy.total_current / 1000.0f;//convert esc control value to unit amp current
			
			if(cpc_buffer_energy.total_current > cpc_buffer_energy.total_current_limit)//cpc_buffer_energy.total_current_unit_amp * 24.0f > cpc_buffer_energy.p_max)
			{
//				  fp32 speed_adp_scale;
					
					cpc_buffer_energy.current_loop_cnt++;
					if(cpc_buffer_energy.current_loop_cnt >= 8)
					{
						//�ﵽ�趨ѭ���������� ֱ������Ŀ���������֤
						current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
						chassis_power_control->motor_speed_pid[0].out*=current_scale;
						chassis_power_control->motor_speed_pid[1].out*=current_scale;
						chassis_power_control->motor_speed_pid[2].out*=current_scale;
						chassis_power_control->motor_speed_pid[3].out*=current_scale;

						cpc_buffer_energy.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
						break;
					}
					else
					{
						//adapt speed
						speed_adp_scale = 0.99f; //cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current; //cpc_buffer_energy.p_max / (cpc_buffer_energy.total_current_unit_amp * 24.0f);
						chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
						chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
						chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
						chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
					}
			}
			else
			{
				cpc_buffer_energy.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
				break;
			}
		}
	}
	
	//values and FSM for debug regarding speed-adaptive power ctrl algorithm
	if(cpc_buffer_energy.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
	{
		cpc_buffer_energy.num_loop_limit_reached++;
	}
	else
	{
		if(cpc_buffer_energy.current_loop_cnt != 0)
		{
			cpc_buffer_energy.num_of_normal_loop++;
		}
		
		if(cpc_buffer_energy.current_loop_cnt > cpc_buffer_energy.max_speed_adp_loop_cnt)
		{
			cpc_buffer_energy.max_speed_adp_loop_cnt = cpc_buffer_energy.current_loop_cnt;
		}
	}
	
	//values for debug
	cpc_buffer_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
	cpc_buffer_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
	cpc_buffer_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
	cpc_buffer_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
	
	cpc_buffer_energy.motor_final_total_current = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		cpc_buffer_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
	}
		
}

//�����ٶ�; �ٶ�����Ӧ�� ���ʿ���; ��� �ֶγ���������
void speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
	  //fp32 current_scale;
	
		cpc_buffer_energy.robot_id = get_robot_id();
	
		/*---����һ����Ҫ�õ��� ��̬�䶯������---*/
	  cpc_buffer_energy.chassis_power_limit = get_chassis_power_limit();
	  if(cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (cpc_buffer_energy.chassis_power_limit <0) )
		{//ʶ�� ������ ��������ֵ
			cpc_buffer_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
		}
		
		//�Ӳ���ϵͳ��ȡ��ǰ��������
		cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
		
		//ʶ�� ������ chassis_power �� chassis_power_buffer ��������ֵ��--- SZL: ��ʱ������ ---
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3, 6; MINIMUM_ENERGY_BUFF=10, 13*/
		if(cpc_buffer_energy.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//һ������cut off����
			cpc_buffer_energy.critical_power_buffer = MINIMUM_ENERGY_BUFF;
		}
		else if(cpc_buffer_energy.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
		{//һ���ر�cut off����
			cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		
		//�ֲ� ���㵱ǰ���ù�������
		if(cpc_buffer_energy.chassis_power_buffer >= WARNING_ENERGY_BUFF)
		{//��������
//			cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
			cpc_buffer_energy.p_max = MAX_POWER_VALUE;
			
			cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//����ʵ� ����
			//convert p_max to total_current_limit for esc raw values
		  cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		else if(cpc_buffer_energy.chassis_power_buffer > MINIMUM_ENERGY_BUFF && cpc_buffer_energy.chassis_power_buffer < WARNING_ENERGY_BUFF)
		{//ֱ�ӵ�������; �����ȽϷ���; ����
			fp32 power_scale = cpc_buffer_energy.chassis_power_buffer / WARNING_ENERGY_BUFF;
//			update_energy_buffer_debuff_total_current_limit(cpc_buffer_energy.chassis_power_limit, &cpc_buffer_energy.buffer_debuff_total_current_limit);
//			cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_debuff_total_current_limit * power_scale;
			cpc_buffer_energy.total_current_limit = 16000.0f * power_scale;
			
			//���Ÿ��� p_max
			cpc_buffer_energy.p_max = cpc_buffer_energy.total_current_limit / 1000.0f * 24.0f;
		}
		else
		{//��������
			//���������ﵽ����С��Σ��ֵ��, ��֤��ǰ����������� С�ڵ��� ����ϵͳ�Ĺ�������
			cpc_buffer_energy.p_max = (fp32)cpc_buffer_energy.chassis_power_limit;//-8.0f;
			
			cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//����ʵ� ����
			//convert p_max to total_current_limit for esc raw values
		  cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		
//		//������ʵ�� ����ʵ� ����
//		if(fabs(cpc_buffer_energy.p_max) > MAX_POWER_VALUE)
//		{
//			cpc_buffer_energy.p_max = MAX_POWER_VALUE;
//		}
		
		/*---��� ��̬�䶯������ �ĸ���---*/
		
		/*�ȴ��� ����ϵͳ���ߵ����---��ֻ�����������*/
		if(toe_is_error(REFEREE_TOE))
		{
			//�Ͱ���һ�����������ƾ�����; �豸����; ��������� �����ݸ���
			cpc_buffer_energy.p_max = REFEREE_OFFLINE_POWER_LIM_VAL;
			cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, REFEREE_OFFLINE_POWER_LIM_VAL);
			cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;
			
			//calculate pid
			/*����ʱ���ֵ�һ������: PID�㷨; set=0; fdb=0; error=0; ����I��, Iout=922; out=530.809
			  ��total_current=1500~3000ʱ ���� ���͹���; 
			  ��������: ����ϵͳ����ʱ��� p_max = 100 -> total_current_limit=2083.3; total_current = 3630.90; ����������ʾ��chassis_power = 3.5w;
			
				(���� �ֶγ��������� ���)�ѻ����˼�����, ҡ����ǰ��, ������ǰ��ת������ٶȺ�; PID set �ӽ� fdb; ʹ�� out����
			  ����total_current=1500~3000ʱ �ϸߵ��̹��ʳ���;
				��������: ҡ���Ƶ���ǰ��; total_current = 1295.1; total_current_limit=20833.3(p_max=499.9); ���ݰ����յ��� chassis_power = 49.43w; ʹ�ù��ʼƲ⵽�Ĺ���Ҳ���
			  ���������Hero��ͨ�� �ֶγ��������� �Ѿ������7-20֮ǰ���Զ�û����
			
			7-20����: ��һ�β��Բ�����ʱ��, �Ѳ������ڼ�����, δʹ�ó������� ���ӿ�ת ��ǰȫ���� ��ʱ����ֳ����ʿ�Ѫ, ������cut-off����ʱ; ������װ�� ����ϵͳ�������ݹ���ģ��
			������7-21����ͬ�����и����Ⲣδ����
			
			
				------ ��һ��������ô�����? ------ Ŀǰ��ʱ��REFEREE_OFFLINE_POWER_LIM_VAL���
			*/
			for (uint8_t i = 0; i < 4; i++)
			{
					PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
			}
				
			cpc_buffer_energy.total_current = 0.0f;
			//calculate the original motor current set
			//����ԭ����������趨
			for(uint8_t i = 0; i < 4; i++)
			{
					cpc_buffer_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			
			if(cpc_buffer_energy.total_current > cpc_buffer_energy.total_current_limit)
			{
				current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
				chassis_power_control->motor_speed_pid[0].out*=current_scale;
				chassis_power_control->motor_speed_pid[1].out*=current_scale;
				chassis_power_control->motor_speed_pid[2].out*=current_scale;
				chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}/*��ʼ �ֶγ��������� + �ٶ�����Ӧ�Ĺ��ʿ���*/
		else if(cpc_buffer_energy.chassis_power_buffer < cpc_buffer_energy.critical_power_buffer)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			cpc_buffer_energy.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			cpc_buffer_energy.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
			
			cpc_buffer_energy.current_loop_cnt = 0;// init value
			while(1)
			{
				//calculate pid
				for (uint8_t i = 0; i < 4; i++)
				{
						PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
				}
				
				cpc_buffer_energy.total_current = 0.0f;
				//calculate the original motor current set
				//����ԭ����������趨
				for(uint8_t i = 0; i < 4; i++)
				{
						cpc_buffer_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
				}
				cpc_buffer_energy.total_current_unit_amp = cpc_buffer_energy.total_current / 1000.0f;//convert esc control value to unit amp current
				
				if(cpc_buffer_energy.total_current > cpc_buffer_energy.total_current_limit)//cpc_buffer_energy.total_current_unit_amp * 24.0f > cpc_buffer_energy.p_max)
				{
	//				  fp32 speed_adp_scale;
						
						cpc_buffer_energy.current_loop_cnt++;
						if(cpc_buffer_energy.current_loop_cnt >= 8)
						{
							//�ﵽ�趨ѭ���������� ֱ������Ŀ���������֤
							current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
							chassis_power_control->motor_speed_pid[0].out*=current_scale;
							chassis_power_control->motor_speed_pid[1].out*=current_scale;
							chassis_power_control->motor_speed_pid[2].out*=current_scale;
							chassis_power_control->motor_speed_pid[3].out*=current_scale;

							cpc_buffer_energy.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
							break;
						}
						else
						{
							//adapt speed
							speed_adp_scale = 0.99f; //cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current; //cpc_buffer_energy.p_max / (cpc_buffer_energy.total_current_unit_amp * 24.0f);
							chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
						}
				}
				else
				{
					cpc_buffer_energy.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
					break;
				}
			}
		}
		
		//values and FSM for debug regarding speed-adaptive power ctrl algorithm
		if(cpc_buffer_energy.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
		{
			cpc_buffer_energy.num_loop_limit_reached++;
		}
		else
		{
			if(cpc_buffer_energy.current_loop_cnt != 0)
			{
				cpc_buffer_energy.num_of_normal_loop++;
			}
			
			if(cpc_buffer_energy.current_loop_cnt > cpc_buffer_energy.max_speed_adp_loop_cnt)
			{
				cpc_buffer_energy.max_speed_adp_loop_cnt = cpc_buffer_energy.current_loop_cnt;
			}
		}
		
		//values for debug
		cpc_buffer_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		cpc_buffer_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		cpc_buffer_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		cpc_buffer_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		cpc_buffer_energy.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
			cpc_buffer_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
		}
		
}

// �ٶ�����Ӧ ���̹��ʿ��� ͨ�ýӿ�
void general_speed_adaptive_chassis_power_control(chassis_move_t *sacpc)
{
	// �ж��Ƿ��ǽ����˵�������������
	if(get_current_superCap() == gen3Cap_ID)
	{
		// ����1: �����������յ��˴�����ʱ error_proc���ǵ�toe is err, ֱ�Ӳ��õ��� -> ����case������
		
		if(toe_is_error(GEN3CAP_TOE))
		{
			// cap ���� �� ����error codeʱ
			speed_adaptive_chassis_power_control(sacpc);
		}
		else
		{
			// cap �޴���:
			
			if(toe_is_error(REFEREE_TOE))
			{
				// ref ����
				gen3_superCap_ref_sys_error_case_sacpc(sacpc);
			}
			else
			{
				// ref��cap����������
				gen3_superCap_speed_adaptive_chassis_power_control(sacpc);
			}
		}
	}
	else if(get_current_superCap() == gen2Cap_ID || get_current_superCap() == ZiDaCap_ID || get_current_superCap() == WuLieCap_CAN_ID)
	{
		gen2_superCap_speed_adaptive_chassis_power_control(sacpc);
	}
	else
	{
		speed_adaptive_chassis_power_control(sacpc);
	}
}

static void map_superCap_charge_pwr_to_debuff_total_current_limit(uint16_t charge_pwr, fp32* total_i_lim)
{
#ifdef HERO_CHASSIS_POWER_CONTROL
		if(charge_pwr<=30)
		{
			*total_i_lim = 8333.33f;//200w
		}
		else if(charge_pwr>30 && charge_pwr<=50)
		{
			*total_i_lim = 10000.0f;//240w
		}
		else if(charge_pwr>50 && charge_pwr<=70)
		{
			*total_i_lim = 16000.0f;//384w
		}
		else if(charge_pwr>70 && charge_pwr<=90)
		{
			*total_i_lim = 16000.0f;
		}
		else if(charge_pwr>90 && charge_pwr<=120)
		{
			*total_i_lim = 16000.0f;
		}
		else
	  {//һ������ֵ = 10000
			*total_i_lim = 10000.0f;
		}
#else
		if(charge_pwr<=30)
		{
			*total_i_lim = 13000.0f;//10000.0f; //8333.33f;//200w
		}
		else if(charge_pwr>30 && charge_pwr<=40)
		{
			*total_i_lim = 12000.0f; //13000.0f;//12000.0f;//240w
		}
		else if(charge_pwr>40 && charge_pwr<=50)
		{
			*total_i_lim = 13000.0f; //14500.0f;//13000.0f;//240w
		}
		else if(charge_pwr>50 && charge_pwr<=60)
		{
			*total_i_lim = 15000.0f; //16000.0f;//14500.0f;//240w
		}
		else if(charge_pwr>60 && charge_pwr<=80)
		{
			*total_i_lim = 17500.0f; //20000.0f;//384w
		}
		else if(charge_pwr>80 && charge_pwr<=100)
		{
			*total_i_lim = 20000.0f; //22000.0f;
		}
		else
	  {//һ������ֵ = 10000
			*total_i_lim = 10000.0f;
		}
#endif
}

// this function is deprecated
void update_energy_buffer_debuff_total_current_limit(uint16_t chassis_p_lim, fp32* total_i_lim) //δʹ��
{
#ifdef HERO_CHASSIS_POWER_CONTROL
		/* Hero ͨ���궨 ���������� ӳ��Ϊ ���������ֵ; ����debuff��һ�� */
		//��������
		if(chassis_p_lim == 50)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 70)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 90)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 120)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 55) //Ѫ������
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 60)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 65)
		{
			*total_i_lim = 16000.0f;
		}
		else
		{
			*total_i_lim = 16000.0f;
		}
#else
		/*Infantry ͨ���궨 ���������� ӳ��Ϊ ���������ֵ; ����debuff��һ�� */
		//��������
		if(chassis_p_lim == 40)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 60)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 80)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 100)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 45) //Ѫ������
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 50)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 55)
		{
			*total_i_lim = 16000.0f;
		}
		else
		{
			*total_i_lim = 16000.0f;
		}
#endif
}

/* ���ٶ�����Ӧ�Ŀ��� ����ʹ��*/
void chassis_power_control_non_speed(chassis_move_t *chassis_power_control)
{//�ǳ�������; ֱ�� ���ʱջ�
//    fp32 chassis_power = 0.0f;
//    fp32 chassis_power_buffer = 0.0f;
//    fp32 total_current_limit = 0.0f;
//    fp32 total_current = 0.0f;
    cpc_buffer_energy.robot_id = get_robot_id();
		
	  /*--- ����һ����Ҫ�õ��� ��̬�䶯������ ---*/
	  cpc_buffer_energy.chassis_power_limit = get_chassis_power_limit();
	  if(cpc_buffer_energy.chassis_power_limit > MAX_REASONABLE_CHARGE_PWR)
		{//ʶ�𲻺�����ֵ
			cpc_buffer_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
		}
		//w=VI ���ȼ��������� ӳ��Ϊ ���������ֵ
		//��������
		if(cpc_buffer_energy.chassis_power_limit == 50)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2080.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 70)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2916.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 90)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 3750.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 120)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 5000.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 55) //Ѫ������
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2291.6f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 60)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2500.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 65)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2708.0f;
		}
		else
		{
			 cpc_buffer_energy.chassis_power_limit = 50;
			 cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			 cpc_buffer_energy.buffer_minimum_total_current_limit = 2080.0f;
		}
		
		//�Ӳ���ϵͳ��ȡ��ǰ��������
		cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
		
		//ʶ�� ������ chassis_power �� chassis_power_buffer ��������ֵ��--- SZL: ��ʱ������ ---
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3; MINIMUM_ENERGY_BUFF=10*/
		if(cpc_buffer_energy.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//һ������cut off����
			cpc_buffer_energy.critical_power_buffer = MINIMUM_ENERGY_BUFF;
		}
		else if(cpc_buffer_energy.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
		{//һ���ر�cut off����
			cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		/*--- ��ɶ�̬���ݵĸ��� ---*/
		
		
		/*��ʼ �ֶγ��������� + ���̹��ʿ���*/
		if(cpc_buffer_energy.chassis_power_buffer < cpc_buffer_energy.critical_power_buffer)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			cpc_buffer_energy.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			if(toe_is_error(REFEREE_TOE))
			{
					cpc_buffer_energy.total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
			}
			else if(cpc_buffer_energy.robot_id == RED_ENGINEER || cpc_buffer_energy.robot_id == BLUE_ENGINEER || cpc_buffer_energy.robot_id == 0)
			{
					cpc_buffer_energy.total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
			}
			else
			{
	//        cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
					// power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
					//���ʳ���80w �ͻ�������С��60j,��Ϊ��������С��60��ζ�Ź��ʳ���80w
					/* ���� ���������˻�Ӣ�ۻ�����δ������������ʱ��������������Ϊ 60J
						��������С��60j, ˵����ǰ���ʳ��� ��ǰ�Ĺ��ʵȼ�����
						��������ʣ50jʱ, ����; ����������С��40jʱ, ��ʼ�� ������� if
					*/
					if(cpc_buffer_energy.chassis_power_buffer < WARNING_POWER_BUFF)
					{
							fp32 power_scale;
							if(cpc_buffer_energy.chassis_power_buffer > 10.0f)
							{
									//scale down WARNING_POWER_BUFF
									//��СWARNING_POWER_BUFF
									//SZL: 10<chassis_power_buffer<(WARNING_POWER_BUFF=40)
									power_scale = cpc_buffer_energy.chassis_power_buffer / WARNING_POWER_BUFF;
									cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_debuff_total_current_limit * power_scale;
									SZL_debug_place = 1;
							}
							else
							{
									//only left 10% of WARNING_POWER_BUFF//С��5��ʱ����5�����Ʒ���
									//power_scale = 5.0f / WARNING_POWER_BUFF;
									cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_minimum_total_current_limit;
									SZL_debug_place = 2;
							}
							/*scale down ��С SZL 7-15-2022 �޸�
								���ݵ�ǰ�ȼ� ����ĳ�繦�� ������, ��<=> ��ǰ��繦������Ӧ�ĵ��������ֵ * power_scale
							*/
							//cpc_buffer_energy.total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
							//SZL_debug_chassis_power_buffer = chassis_power_buffer;//SZL �����ӵ�
					}
					else
					{
							/*power > WARNING_POWER ���ʴ���WARNING_POWER; WARNING_POWER=400; POWER_LIMIT=500
								
							*/
							if(cpc_buffer_energy.chassis_power > WARNING_POWER)
							{
									fp32 power_scale;
									if(cpc_buffer_energy.chassis_power < POWER_LIMIT)
									{
											/*scale down;
												WARNING_POWER=400 < chassis_power < POWER_LIMIT=500
											*/
											power_scale = (POWER_LIMIT - cpc_buffer_energy.chassis_power) / (POWER_LIMIT - WARNING_POWER);
											SZL_debug_place = 3;
									}
									else
									{
											//chassis_power > POWER_LIMIT=500
											power_scale = 0.0f;
											SZL_debug_place = 4;
									}
									//total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
									cpc_buffer_energy.total_current_limit = POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED * power_scale;
							}
							//power < WARNING_POWER
							//����С��WARNING_POWER
							else
							{
									//total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
									cpc_buffer_energy.total_current_limit = POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED;
									SZL_debug_place = 5;
							}
	//              cpc_buffer_energy.total_current_limit = 64000.0f; //POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED;
					}
			}

			
			cpc_buffer_energy.total_current = 0.0f;
			//calculate the original motor current set
			//����ԭ����������趨
			for(uint8_t i = 0; i < 4; i++)
			{
					cpc_buffer_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			

			if(cpc_buffer_energy.total_current > cpc_buffer_energy.total_current_limit)
			{
					//fp32 current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
					current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
					chassis_power_control->motor_speed_pid[0].out*=current_scale;
					chassis_power_control->motor_speed_pid[1].out*=current_scale;
					chassis_power_control->motor_speed_pid[2].out*=current_scale;
					chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}
		
		//values for debug
		cpc_buffer_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		cpc_buffer_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		cpc_buffer_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		cpc_buffer_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		cpc_buffer_energy.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
				cpc_buffer_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
		}

}
