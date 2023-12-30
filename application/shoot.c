/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee_usart_task.h"

#include "miniPC_msg.h"
#include "prog_msg_utility.h"
#include "odometer_task.h"

#include "stdlib.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off()    fric_off()      //�ر�����Ħ����

#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)


//extern miniPC_info_t miniPC_info; //3-26-2023 update never use this again

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back_17mm(void); //�о���λ�û��˵�

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η��� �о���λ�û�
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control_absolute_17mm(void);
static void shoot_bullet_control_continuous_17mm(uint8_t shoot_freq);
uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat);

shoot_control_t shoot_control;          //�������


int16_t temp_rpm_left;
int16_t temp_rpm_right;

fp32 temp_speed_setALL = 23; //14; //15 - 3.0; //11.5;//Ŀǰ ICRA Only ����

/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_IN_PID_KP, TRIGGER_SPEED_IN_PID_KI, TRIGGER_SPEED_IN_PID_KD};
		static const fp32 Trigger_position_pid_17mm_outerLoop[3] = {TRIGGER_ANGLE_PID_OUTER_KP, TRIGGER_ANGLE_PID_OUTER_KI, TRIGGER_ANGLE_PID_OUTER_KD};
		
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
    shoot_control.shoot_motor_measure = get_trigger_motor_R_measure_point(); //   get_trigger_motor_measure_point();
    //��ʼ��PID
//    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		shoot_PID_init(&shoot_control.trigger_motor_pid, SHOOT_PID_SEPARATED_INTEGRAL_IN_SPEED, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		
		//17mm�⻷PID
		shoot_PID_init(&shoot_control.trigger_motor_angle_pid, SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS, Trigger_position_pid_17mm_outerLoop, TRIGGER_BULLET_PID_OUTER_MAX_OUT, TRIGGER_BULLET_PID_OUTER_MAX_IOUT);
		
    //��������
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		
		/*12-28-2021 SZL add for 
		infantry pid shooter friction wheel LEFT and RIGHT
		Everything above keep the same as the old PWM shooter
		*/
		//��ʼ�������������
		shoot_control.currentLeft_speed_set = 0;
		shoot_control.currentRight_speed_set = 0;
		shoot_control.currentLIM_shoot_speed_17mm = 0;
		
		
		//LEFT friction PID const init
		static const fp32 Left_friction_speed_pid[3] = {M3508_LEFT_FRICTION_PID_KP, M3508_LEFT_FRICTION_PID_KI, M3508_LEFT_FRICTION_PID_KD};
		//RIGHT friction PID const init
		static const fp32 Right_friction_speed_pid[3] = {M3508_RIGHT_FRICTION_PID_KP, M3508_RIGHT_FRICTION_PID_KI, M3508_RIGHT_FRICTION_PID_KD};

		//���ָ�� M3508ƨ�� ����Ħ����
		shoot_control.left_friction_motor_measure = get_left_friction_motor_measure_point();
		shoot_control.right_friction_motor_measure = get_right_friction_motor_measure_point();
		
		//��ʼ��PID
		PID_init(&shoot_control.left_fric_motor_pid, PID_POSITION, Left_friction_speed_pid, M3508_LEFT_FRICTION_PID_MAX_OUT, M3508_LEFT_FRICTION_PID_MAX_IOUT);
		PID_init(&shoot_control.right_fric_motor_pid, PID_POSITION, Right_friction_speed_pid, M3508_RIGHT_FRICTION_PID_MAX_OUT, M3508_RIGHT_FRICTION_PID_MAX_IOUT);
		//��������
		shoot_control.total_bullets_fired = 0;
		
		get_shooter_id1_17mm_heat_limit_and_heat(&shoot_control.heat_limit, &shoot_control.heat);
		shoot_control.local_heat_limit = shoot_control.heat_limit; //ͨ�� ����
		shoot_control.local_cd_rate = get_shooter_id1_17mm_cd_rate(); //ͨ�� ����
    shoot_control.local_heat = 0.0f;
}

/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */

//===============================================
//uint8_t robot_Level = 0;

int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������
	
//		//��ʼ�ж��ٶ�����
//	  robot_Level = get_robot_level();

//	 	 if(robot_Level == 0){
//	  shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }else if(robot_Level == 1){
//	  shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }else if(robot_Level == 2){
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV2;
//    shoot_control.fric2_ramp.max_value = FRIC_LV2;
//	 }else if(robot_Level == 3){
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV3;
//    shoot_control.fric2_ramp.max_value = FRIC_LV3;
//	 }else{
//	 	shoot_control.fric1_ramp.max_value = FRIC_LV1;
//    shoot_control.fric2_ramp.max_value = FRIC_LV1;
//	 }
//		//����Ϊ�ϰ汾��------------------------------------	 

//------------------�޸ĵȼ��ж� Texas A&M ����ʹ��
	 if(toe_is_error(REFEREE_TOE))
   {
      shoot_control.referee_current_shooter_17mm_speed_limit = INITIAL_PROJECTILE_SPEED_LIMIT_17mm; 
   }
	 else
	 {
			shoot_control.referee_current_shooter_17mm_speed_limit = get_shooter_id1_17mm_speed_limit();
	 }
	 
	 /*�ǵ���� ���ݳ�����������ֵʱ�Ĳ���*/
	 if(shoot_control.referee_current_shooter_17mm_speed_limit > 18)
	 {
		 shoot_control.referee_current_shooter_17mm_speed_limit = 18;
	 }
	 
	 //17mm ������
	 //shoot_control.referee_current_shooter_17mm_speed_limit = 18;//ǿ��ʹ��=18 ���ڵ���-----------------------------------------------------------------------------------------------
	 if(shoot_control.referee_current_shooter_17mm_speed_limit == 15)
	 {
		 shoot_control.currentLIM_shoot_speed_17mm = (fp32)(15 - 3.0);//����----------------------------
		 shoot_control.predict_shoot_speed = shoot_control.currentLIM_shoot_speed_17mm + 2;//����
		 /*1) ����ZYZ�� 15.5 �����14.5
		   2) ����ZYZ�� 14.0 ����� 14.0
		 */
	 }
	 else if(shoot_control.referee_current_shooter_17mm_speed_limit == 18)
	 {//6-15֮ǰ������һֱ�ǰ�������Ե�
		 // 18- 4.5 Ϊ RMUL ʵ�� 16.7-17.1 - .3 m/s ���ٱ궨 SZL
		 shoot_control.currentLIM_shoot_speed_17mm = (fp32)(18 - 4.5);
		 shoot_control.predict_shoot_speed = shoot_control.currentLIM_shoot_speed_17mm + 3;
		 /*
		 1) ����ZYZ�� 16.5 ����� 16.5
		 */
	 }
	 else
	 {//Ĭ������15
		 shoot_control.currentLIM_shoot_speed_17mm = (fp32)(15 - 3.0);//����-----------------------------
		 shoot_control.predict_shoot_speed = shoot_control.currentLIM_shoot_speed_17mm + 2;//����
	 }
	 
	 //���ٲ��� 12-28
	 shoot_control.currentLIM_shoot_speed_17mm = (fp32)temp_speed_setALL;
	 shoot_control.predict_shoot_speed = shoot_control.currentLIM_shoot_speed_17mm;
	 
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0;
			  //��һ������PID
				shoot_PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.trigger_motor_angle_pid);
			
				//��ʼ����һ��PID֡�ļ���
				shoot_control.set_angle = shoot_control.angle;
				shoot_control.speed_set = shoot_control.speed;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        shoot_control.trigger_speed_set = 0.0f;
        shoot_control.speed_set = 0.0f;
        //���if ������ ����ûɶ��
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
				//shoot_control.trigger_speed_set = 0.0f;//------------
        //���ò����ֵ��ٶ�
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;//-----------------------------------------
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control_absolute_17mm();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
//        //���ò����ֵĲ����ٶ�,��������ת��ת���� 5-31-2023ǰ�ϴ���
//        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
//        trigger_motor_turn_back_17mm();
			
				//��PIDλ���⻷��, �������궨����Ƶ
				shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;//-----------------------------------------
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
			
				shoot_bullet_control_continuous_17mm(8); // 3v3 �ĳ��� Ϊ 5 6 - 1v1������8
				
//				if(toe_is_error(REFEREE_TOE))
//				{
//					shoot_bullet_control_continuous_17mm(6);
//				}
//				else
//				{
//					if(shoot_control.heat_limit >= )
//					{
//					}
//					else
//					{
//					}
//				}
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
				//λ�û�PID �����������
				shoot_control.set_angle = shoot_control.angle;
			
				//�ٶ�PID �����������
				shoot_control.speed_set = shoot_control.speed;
			
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
//        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
//        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
			
				shoot_control.fric_pwm1 = FRIC_OFF;
				shoot_control.fric_pwm2 = FRIC_OFF;
				//�رղ���Ҫб�¹ر�
			
			
			//��ɲ�� -Ȼ����0����
			shoot_control.currentLeft_speed_set = M3508_FRIC_STOP;
			shoot_control.currentRight_speed_set = M3508_FRIC_STOP;
			M3508_fric_wheel_spin_control(-shoot_control.currentLeft_speed_set, shoot_control.currentRight_speed_set);
			//��ɲ��Ȼ��0����
			if(shoot_control.left_fricMotor.fricW_speed < 1.1f && shoot_control.right_fricMotor.fricW_speed < 1.1f)
			{
				CAN_cmd_friction_wheel(0, 0); //�����ɲ��, ��ʼ no power
			}
    }
    else
    {
        shoot_laser_on(); //���⿪��
			
				//5-27-2023���Ӵ���PID----
			  if(shoot_control.block_flag == 0)
				{ //�˵����ô���PID
//					shoot_control.speed_set = PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.angle, shoot_control.set_angle);
					shoot_control.speed_set = shoot_PID_calc(&shoot_control.trigger_motor_angle_pid, shoot_control.angle, shoot_control.set_angle);
        }
				
        //���㲦���ֵ��PID
//        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
				shoot_PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        
#if TRIG_MOTOR_TURN
				shoot_control.given_current = -(int16_t)(shoot_control.trigger_motor_pid.out);
#else
				shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
#endif
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
				
				//����Ħ�����ٶ�
				shoot_control.currentLeft_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
				shoot_control.currentRight_speed_set = shoot_control.currentLIM_shoot_speed_17mm;
				
//				M3508_fric_wheel_spin_control(-shoot_control.currentLeft_speed_set, shoot_control.currentRight_speed_set);//�������� - �ϲ���
				M3508_fric_wheel_spin_control(shoot_control.currentLeft_speed_set, -shoot_control.currentRight_speed_set);//��������
    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);// + 19);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
		
		
		
    shoot_fric1_on(shoot_control.fric_pwm1);
    shoot_fric2_on(shoot_control.fric_pwm2);
		
		//vTaskDelay(5);
		
		//M3508_fric_wheel_spin_control(-tempLeft_speed_set, tempRight_speed_set);
//		M3508_fric_wheel_spin_control(-shoot_control.currentLeft_speed_set, shoot_control.currentRight_speed_set);
		
    return shoot_control.given_current;
}




/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
/*
����Ħ����״̬�����л�����: �ϲ�һ�� SHOOT_READY_FRIC; �ٲ�һ�� SHOOT_STOP;
б������ ������ ֮ǰ�������ǻ����� �����; ��б�µ�MAXʱ �����Ԥ��

*/
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;//�ϲ�һ�ο���Ħ����
			  shoot_control.user_fire_ctrl = user_SHOOT_SEMI;//����Ħ���� Ĭ��auto
			  shoot_control.key_Q_cnt = 2;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;//�ϲ�һ���ٹر�Ħ����
			  shoot_control.key_Q_cnt = 0;
    }
				
    //�����е��� ����ʹ�ü��̿���Ħ����
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC; 
				shoot_control.user_fire_ctrl = user_SHOOT_AUTO;//����Ħ���� Ĭ��auto
    }
    //�����е��� ����ʹ�ü��̹ر�Ħ����
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
			  shoot_control.key_Q_cnt = 0;
    }

		//�����е�ʱ�� ����Q ���¼�� �� �û����״̬ ģʽ�ж�
		if(switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && (shoot_control.shoot_mode > SHOOT_STOP))
		{
				//shoot_control.key_Q_cnt++;
				if(shoot_control.last_key_Q_sts == 0)
				{
					shoot_control.key_Q_cnt++;
					//shoot_control.shoot_mode = SHOOT_READY;
					shoot_control.last_key_Q_sts = 1;
				}
				else
				{
					shoot_control.last_key_Q_sts = 1;
				}
		}
		else
		{
			 shoot_control.last_key_Q_sts = 0;
		}
		
		if(shoot_control.key_Q_cnt > 2)
		{
			shoot_control.key_Q_cnt = 1;//ʵ�� ������
		}
		
		if(shoot_control.key_Q_cnt == 1)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_AUTO;
		}
		else if(shoot_control.key_Q_cnt == 2)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_SEMI;
		}
		else if(shoot_control.key_Q_cnt == 0)
		{
			shoot_control.user_fire_ctrl = user_SHOOT_OFF;
		}
		//---------Q���������Լ���ؼ�����---------
		/*�����Ƕ���DJI��Դ����ļ��� - ͨ������ ��ͨ�˲�ֵ ֮ǰ��shoot_mode, ǰ����(����<-map->user_fire_ctrl);
			�ȶԵ�ǰ shoot_mode ��ֵһ��(��������<-map->shoot_mode), �������user_fire_ctrl���shoot_mode��ֵ�ڶ���(user_fire_mode<-map->shoot_mode) - ����Ϊshoot_mode�л��ܿ�, ���ƻ�ֱ�������״̬��
			ʵ�ֶ��user_fire_ctrlӳ�䵽���޸�shoot_mode - ����ɨ�軹�����Ż�*/
    if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET; //��Ħ�������Ԥ�� //A
    }
    else if(shoot_control.shoot_mode == SHOOT_READY_BULLET) //&& shoot_control.key == SWITCH_TRIGGER_ON)
    {
			shoot_control.shoot_mode = SHOOT_READY;  //shoot_control.key��Ĭ�ϳ�ʼ��Ϊ0 ����:��һ�λ����A �ڶ��λ������� ʹ��shoot_mode = SHOOT_READY
    }
    else if(0) //shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;//�Ӳ���������else if
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
			if(shoot_control.trigger_motor_17mm_is_online)//��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬
			{
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
			}
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.key_time++;
				//΢������ ����ʱ�䵽��֮��, ��Ū��SHOOT_READY_BULLET
				//������ ����ʱ��
        if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
    }
/*
    if(shoot_control.shoot_mode > SHOOT_READY_FRIC){ //�Զ�����ָ���
		   if(shootCommand == 0xff){
			 shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			 }else if(shootCommand == 0x00){
			 shoot_control.shoot_mode = SHOOT_READY_BULLET;
			 }
		}
	*/
		
		/*�������鿪���߼�  X��������*/
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_X)
		{
			if(shoot_control.last_key_X_sts == 0)
			{
				shoot_control.key_X_cnt++;
				shoot_control.last_key_X_sts = 1;
			}
			else
			{
				shoot_control.last_key_X_sts = 1;
			}
		}
		else
		{
			shoot_control.last_key_X_sts = 0;
		}
		
		if(shoot_control.key_X_cnt > 2)
		{
			shoot_control.key_X_cnt = 1;//ʵ�� ������
		}
		//press X to turn on auto aim, 1=aid 2=lock 
		//�� ������ֻ�ܿ���aim
		if(shoot_control.key_X_cnt == 0)
		{
			set_autoAimFlag(0); //miniPC_info.autoAimFlag = 0;
		}
		else if(shoot_control.key_X_cnt == 1) 
		{
			set_autoAimFlag(1); //miniPC_info.autoAimFlag = 1;
		}
		else if(shoot_control.key_X_cnt == 2)
		{
			//miniPC_info.autoAimFlag = 2;
			set_autoAimFlag(1); //miniPC_info.autoAimFlag = 1;
		}
		
		if(shoot_control.press_r_time == PRESS_LONG_TIME_R || shoot_control.press_key_V_time == PRESS_LONG_TIME_V)
		{
			set_autoAimFlag(2); //miniPC_info.autoAimFlag = 2;
			//shoot_control.key_X_cnt = 2;
		}
//		else
//		{
//			miniPC_info.autoAimFlag = 1;
//			shoot_control.key_X_cnt = 1;
//		}
		
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_C) // press C to turn off auto aim
		{
			set_autoAimFlag(0); //miniPC_info.autoAimFlag = 0;
			shoot_control.key_X_cnt = 0;
		}
		//X���������Լ���ؼ�����
		
		//10-ĳһ��-2022�޸�
		//���������ж�; ��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬
    if(shoot_control.shoot_mode > SHOOT_READY_FRIC && shoot_control.trigger_motor_17mm_is_online)
    {
        //��곤��һֱ�������״̬ ��������
				//(shoot_control.user_fire_ctrl==user_SHOOT_AUTO && shoot_control.press_l)
			  
			  //��Ҫ TODO: ��� ����ʶ��
			
				if(shoot_control.user_fire_ctrl==user_SHOOT_SEMI)
				{
					//(((miniPC_info.shootCommand == 0xff) && (miniPC_info.autoAimFlag > 0))|| (shoot_control.press_l_time == PRESS_LONG_TIME_L ) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0) )|| (shoot_control.press_l_time == PRESS_LONG_TIME_L ) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode =SHOOT_READY_BULLET;
					}
				}
				else if(shoot_control.user_fire_ctrl==user_SHOOT_AUTO)
				{
					//(((miniPC_info.shootCommand == 0xff) && (miniPC_info.autoAimFlag > 0)) || (shoot_control.press_l ))
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0) ) || (shoot_control.press_l ))
					{
							shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode =SHOOT_READY_BULLET;
					}
				}
				else
				{
					//(((miniPC_info.shootCommand == 0xff) && (miniPC_info.autoAimFlag > 0)) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					if (( (get_shootCommand() == 0xff) && (get_autoAimFlag() > 0) ) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
					{
							shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
					}
					else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
					{
							shoot_control.shoot_mode =SHOOT_READY_BULLET;
					}
				}
    }

		//���¿�ʼ������
		shoot_heat_update_calculate(&shoot_control);
		//17mm ref��������
    get_shooter_id1_17mm_heat_limit_and_heat(&shoot_control.heat_limit, &shoot_control.heat);
		//ֻ�ò���ϵͳ���ݵĳ���������
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }
		//����: �ѵ�referee uart���ߺ� ��û������������?
		
//		//δʹ��ʵʱ��̼Ƶĳ��������� - ֻ�ǿ���ʱ��һ������δ��ֲ������������
//		if(shoot_control.local_heat + LOCAL_SHOOT_HEAT_REMAIN_VALUE >= (fp32)shoot_control.local_heat_limit)
//    {
//        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
//        {
//            shoot_control.shoot_mode =SHOOT_READY_BULLET;
////						shoot_control.local_heat -= ONE17mm_BULLET_HEAT_AMOUNT; //��ǰ�ӵ�δ���ȥ -- �������
//        }
//    }
		
		//ʹ��ʵʱ��̼Ƶĳ���������
		if(shoot_control.rt_odom_local_heat[0] + LOCAL_SHOOT_HEAT_REMAIN_VALUE >= (fp32)shoot_control.local_heat_limit)
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }
		
//    //�����̨״̬�� ����״̬���͹ر����
//    if (gimbal_cmd_to_shoot_stop())
//    {
//        shoot_control.shoot_mode = SHOOT_STOP;
//    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          ������ݸ���
	shoot motor �ǲ������
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
#if TRIG_MOTOR_TURN
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] - (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;
#else
		speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;
#endif
		
		//����Ƿ����߼��
		/*ֻɨ��һ�ΰ������˼·*/
		if(toe_is_error(TRIGGER_MOTOR17mm_R_TOE))
		{
			shoot_control.trigger_motor_17mm_is_online = 0x00;
		}
		else
		{
			shoot_control.trigger_motor_17mm_is_online = 0x01;
		}

//    /*
//		������ע�͹�, ��д����: ���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
//		Ӧ����:
//		�⼸�仰��Ŀ�����жϵ�������, ������ֵ���л���ʱ, ��Ҫȷ����һ֡step�ķ���, ������rpm��, rpm��˲ʱ��
//		*/
//    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count--;
//    }
//    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count++;
//    }

////    if (shoot_control.ecd_count == FULL_COUNT)
//    {
//        shoot_control.ecd_count = -(FULL_COUNT - 1);//-(FULL_COUNT - 1);
//    }
//    else if (shoot_control.ecd_count == -FULL_COUNT)
//    {
//        shoot_control.ecd_count = FULL_COUNT - 1;
//    }
//    //���������Ƕ� 5-19֮ǰ
//		//ecd_count ������ ���� ����Ȧ�� ����
//		//֮ǰ��ת�˼�Ȧ + ��ǰ�ı�����ֵ ����ת��Ϊ������
//    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
		
		//���������ֵ���ֺ� �Բ�����angle�ļ��� SZL 5-19
		//֮ǰ��ת�˼�Ȧ + ��ǰ�ı�����ֵ ����ת��Ϊ������ ����ֵ��̼�
#if TRIG_MOTOR_TURN
		shoot_control.angle = -(shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
#else
		shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->delta_ecd) * MOTOR_ECD_TO_ANGLE;
		//shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
#endif

		//��ʵ���԰����а������״̬���ŵ����� ��set mode���Ƶ������� ��Ȼ�������
		
		//����V��ʱ, Vֻ�Ǽ�¼����һ��״̬, ����û�м���
		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_V)
		{
			if(shoot_control.press_key_V_time < PRESS_LONG_TIME_V)
			{
				shoot_control.press_key_V_time++;
			}
			shoot_control.last_key_V_sts = 1;
		}
		else
		{
			shoot_control.last_key_V_sts = 0;
			shoot_control.press_key_V_time = 0;
		}
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //������ʱ
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME_L)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME_R)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }
		//12-30-2021 SZL ��� friction ��� ���� ����
		shoot_control.left_fricMotor.fricW_speed = M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN * shoot_control.left_friction_motor_measure->speed_rpm;
		shoot_control.right_fricMotor.fricW_speed = M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN * shoot_control.right_friction_motor_measure->speed_rpm;
		
		//Added for J-scope debug
		temp_rpm_right = shoot_control.right_friction_motor_measure->speed_rpm;
		temp_rpm_left = shoot_control.left_friction_motor_measure->speed_rpm;
		
}

////�ϵ�ģ��λ�ÿ��� -���˵� 5-27-2023ע��
//static void trigger_motor_turn_back_17mm(void)
//{
//    if( shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.speed_set = shoot_control.trigger_speed_set;
//    }
//    else
//    {
//        shoot_control.speed_set = -shoot_control.trigger_speed_set;
//    }

//    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.block_time++;
//        shoot_control.reverse_time = 0;
//    }
//    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
//    {
//        shoot_control.reverse_time++;
//    }
//    else
//    {
//        shoot_control.block_time = 0;
//    }
//}

///**
//  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η��� �ϵ�ģ��λ�ÿ��� 5-27-2023ע��
//  * @param[in]      void
//  * @retval         void
//  */
//static void shoot_bullet_control_17mm(void)
//{
//    //ÿ�β��� 1/4PI�ĽǶ�
//    if (shoot_control.move_flag == 0)
//    {
//        shoot_control.set_angle = (shoot_control.angle + PI_TEN);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
//        shoot_control.move_flag = 1;
//    }
//		
//		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
//		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
//	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
//		if(shoot_control.trigger_motor_17mm_is_online == 0x00)
//		{
//				shoot_control.set_angle = shoot_control.angle;
//				return;
//		}
//		
//    if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
//    {
//        shoot_control.shoot_mode = SHOOT_DONE;
//    }
//    //����Ƕ��ж�
//    if ((shoot_control.set_angle - shoot_control.angle) > 0.05f)//(rad_format(shoot_control.set_angle - shoot_control.angle) > 0.0005f)//0.15f) //pr�Ķ�ǰΪ0.05f shooter_rad_format
//    {
//        //û����һֱ������ת�ٶ�
//        shoot_control.trigger_speed_set = TRIGGER_SPEED;
//        trigger_motor_turn_back_17mm();
//    }
//    else
//    {
//        shoot_control.move_flag = 0;
//			  shoot_control.shoot_mode = SHOOT_DONE; //pr test
//    }
//   
//}

//�ٶȻ����� �˵� �µ��ٶȻ��˵�
static void trigger_motor_turn_back_17mm(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {//δ������ת
        //shoot_control.speed_set = shoot_control.trigger_speed_set;
				shoot_control.block_flag = 0;
    }
    else
    {		//������ת
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.trigger_motor_pid);
				shoot_control.block_flag = 1;//block_flag=1��ʾ������ת; block_flag=0��ʾδ������ת������ɶ�ת���
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

		//����תʱ��
    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;//������ת��ʼ��ʱ
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;//��ʼ��ת ��ʼ��ʱ��תʱ��
    }
    else
    {//��ɷ�ת
//				PID_clear(&shoot_control.trigger_motor_pid);
				shoot_PID_clear(&shoot_control.trigger_motor_pid);
				shoot_control.block_flag = 0;
        shoot_control.block_time = 0;	
    }
		
		if(shoot_control.last_block_flag == 0 && shoot_control.block_flag == 1)
		{//�շ�����ת
			shoot_control.total_bullets_fired--; //��ǰ�ӵ�δ���ȥ
			shoot_control.local_heat -= ONE17mm_BULLET_HEAT_AMOUNT;
		}
		
		if(shoot_control.last_block_flag == 1 && shoot_control.block_flag == 0)
		{//���һ�ζ�ת���
			//������ǰ�Ĵ�����
			shoot_control.set_angle = shoot_control.angle;
		}
		
		shoot_control.last_block_flag = shoot_control.block_flag;
		/*block_flag = 1������ת
			block_flag = 0δ������ת*/
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���, ��ȷ�ĽǶȻ�PID
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control_absolute_17mm(void)
{
	  //ÿ�β��� 120�� �ĽǶ�
    if (shoot_control.move_flag == 0)
    {
				/*һ��ֻ��ִ��һ�η�������, ��һ�η�������������ɺ�, ��δ���ʱ, ����ڶ���->����ִ�еڶ��η���
				һ�β�һ����λ
        */
				shoot_control.set_angle = (shoot_control.angle + PI_TEN);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.move_flag = 1;
			  shoot_control.total_bullets_fired++; //
			  shoot_control.local_heat += ONE17mm_BULLET_HEAT_AMOUNT;
    }
		
		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor_17mm_is_online == 0x00)
		{
				shoot_control.set_angle = shoot_control.angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }
		//��ʣ���С�Ƕ�ʱ, �㵽����
		if(shoot_control.set_angle - shoot_control.angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.trigger_speed_set = TRIGGER_SPEED;
				//������Ҫֱ���ٶȿ���ʱ�Ŀ����ٶ������Ƕ�ת��ת�ٶ� TRIGGER_SPEED����ָ��������ת����
				trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.move_flag = 0;
				shoot_control.shoot_mode = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0��ǰ֡������� û������ִ�еķ�������
			shoot_control.move_flag = 1��ǰ֡������� ������ִ�еķ�������*/
}

//������������ ÿ����ٿ�; shoot_freq��Ƶ
static void shoot_bullet_control_continuous_17mm(uint8_t shoot_freq)
{
		 //if(xTaskGetTickCount() % (1000 / shoot_freq) == 0) //1000Ϊtick++��Ƶ��
		 if( get_para_hz_time_freq_signal_FreeRTOS(shoot_freq) )
		 {
			 	shoot_control.set_angle = (shoot_control.angle + PI_TEN);//rad_format(shoot_control.angle + PI_TEN); shooter_rad_format
        shoot_control.move_flag = 1; //�̶�Ƶ����������ʱ, move_flag��û��ʹ��, ����ʱ����нǶ�����
			  shoot_control.total_bullets_fired++; //
			  shoot_control.local_heat += ONE17mm_BULLET_HEAT_AMOUNT;
		 }
		
		/*��δ���Ĳ�������NewINF v6.4.1 �в��Ե�, Ҳ���ǲ������:(��������ϵ�ʱ, shoot_mode״̬�����ᱻ��Ϊ�������״̬)
		������߼���: �����������ϵ�, shoot_mode״̬�����ᱻ��Ϊ�������״̬, �������˺���; ��δ���ֻ�������ﱣ��
	  �������, ����������ϵ���������ʱ, ������ǰ��������*/
		if(shoot_control.trigger_motor_17mm_is_online == 0x00)
		{
				shoot_control.set_angle = shoot_control.angle;
				return;
		}
		
		if(0)//shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }
		//��ʣ���С�Ƕ�ʱ, �㵽����
		if(shoot_control.set_angle - shoot_control.angle > 0.05f) //(fabs(shoot_control.set_angle - shoot_control.angle) > 0.05f)
		{
				shoot_control.trigger_speed_set = TRIGGER_SPEED;
				//������Ҫֱ���ٶȿ���ʱ�Ŀ����ٶ������Ƕ�ת��ת�ٶ� TRIGGER_SPEED����ָ��������ת����
				trigger_motor_turn_back_17mm();
		}
		else
		{
			
				shoot_control.move_flag = 0;
				shoot_control.shoot_mode = SHOOT_DONE; 
		}
		/*shoot_control.move_flag = 0��ǰ֡������� û������ִ�еķ�������
			shoot_control.move_flag = 1��ǰ֡������� ������ִ�еķ�������*/
}

const shoot_control_t* get_robot_shoot_control()
{
	return &shoot_control;
}

/* ---------- getter method ��ȡ���� ---------- */
shoot_mode_e get_shoot_mode()
{
	return shoot_control.shoot_mode;
}

user_fire_ctrl_e get_user_fire_ctrl()
{
	return shoot_control.user_fire_ctrl;
}

uint8_t get_ammoBox_sts()
{
	return shoot_control.ammoBox_sts;
}
/* ---------- getter method end ---------- */

/*
������� ������� �Լ���PID, ��Ҫʹ�û��ַ��� ��ֵȡ�����豸����
*/
void shoot_PID_init(shoot_pid_t *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 shoot_PID_calc(shoot_pid_t *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
		
		pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

		//���ַ����㷨
    pid->Pout = pid->Kp * pid->error[0];
		
		if(pid->mode == SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS)
		{
				if(fabs(pid->error[0]) < PID_TRIG_POSITION_INTEGRAL_THRESHOLD)
				{//�ڷ�Χ��, �Դ�ʱ��ֵ���л���
					pid->Iout += pid->Ki * pid->error[0];
				}
				else
				{//���ڷ�Χ��, ��ʱ���Ʒ�
					pid->Iout = pid->Iout;
				}

				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				abs_limit(&pid->Iout, pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				abs_limit(&pid->out, pid->max_out);
		}
		else
		{
				if(fabs(pid->error[0]) < PID_TRIG_SPEED_INTEGRAL_THRESHOLD)
				{//�ڷ�Χ��, �Դ�ʱ��ֵ���л���
					pid->Iout += pid->Ki * pid->error[0];
				}
				else
				{//���ڷ�Χ��, ��ʱ���Ʒ�
					pid->Iout = pid->Iout;
				}

				pid->Dbuf[2] = pid->Dbuf[1];
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				pid->Dout = pid->Kd * pid->Dbuf[0];
				abs_limit(&pid->Iout, pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				abs_limit(&pid->out, pid->max_out);
		}

    return pid->out;
		
}

//����PID
void shoot_PID_clear(shoot_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat)
{
	if(!toe_is_error(REFEREE_TOE))
  {
		 get_shooter_id1_17mm_heat_limit_and_heat(&shoot_heat->heat_limit, &shoot_heat->heat);
		 shoot_heat->local_heat_limit = shoot_heat->heat_limit;
		 shoot_heat->local_cd_rate = get_shooter_id1_17mm_cd_rate();
  }
	else
	{
		 //����ϵͳ����ʱ hard code һ��Ĭ�ϵ���ȴ������
//		 get_shooter_id1_17mm_heat_limit_and_heat(&shoot_heat->heat_limit, &shoot_heat->heat);
		 shoot_heat->local_heat_limit = LOCAL_HEAT_LIMIT_SAFE_VAL;
		 shoot_heat->local_cd_rate = LOCAL_CD_RATE_SAFE_VAL;
	}
	
//	//ʹ�ú�����10Hz��
//	//if(xTaskGetTickCount() % (1000 / shoot_freq) == 0) //1000Ϊtick++��Ƶ��
//	if( get_para_hz_time_freq_signal_HAL(10) ) //10Hz (shoot_control.local_heat > 0) && 
//	{
//		 shoot_heat->local_heat -= (fp32)((fp32)shoot_heat->local_cd_rate / 10.0f);
//		 if(shoot_heat->local_heat < 0.0f)
//		 {
//			 shoot_heat->local_heat = 0.0f;
//		 }
//		 
//		 shoot_control.temp_debug++;
//	}
//	//----section end----
	
//	//ʹ��timestamp��
//	shoot_heat->local_heat -= ( ((fp32) (xTaskGetTickCount() - shoot_control.local_last_cd_timestamp)) / ((fp32) Tick_INCREASE_FREQ_FREE_RTOS_BASED) * (fp32)shoot_heat->local_cd_rate );
//	if(shoot_heat->local_heat < 0.0f)
//	{
//		shoot_heat->local_heat = 0.0f;
//	}
//		 
//	shoot_control.temp_debug += ((fp32) (xTaskGetTickCount() - shoot_control.local_last_cd_timestamp)) / ((fp32) Tick_INCREASE_FREQ_FREE_RTOS_BASED);
//		 
//	//����ʱ���
//	shoot_control.local_last_cd_timestamp = xTaskGetTickCount();
//	
//	//�ںϲ���ϵͳ��heat��Ϣ, �������صļ��� --TODO ������
//	if( abs( ((int32_t)shoot_control.local_last_cd_timestamp) - ((int32_t)get_last_robot_state_rx_timestamp()) ) > 200 )
//	{
//		shoot_heat->local_heat = shoot_heat->heat;
//	}
//	
//	//local heat�޶�
//	shoot_heat->local_heat = loop_fp32_constrain(shoot_heat->local_heat, MIN_LOCAL_HEAT, (fp32)shoot_heat->local_heat_limit*2.0f); //MAX_LOCAL_HEAT); //(fp32)shoot_heat->local_heat_limit
//	
//	//----section end----
	
	//�ú���10Hz + ��̼���Ϣ��
	//�������Ӽ���
	if( get_para_hz_time_freq_signal_HAL(10) )
	{
		/*����������ϵ�ʱ, Ҳ���ǵ���������ϵ�ʱ, ������������, ֻ������ȴ*/
		if(shoot_control.trigger_motor_17mm_is_online)
		{ //�������δ�ϵ�
#if TRIG_MOTOR_TURN
			shoot_heat->rt_odom_angle = -(get_R_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE;
//		shoot_heat->rt_odom_angle = -(shoot_heat->angle);
#else
			shoot_heat->rt_odom_angle = (get_R_barrel_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE; //TODO ��̼� ��ʼֵ�Ǹ��� - �ų�����
//		shoot_heat->rt_odom_angle = (shoot_heat->angle);
#endif

//		shoot_heat->rt_odom_local_heat = (fp32)(shoot_heat->rt_odom_angle - shoot_heat->last_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC) * ONE17mm_BULLET_HEAT_AMOUNT; //��������
	
			//�õ�ǰ����������������
			shoot_heat->rt_odom_total_bullets_fired = ((fp32)shoot_heat->rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC);
			shoot_heat->rt_odom_local_heat[0] += (fp32)abs( ((int32_t)shoot_heat->rt_odom_total_bullets_fired) - ((int32_t)shoot_heat->rt_odom_calculated_bullets_fired) ) * (fp32)ONE17mm_BULLET_HEAT_AMOUNT;
			
			//update last
			shoot_heat->rt_odom_calculated_bullets_fired = shoot_heat->rt_odom_total_bullets_fired;
			shoot_heat->last_rt_odom_angle = shoot_heat->rt_odom_angle;
		}
		else
		{ //��������ϵ� - TODO �Ƿ��һ��ʱ���ϵĻ���
			shoot_heat->rt_odom_calculated_bullets_fired = shoot_heat->rt_odom_total_bullets_fired;
			shoot_heat->last_rt_odom_angle = shoot_heat->rt_odom_angle;
		}
		
		//��ȴ
		shoot_heat->rt_odom_local_heat[0] -= (fp32)((fp32)shoot_heat->local_cd_rate / 10.0f);
		if(shoot_heat->rt_odom_local_heat[0] < 0.0f)
		{
			shoot_heat->rt_odom_local_heat[0] = 0.0f;
		}
			 
		shoot_control.temp_debug += ((fp32) (xTaskGetTickCount() - shoot_control.local_last_cd_timestamp)) / ((fp32) Tick_INCREASE_FREQ_FREE_RTOS_BASED);
			 
		//����ʱ���
		shoot_control.local_last_cd_timestamp = xTaskGetTickCount();
		
		//�ںϲ���ϵͳ��heat��Ϣ, �������صļ��� --TODO ������
//		if( abs( ((int32_t)shoot_control.local_last_cd_timestamp) - ((int32_t)get_last_robot_state_rx_timestamp()) ) > 200 )
//		{
//			shoot_heat->rt_odom_local_heat = shoot_heat->heat;
//		}
//		 fp32 delta_heat = shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat); // fabs(shoot_heat->rt_odom_local_heat[3] - ((fp32)shoot_heat->heat));
//		 if(delta_heat > 12.0f) //���һ�������� fabs(delta_heat) 
//		 {
//			 shoot_heat->rt_odom_local_heat[0] -= delta_heat;
//		 }
		
		//local heat�޶�
		shoot_heat->rt_odom_local_heat[0] = fp32_constrain(shoot_heat->rt_odom_local_heat[0], MIN_LOCAL_HEAT, (fp32)shoot_heat->local_heat_limit*2.0f); //MAX_LOCAL_HEAT); //(fp32)shoot_heat->local_heat_limit
		
		//���ȥ��
		shoot_heat->rt_odom_local_heat[3] = shoot_heat->rt_odom_local_heat[2];
		shoot_heat->rt_odom_local_heat[2] = shoot_heat->rt_odom_local_heat[1];
		shoot_heat->rt_odom_local_heat[1] = shoot_heat->rt_odom_local_heat[0];
		//----section end----
	}
	
//	//��timestamp + ��̼���Ϣ�� 6-3-2023δ�Թ�
//	//�������Ӽ���
//#if TRIG_MOTOR_TURN
//		shoot_heat->rt_odom_angle = -(get_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE;
////		shoot_heat->rt_odom_angle = -(shoot_heat->angle);
//#else
//		shoot_heat->rt_odom_angle = (get_trig_modor_odom_count()) * MOTOR_ECD_TO_ANGLE;
////		shoot_heat->rt_odom_angle = (shoot_heat->angle);
//#endif

//	shoot_heat->rt_odom_local_heat = (fp32)(shoot_heat->rt_odom_angle - shoot_heat->last_rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC) * ONE17mm_BULLET_HEAT_AMOUNT; //��������
//	
//	//�õ�ǰ����������������
//	shoot_heat->rt_odom_total_bullets_fired = ((fp32)shoot_heat->rt_odom_angle) / ((fp32)RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC);
//	shoot_heat->rt_odom_local_heat = abs( ((int32_t)shoot_heat->rt_odom_total_bullets_fired) - ((int32_t)shoot_heat->rt_odom_calculated_bullets_fired) ) * ONE17mm_BULLET_HEAT_AMOUNT;
//	
//	//update last
//	shoot_heat->rt_odom_calculated_bullets_fired = shoot_heat->rt_odom_total_bullets_fired;
//	shoot_heat->last_rt_odom_angle = shoot_heat->rt_odom_angle;
//	
//	//��ȴ
//	shoot_heat->rt_odom_local_heat -= ( ((fp32) (xTaskGetTickCount() - shoot_control.local_last_cd_timestamp)) / ((fp32) Tick_INCREASE_FREQ_FREE_RTOS_BASED) * (fp32)shoot_heat->local_cd_rate );
//	if(shoot_heat->rt_odom_local_heat < 0.0f)
//	{
//		shoot_heat->rt_odom_local_heat = 0.0f;
//	}
//		 
//	shoot_control.temp_debug += ((fp32) (xTaskGetTickCount() - shoot_control.local_last_cd_timestamp)) / ((fp32) Tick_INCREASE_FREQ_FREE_RTOS_BASED);
//		 
//	//����ʱ���
//	shoot_control.local_last_cd_timestamp = xTaskGetTickCount();
//	
//	//local heat�޶�
//	shoot_heat->rt_odom_local_heat = loop_fp32_constrain(shoot_heat->rt_odom_local_heat, MIN_LOCAL_HEAT, (fp32)shoot_heat->local_heat_limit*2.0f); //MAX_LOCAL_HEAT); //(fp32)shoot_heat->local_heat_limit
//	//----section end----
	
	return 0;
}

