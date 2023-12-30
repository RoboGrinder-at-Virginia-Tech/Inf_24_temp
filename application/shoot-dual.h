/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"



//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    200.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断 之前80
#define PRESS_LONG_TIME             999

//SZL添加 给鼠标左键用的 鼠标左键长按
#define PRESS_LONG_TIME_L						200 //999

//鼠标右键 长按 
#define PRESS_LONG_TIME_R						50 //100 //999 //50

//键盘v键长按
#define PRESS_LONG_TIME_V						50

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
//rpm to rad/s
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18 // 未使用
// 拨弹电机 Left barrel 左枪管 相关数据
//拨弹速度
#define TRIGGER_SPEED_L               10.0f //8.0f //10.0f
#define CONTINUE_TRIGGER_SPEED_L      8.0f //10.0f //12.0f//9.0f //SZL3-13 change from 12 to 10
#define READY_TRIGGER_SPEED_L         5.0f

#define KEY_OFF_JUGUE_TIME_L          500
#define SWITCH_TRIGGER_ON_L           0
#define SWITCH_TRIGGER_OFF_L          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED_L         1.0f
#define BLOCK_TIME_L                  700
#define REVERSE_TIME_L                500
#define REVERSE_SPEED_LIMIT_L         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f //未使用

// 拨弹电机 Right barrel 左枪管 相关数据
//拨弹速度
#define TRIGGER_SPEED_R               10.0f //8.0f //10.0f
#define CONTINUE_TRIGGER_SPEED_R      8.0f //10.0f //12.0f//9.0f //SZL3-13 change from 12 to 10
#define READY_TRIGGER_SPEED_R         5.0f

#define KEY_OFF_JUGUE_TIME_R          500
#define SWITCH_TRIGGER_ON_R           0
#define SWITCH_TRIGGER_OFF_R          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED_R         1.0f
#define BLOCK_TIME_R                  700
#define REVERSE_TIME_R                500
#define REVERSE_SPEED_LIMIT_R         13.0f


/*
Angle calculations for different robot <-> SZL 5-19-2022
弧度制, 范围 (0,2PI], 注意这与 (-PI,PI] 的相位差 不同

Infantry; 拨盘有9个洞, 2pi/9 = 0.698131701f; 为了保证不过冲 set 0.67f
0.57f

Hero; 拨盘3个洞, 2pi/3 = 2.094395102f; 为了保证不过冲 set = 2.05f

Omni drive 机器人 炮塔; 拨盘有8个洞, 2pi/8 = 0.78539816339744830961566084581988f

测试用旋转角度180度, 2pi/2 = pi = 3.1415926f; 
1.5PI = 4.712388980f
2.0PI = 6.283185307f
*/
#define PI_TEN_L                      0.70f
#define PI_TEN_R                      0.70f
//2.05f//3.1415926f//0.67f//0.698131701f//3.1415926f//2.094395102f//0.69f//initial 0.314 radian,0.69 is approximately 40 degree

/*仿照云台控制逻辑 新增一个宏定义 电机和转盘安装方向*/
#define TRIG_MOTOR_TURN_LEFT_BARREL 1
#define TRIG_MOTOR_TURN_RIGHT_BARREL 0

/*
SZL
Original PID parameter
#define TRIGGER_ANGLE_PID_KP        800.0f//600//800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f//1.0//0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f
*/
/*6-5-2023 左右枪管 分为两套PID参数*/
//左枪管----------------------------------------------------------------------
//拨弹轮电机PID 外环PID
#define L_BARREL_TRIGGER_ANGLE_PID_OUTER_KP        50.0f //40.0f //50.0 //30.0f //25.0f
#define L_BARREL_TRIGGER_ANGLE_PID_OUTER_KI        0.0f
#define L_BARREL_TRIGGER_ANGLE_PID_OUTER_KD        5.5f

#define L_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_OUT  30.0f //10.0f
#define L_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_IOUT 2.0f//1.5f //1.0f
/*
外环的输出是内环的输入 内环输入单位是rad/s 
*/
//拨弹轮电机PID  这个是速度环的PID - 600 or 800Kp
#define L_BARREL_TRIGGER_SPEED_IN_PID_KP        650.0f //800.0f//100.0f//800.0f//600//800.0f TRIGGER_ANGLE_PID_KP
#define L_BARREL_TRIGGER_SPEED_IN_PID_KI        0.25f //0.5f//1.0//0.5f TRIGGER_ANGLE_PID_KI
#define L_BARREL_TRIGGER_SPEED_IN_PID_KD        0.1f //TRIGGER_ANGLE_PID_KD

#define L_BARREL_TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define L_BARREL_TRIGGER_BULLET_PID_MAX_IOUT 9000.0f//9000.0f 

#define L_BARREL_TRIGGER_READY_PID_MAX_OUT   10000.0f
#define L_BARREL_TRIGGER_READY_PID_MAX_IOUT  5000.0f//7000.0f
//右枪管-----------------------------------------------------------------------
//拨弹轮电机PID 外环PID
#define R_BARREL_TRIGGER_ANGLE_PID_OUTER_KP        50.0f //40.0f //50.0 //30.0f //25.0f
#define R_BARREL_TRIGGER_ANGLE_PID_OUTER_KI        0.0f
#define R_BARREL_TRIGGER_ANGLE_PID_OUTER_KD        5.5f

#define R_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_OUT  30.0f //10.0f
#define R_BARREL_TRIGGER_BULLET_PID_OUTER_MAX_IOUT 2.0f//1.5f //1.0f
/*
外环的输出是内环的输入 内环输入单位是rad/s 
*/
//拨弹轮电机PID  这个是速度环的PID - 600 or 800Kp
#define R_BARREL_TRIGGER_SPEED_IN_PID_KP        650.0f //800.0f//100.0f//800.0f//600//800.0f TRIGGER_ANGLE_PID_KP
#define R_BARREL_TRIGGER_SPEED_IN_PID_KI        0.25f //0.5f//1.0//0.5f TRIGGER_ANGLE_PID_KI
#define R_BARREL_TRIGGER_SPEED_IN_PID_KD        0.1f //TRIGGER_ANGLE_PID_KD

#define R_BARREL_TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define R_BARREL_TRIGGER_BULLET_PID_MAX_IOUT 9000.0f//9000.0f 

#define R_BARREL_TRIGGER_READY_PID_MAX_OUT   10000.0f
#define R_BARREL_TRIGGER_READY_PID_MAX_IOUT  5000.0f//7000.0f
//------------------------------------------------------------------------------

/*直接 - 裁判系统: SHOOT_HEAT_REMAIN_VALUE 需要 <= LOCAL_SHOOT_HEAT_REMAIN_VALUE*/
#define SHOOT_HEAT_REMAIN_VALUE     35 //20 //30 //405-24之前:40//30

/* 其它热量相关宏定义 - 本地计算热量 - 不分左右枪管; 左右枪管共用这些值*/
#define ONE17mm_BULLET_HEAT_AMOUNT 10
#define MIN_LOCAL_HEAT 0
#define MAX_LOCAL_HEAT 500
#define LOCAL_SHOOT_HEAT_REMAIN_VALUE 20 //20 //5
/*2023 dual barrel infantry; 拨盘有9个洞, 2pi/9 = 0.698131701f; 为了保证不过冲发弹set 0.67f*/
#define RAD_ANGLE_FOR_EACH_HOLE_HEAT_CALC 0.698131701f
//Local heat安全值, 裁判系统离线时的安全值 - 2022步兵 冷却模式一级
#define LOCAL_HEAT_LIMIT_SAFE_VAL 50
#define LOCAL_CD_RATE_SAFE_VAL 40

/*
12-28-2021 SZL添加 PID M3508 屁股 shooter 电机 2个
发射方向左Left 右Right两个电机，两套变量+宏定义，一般数值保持一样
左为Can ID 1 右为Can ID2
M3508_RIGHT_FRICTION_PID_MAX_OUT = M3508_LEFT_FRICTION_PID_MAX_OUT = TRIGGER_READY_PID_MAX_OUT 约等于 MAX_MOTOR_CAN_CURRENT 16000.0f
*/
//底盘3508最大can发送电流值 16384-->20A
//#define MAX_MOTOR_CAN_CURRENT 16000.0f

//LEFT
#define M3508_LEFT_FRICTION_PID_KP 800.0f
#define M3508_LEFT_FRICTION_PID_KI 10.0f
#define M3508_LEFT_FRICTION_PID_KD 600.0f 

#define M3508_LEFT_FRICTION_PID_MAX_OUT 10000.0f//10000
#define M3508_LEFT_FRICTION_PID_MAX_IOUT 2000.0f

//RIGHT
#define M3508_RIGHT_FRICTION_PID_KP 800.0f //800 //900
#define M3508_RIGHT_FRICTION_PID_KI 10.0f //10 //20
#define M3508_RIGHT_FRICTION_PID_KD 600.0f //600 //600

#define M3508_RIGHT_FRICTION_PID_MAX_OUT 10000.0f
#define M3508_RIGHT_FRICTION_PID_MAX_IOUT 2000.0f

#define M3508_FRIC_MOTOR_RPM_TO_LINEAR_VETOR_SEN 3.141592654e-3f


//SZL 5-15-2022 referee speed limit
#define INITIAL_PROJECTILE_SPEED_LIMIT_17mm 15

//ICRA 子弹速度上线 为 18m/s
#define ICRA_PROJECTILE_SPEED_LIMIT 18

/*
发射机构 拨弹电机 自己的PID, 需要使用积分分离 阈值取决于设备本身
*/
enum SHOOT_PID_MODE
{
    SHOOT_PID_SEPARATED_INTEGRAL_IN_SPEED = 0, // inner speed loop
		SHOOT_PID_SEPARATED_INTEGRAL_OUT_POS, //outer position loop
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
	
}shoot_pid_t;

#define PID_TRIG_SPEED_INTEGRAL_THRESHOLD 3.0f //2.0f //速度 弧度制

#define PID_TRIG_POSITION_INTEGRAL_THRESHOLD 3.0f //1.0f //角度 弧度制

//PID_DIFFERENTIAL_THRESHOLD 在此积分分离PID中未使用

void shoot_PID_init(shoot_pid_t *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 shoot_PID_calc(shoot_pid_t *pid, fp32 ref, fp32 set);
void shoot_PID_clear(shoot_pid_t *pid);

// --------------------- PID related END ---------------------

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,    //1
    SHOOT_READY_BULLET,  //2
    SHOOT_READY,         //3
    SHOOT_BULLET,        //4
    SHOOT_CONTINUE_BULLET,  //5
		SHOOT_ALTERNATE_CONTINUE_BULLET, //6
    SHOOT_DONE,          //7
} shoot_mode_e;

//SZL 12-30-2021 添加 fric 电机 M3508 数据解包 待打包发送数据 结构体
//fric Wheel
typedef struct
{
		fp32 fricW_speed_set;
	  fp32 fricW_speed;
	
    //fp32 fricW_angle;
    //fp32 fricW_set_angle;
    //int8_t fricW_ecd_count;
		
		int16_t fricW_given_current;
} M3508_fric_motor_t;

typedef enum
{
	user_SHOOT_OFF=0,
	user_SHOOT_AUTO, //1-交替开火
	user_SHOOT_BOTH, //2-同时单发开火
	user_SHOOT_L_CONT, //3-左侧连续开火
	user_SHOOT_R_CONT, //4-右侧连续开火
}user_fire_ctrl_e;

typedef struct
{
		//L - left trigger motor; R - right trigger motor
	  uint8_t trigger_motor17mm_L_is_online;//0x01=online; 0x00=offline
		uint8_t trigger_motor17mm_R_is_online;
	
    shoot_mode_e shoot_mode_L; // left gun shoot mode
		shoot_mode_e shoot_mode_R; // right gun shoot mode
	
	
		//SZL 6-10-2022新增
		uint8_t last_key_Q_sts; //0未按下, 1按下
		uint8_t key_Q_cnt;
	
		uint8_t last_key_X_sts;
		uint8_t key_X_cnt;
		uint16_t press_key_X_time;//只是定义了 未使用
	
		uint8_t last_key_V_sts;
		uint8_t key_V_cnt;
		uint16_t press_key_V_time; 
	
		user_fire_ctrl_e user_fire_ctrl; // 左右枪共用
    const RC_ctrl_t *shoot_rc;
		
    const motor_measure_t *shoot_motor_L_measure; // left trigger
		const motor_measure_t *shoot_motor_R_measure; // right trigger
		
		// 1号对应: 看向前进方向, 左侧发射机构, 上面那个枪管
    ramp_function_source_t L_barrel_fric1_ramp;
    uint16_t L_barrel_fric_pwm1;
    ramp_function_source_t L_barrel_fric2_ramp; // 2号对应: 看向前进方向, 左侧发射机构, 下面那个枪管
    uint16_t L_barrel_fric_pwm2;
		
		// 3号对应: 看向前进方向, 右侧发射机构, 上面那个枪管
    ramp_function_source_t R_barrel_fric3_ramp;
    uint16_t R_barrel_fric_pwm3;
    ramp_function_source_t R_barrel_fric4_ramp; // 4号对应: 看向前进方向, 右侧发射机构, 下面那个枪管
    uint16_t R_barrel_fric_pwm4;
		
		// 17mm left barrel TRIG 各种控制 逻辑相关 --------------
//    pid_type_def L_barrel_trigger_motor_pid;//内环PID
//		pid_type_def L_barrel_trigger_motor_angle_pid;//外环PID--只是写在这里 没用
		shoot_pid_t L_barrel_trigger_motor_pid;//17mm拨盘电机 内环PID
		shoot_pid_t L_barrel_trigger_motor_angle_pid;//17mm拨盘电机 外环PID
    fp32 L_barrel_trigger_speed_set; // 需要用trigger_speed_set 来 update speed_set(PID 用)
    fp32 L_barrel_speed;
    fp32 L_barrel_speed_set;
    fp32 L_barrel_angle;
    fp32 L_barrel_set_angle;
		//发射时间 - 用于控制射频
		uint32_t L_barrel_last_tick;//
    int16_t L_barrel_given_current;
    int8_t L_barrel_ecd_count; //未使用
		
		// 17mm right barrel TRIG 各种控制 逻辑相关 --------------
//		pid_type_def R_barrel_trigger_motor_pid;//内环PID
//		pid_type_def R_barrel_trigger_motor_angle_pid;//外环PID--只是写在这里 没用
		shoot_pid_t R_barrel_trigger_motor_pid;//17mm拨盘电机 内环PID
		shoot_pid_t R_barrel_trigger_motor_angle_pid;//17mm拨盘电机 外环PID
    fp32 R_barrel_trigger_speed_set;
    fp32 R_barrel_speed;
    fp32 R_barrel_speed_set;
    fp32 R_barrel_angle;
    fp32 R_barrel_set_angle;
		//发射时间 - 用于控制射频
		uint32_t R_barrel_last_tick;//
    int16_t R_barrel_given_current;
    int8_t R_barrel_ecd_count; //未使用
		
		//交替发射 时钟 发射时间 - 用于控制射频
		uint32_t L_barrel_alternate_shoot_last_tick;
		uint32_t R_barrel_alternate_shoot_last_tick;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

		// 17mm left barrel control related values --------------
    uint16_t L_barrel_block_time;
    uint16_t L_barrel_reverse_time;
    bool_t L_barrel_move_flag;
		uint8_t L_barrel_block_flag;//左枪管 17mm堵转标志位
		uint8_t L_barrel_last_block_flag;

    bool_t L_barrel_key; //微动开关 PR 屏蔽掉了
    uint8_t L_barrel_key_time;

    uint16_t L_barrel_heat_limit;
    uint16_t L_barrel_heat;
		
		// 17mm right barrel control related values --------------
		uint16_t R_barrel_block_time;
    uint16_t R_barrel_reverse_time;
    bool_t R_barrel_move_flag;
		uint8_t R_barrel_block_flag;//右枪管 17mm堵转标志位
		uint8_t R_barrel_last_block_flag;

    bool_t R_barrel_key; //微动开关 PR 屏蔽掉了
    uint8_t R_barrel_key_time;

    uint16_t R_barrel_heat_limit;
    uint16_t R_barrel_heat;
		
		/*12-28-2021 SZL add for 
		infantry pid shooter friction wheel LEFT and RIGHT
		Everything above keep the same as the old PWM shooter
		*/
//		const motor_measure_t *left_friction_motor_measure; //NOT USED for MD
//		const motor_measure_t *right_friction_motor_measure; //NOT USED for MD
//		pid_type_def left_fric_motor_pid; //NOT USED for MD
//		pid_type_def right_fric_motor_pid; //NOT USED for MD
		
		//LEFT and RIGHT
//		M3508_fric_motor_t left_fricMotor; //NOT USED for MD
//		M3508_fric_motor_t right_fricMotor; //NOT USED for MD
		
//		fp32 currentLeft_speed_set; //NOT USED for MD
//		fp32 currentRight_speed_set; //NOT USED for MD

		// Left barrel speed related value for each fric
		// 1号对应: 看向前进方向, 左侧发射机构, 上面那个枪管
		fp32 L_barrel_fric1_speed_set; // 对应老的 currentLeft_speed_set
		// 2号对应: 看向前进方向, 左侧发射机构, 下面那个枪管
		fp32 L_barrel_fric2_speed_set;
		
		// Right barrel speed related value for each fric
		// 3号对应: 看向前进方向, 右侧发射机构, 上面那个枪管
		fp32 R_barrel_fric3_speed_set;
		// 4号对应: 看向前进方向, 右侧发射机构, 下面那个枪管
		fp32 R_barrel_fric4_speed_set;
		
		fp32 currentLIM_shoot_speed_17mm; // 左右炮塔各自速度上限 应一样
		//当前 摩擦轮PID速度环 输入; 当前规则允许 速度上限 - offset 后 = 这个数
		//所以 上面这个数 + offset = 预计速度
		
		fp32 predict_shoot_speed;//for CV
		
		uint16_t referee_current_shooter_17mm_speed_limit;
		
		uint8_t ammoBox_sts;
		
		//左枪管 相关热量计算
		uint16_t L_barrel_local_heat_limit; //用于当前 本地计算的热量上线
		uint16_t L_barrel_local_cd_rate; //用于当前 本地计算的冷却数值 率
		
		uint32_t L_barrel_local_last_cd_timestamp; //上一次冷却的time stamp
		
		//实时里程计 - 6-1-2023再次尝试
		fp32 L_barrel_rt_odom_angle; //当前时刻 里程计 角度
		fp32 L_barrel_last_rt_odom_angle; //上一时刻里程计角度
		
		uint32_t L_barrel_rt_odom_total_bullets_fired; // 总的发弹量
		uint32_t L_barrel_rt_odom_calculated_bullets_fired; // 已经计算过热量的子弹量

		fp32 L_barrel_rt_odom_local_heat[4]; //本地热量 [0] 当前 [1]上一次 [2]上上次 受到射频影响		
		
		//右枪管 相关热量计算
		uint16_t R_barrel_local_heat_limit; //用于当前 本地计算的热量上线
		uint16_t R_barrel_local_cd_rate; //用于当前 本地计算的冷却数值 率
		
		uint32_t R_barrel_local_last_cd_timestamp; //上一次冷却的time stamp
		
		//实时里程计 - 6-1-2023再次尝试
		fp32 R_barrel_rt_odom_angle; //当前时刻 里程计 角度
		fp32 R_barrel_last_rt_odom_angle; //上一时刻里程计角度
		
		uint32_t R_barrel_rt_odom_total_bullets_fired; // 总的发弹量
		uint32_t R_barrel_rt_odom_calculated_bullets_fired; // 已经计算过热量的子弹量

		fp32 R_barrel_rt_odom_local_heat[4]; //本地热量 [0] 当前 [1]上一次 [2]上上次 受到射频影响	

		//断电+上电 自动重启
		bool_t auto_rst_signal;
		uint32_t rst_m_off_time; //拨弹电机离线时间
		uint32_t rst_on_wait_time; //拨弹电机 -新上线时间
		
		//每个枪管的超热量停止功能
		uint8_t L_barrel_overheat_stop; //0未发生超热量 1发生超热量,停
		uint8_t R_barrel_overheat_stop; //0未发生超热量 1发生超热量,停
		
		
		//动态适应射频 相位差
		uint8_t shoot_freq_set; //目标射频
		uint32_t phase_diff_ms_set; //目标相位差
		uint16_t local_cd_rate_min; //两根枪管最小的冷却值
		uint32_t local_shoot_heat_remain_value_var_set; //冷却预留值 - 动态改变
		
} shoot_control_t;

//shoot motor 是 拨弹轮 M2006 motor


extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

extern const shoot_control_t* get_robot_shoot_control(void);
extern void L_R_barrel_all_fric_esc_pwm_calibration(void);

extern shoot_mode_e get_shoot_mode(void);
extern user_fire_ctrl_e get_user_fire_ctrl(void);
extern uint8_t get_ammoBox_sts(void);
extern uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat);

#endif
