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
#define PRESS_LONG_TIME_L						999

//鼠标右键 长按 
#define PRESS_LONG_TIME_R						50

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
#define FULL_COUNT                  18
//拨弹速度
#define TRIGGER_SPEED               10.0f
#define CONTINUE_TRIGGER_SPEED      9.0f//12.0f//9.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         0.88f //0.5f //之前的参数是1.0f
#define BLOCK_TIME                  400 //700
#define REVERSE_TIME                200 //500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
/*
Angle calculations for different robot <-> SZL 5-19-2022
弧度制, 范围 (0,2PI], 注意这与 (-PI,PI] 的相位差 不同

Infantry; 拨盘有9个洞, 2pi/9 = 0.698131701f; 为了保证不过冲 set 0.67f
0.57f

Hero; 拨盘3个洞, 2pi/3 = 2.094395102f; 为了保证不过冲 set = 2.05f

测试用旋转角度180度, 2pi/2 = pi = 3.1415926f; 
1.5PI = 4.712388980f
2.0PI = 6.283185307f
*/
#define PI_TEN                      0.60f//0.67f
//2.05f//3.1415926f//0.67f//0.698131701f//3.1415926f//2.094395102f//0.69f//initial 0.314 radian,0.69 is approximately 40 degree

/*仿照云台控制逻辑 新增一个宏定义 电机和转盘安装方向*/
#define TRIG_MOTOR_TURN 0

//用于PID位置环后, 连续发弹 - 射频; 每秒打这么多课
#define CONTINUE_SHOOT_TRIG_FREQ 5

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
//拨弹轮电机PID 外环PID
#define TRIGGER_ANGLE_PID_OUTER_KP        50.0f //40.0f //50.0 //30.0f //25.0f
#define TRIGGER_ANGLE_PID_OUTER_KI        0.0f
#define TRIGGER_ANGLE_PID_OUTER_KD        5.5f

#define TRIGGER_BULLET_PID_OUTER_MAX_OUT  30.0f //10.0f
#define TRIGGER_BULLET_PID_OUTER_MAX_IOUT 2.0f//1.5f //1.0f
/*
外环的输出是内环的输入 内环输入单位是rad/s 
*/
//拨弹轮电机PID  这个是速度环的PID - 600 or 800Kp
#define TRIGGER_SPEED_IN_PID_KP        650.0f //800.0f//100.0f//800.0f//600//800.0f TRIGGER_ANGLE_PID_KP
#define TRIGGER_SPEED_IN_PID_KI        0.25f //0.5f//1.0//0.5f TRIGGER_ANGLE_PID_KI
#define TRIGGER_SPEED_IN_PID_KD        0.1f //TRIGGER_ANGLE_PID_KD

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f//9000.0f 

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  5000.0f//7000.0f

/*直接 - 裁判系统 原始值是#define SHOOT_HEAT_REMAIN_VALUE     30*/
#define SHOOT_HEAT_REMAIN_VALUE     10 //30 //50 //30//50//60//5-24之前:40//30: 1v1参数30; 3v3参数{20}

/* 其它热量相关宏定义 - 本地计算热量 */
#define ONE17mm_BULLET_HEAT_AMOUNT 10
#define MIN_LOCAL_HEAT 0
#define MAX_LOCAL_HEAT 500
#define LOCAL_SHOOT_HEAT_REMAIN_VALUE 10 //20 //5 1v1参数20; 3v3参数{10}
/*2022 infantry; 拨盘有9个洞, 2pi/9 = 0.698131701f; 为了保证不过冲发弹set 0.67f*/
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
    SHOOT_DONE,          //6
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
	user_SHOOT_AUTO, //1
	user_SHOOT_SEMI, //2
}user_fire_ctrl_e;

typedef struct
{
	  uint8_t trigger_motor_17mm_is_online;//0x01=online; 0x00=offline
	
    shoot_mode_e shoot_mode;
		//SZL 6-10-2022新增
		uint8_t last_key_Q_sts; //0未按下, 1按下
		uint8_t key_Q_cnt;
	
		uint8_t last_key_X_sts;
		uint8_t key_X_cnt;
		uint16_t press_key_X_time;//只是定义了 未使用
	
		uint8_t last_key_V_sts;
		uint8_t key_V_cnt;
		uint16_t press_key_V_time; 
	
		user_fire_ctrl_e user_fire_ctrl;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
//    pid_type_def trigger_motor_pid;//内环PID
//		pid_type_def trigger_motor_angle_pid;//外环PID--只是写在这里 没用
		shoot_pid_t trigger_motor_pid;//17mm拨盘电机 内环PID
		shoot_pid_t trigger_motor_angle_pid;//17mm拨盘电机 外环PID
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count; //未使用

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;
		uint8_t block_flag;//17mm堵转标志位
		uint8_t last_block_flag;

    bool_t key; //微动开关 PR 屏蔽掉了
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
		
		/*12-28-2021 SZL add for 
		infantry pid shooter friction wheel LEFT and RIGHT
		Everything above keep the same as the old PWM shooter
		*/
		const motor_measure_t *left_friction_motor_measure;
		const motor_measure_t *right_friction_motor_measure;
		pid_type_def left_fric_motor_pid;
		pid_type_def right_fric_motor_pid;
		
		//LEFT and RIGHT
		M3508_fric_motor_t left_fricMotor;
		M3508_fric_motor_t right_fricMotor;
		
		fp32 currentLeft_speed_set;
		fp32 currentRight_speed_set;
		
		fp32 currentLIM_shoot_speed_17mm;
		//当前 摩擦轮PID速度环 输入; 当前规则允许 速度上限 - offset 后 = 这个数
		//所以 上面这个数 + offset = 预计速度
		
		fp32 predict_shoot_speed;//for CV
		
		uint16_t referee_current_shooter_17mm_speed_limit;
		
		uint8_t ammoBox_sts;
		
		uint32_t total_bullets_fired; // 总发弹量 -主要用于debug --没用
		uint16_t local_heat_limit; //用于当前 本地计算的热量上线
		uint16_t local_cd_rate; //用于当前 本地计算的冷却数值 率
    fp32 local_heat; //本地热量未使用 实时里程计 只是开发时的一个测试未移植到其他机器人 --没用
		fp32 temp_debug; //--没用
		uint8_t local_heat_protection_trig; //触发了本地过热保护 --没用
		
		uint32_t local_last_cd_timestamp; //上一次冷却的time stamp
		
		//实时里程计 - 6-1-2023再次尝试
		fp32 rt_odom_angle; //当前时刻 里程计 角度
		fp32 last_rt_odom_angle; //上一时刻里程计角度
		
		uint32_t rt_odom_total_bullets_fired; // 总的发弹量
		uint32_t rt_odom_calculated_bullets_fired; // 已经计算过热量的子弹量

		fp32 rt_odom_local_heat[4]; //本地热量 [0] 当前 [1]上一次 [2]上上次 受到射频影响		
} shoot_control_t;

//shoot motor 是 拨弹轮 M2006 motor


extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

extern const shoot_control_t* get_robot_shoot_control(void);

extern shoot_mode_e get_shoot_mode(void);
extern user_fire_ctrl_e get_user_fire_ctrl(void);
extern uint8_t get_ammoBox_sts(void);
extern uint32_t shoot_heat_update_calculate(shoot_control_t* shoot_heat);
#endif
