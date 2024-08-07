/*
整个协议的组成为:
frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,整包校验)
*/
#ifndef __REFEREE_UI__
#define __REFEREE_UI__

#define Robot_ID UI_Data_RobotID_BStandard1 //UI_Data_RobotID_RHero //UI_Data_RobotID_BStandard1 //UI_Data_RobotID_RStandard1
#define Cilent_ID UI_Data_CilentID_BStandard1 //UI_Data_CilentID_RHero //UI_Data_CilentID_BStandard1        //机器人角色设置

#include "referee_interact_task.h"
#include "stm32f4xx.h"
#include "stdarg.h"
#include "usart.h"
#include "referee_ui_coordinate_info.h"
#include "user_lib.h"
#include "struct_typedef.h"
#include "gimbal_task.h"
#include "detect_task.h"

#pragma pack(1)                           //按1字节对齐

#define NULL 0
#define __FALSE 100

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
//包头中CMD_ID 机器人间交互数据，发送方触发发送，上限 10Hz 
#define UI_CMD_Robo_Exchange 0x0301    
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点型
#define UI_Graph_Int 6          //整形
#define UI_Graph_Char 7         //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1       //默认黄色 白绿橙
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8


typedef unsigned char Uint8_t;
typedef unsigned char U8;



typedef struct
{
   u8 SOF;                    //起始字节,固定0xA5
   u16 Data_Length;           //帧数据长度
   u8 Seq;                    //包序号
   u8 CRC8;                   //CRC8校验值
   u16 CMD_ID;                //命令ID
} UI_Packhead;             //帧头
//frame_header(5-byte)+cmd_id(2-byte)

typedef struct
{
   u16 Data_ID;               //内容ID
   u16 Sender_ID;             //发送者ID
   u16 Receiver_ID;           //接收者ID
} UI_Data_Operate;         //操作定义帧
//<=>ext_student_interactive_header_data_t
//交互数据包括了一个统一的数据段头结构: 内容ID 发送者 接收者的ID和内容数据段

typedef struct
{
   u8 Delete_Operate;         //删除操作
   u8 Layer;                  //删除图层
} UI_Data_Delete;          //删除图层帧
//<=>ext_client_custom _graphic_delete_t 

typedef struct
{ 
   uint8_t graphic_name[3]; 
   uint32_t operate_tpye:3; 
   uint32_t graphic_tpye:3; 
   uint32_t layer:4; 
   uint32_t color:4; 
   uint32_t start_angle:9;
   uint32_t end_angle:9;
   uint32_t width:10; 
   uint32_t start_x:11; 
   uint32_t start_y:11;
   int32_t graph_Float;              //浮点数据
} Float_Data;
/*
SZL 6-8-2022 修改 浮点数据 应为int32_t
*/

typedef struct
{ 
uint8_t graphic_name[3]; 
uint32_t operate_tpye:3; 
uint32_t graphic_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11;
uint32_t radius:10; 
uint32_t end_x:11; 
uint32_t end_y:11;              //图形数据
} Graph_Data;
//<=> graphic_data_struct_t 图像类别的通用 结构体

typedef struct
{
   Graph_Data Graph_Control;
   uint8_t show_Data[30];
} String_Data;                  //打印字符串数据

#pragma pack()

typedef enum
{
		NORM,
		BOOST
}ui_chassis_sts_e;

typedef enum
{
		spinFOLL,
		spinSPIN
}ui_spin_sts_e;

typedef enum
{
		cvOFF,
		cvNORMAL, //新增 cv_gimbal_st只有0-cvOFF或其它-cvNORMAL
		cvAID,
		cvLOCK
}ui_cv_sts_e;

typedef enum
{
		gunOFF,
		gunAUTO,
		gunSEMI
}ui_gun_sts_e;

typedef enum
{
		ammoOFF,
		ammoOPEN
}ui_ammoBox_sts_e;

/*
5-26-2023 设备连接OK是删除该UI; 设备连接ERR时显示
*/
typedef enum
{
		devError = UI_Graph_ADD,
		devOK = UI_Graph_Del,
}ui_dev_connection_sts_e;

typedef struct
{
	Referee_Interactive_info_t* Referee_Interactive_info;
	uint32_t dynamic_ui_send_timestamp;
	uint32_t static_ui_send_timestamp;
	
  float cap_volt;
	float cap_pct;
	float cap_relative_pct;
	
	float enemy_dis;
	float proj_speed_limit;
	
	/*[0] start x; [1] start y; [2] end x; [3] end y*/
	ui_chassis_sts_e ui_chassis_sts;
	uint16_t box_chassis_sts_coord[4];
	
	ui_spin_sts_e ui_spin_sts;
	uint16_t box_spin_sts_coord[4];
	
	ui_cv_sts_e ui_cv_sts;//用户设定的CV status
	uint16_t box_cv_sts_coord[4];
	
	ui_gun_sts_e ui_gun_sts;
	uint16_t box_gun_sts_coord[4];
	
	ui_ammoBox_sts_e ui_ammoBox_sts;
	uint16_t box_ammoBox_sts_coord[4];
	
	// miniPC反馈的status
	ui_cv_sts_e ui_cv_feedback_sts;
	uint16_t box_cv_feedback_sts[4];
	
	uint16_t superCap_line_var_length;
	
	const gimbal_control_t* gimbal_control_ptr;
	fp32 yaw_relative_angle; //= rad
	
	/* matrix element (i, j) is stored at: pData[i*numCols + j] */
	
	//底盘 旋转 指示框 [0] start x; [1] start y; [2] end x; [3] end y
	//初始角度
	fp32 frame_chassis_coord_start_raw[2]; //非向量形式, 按数组存
	mat frame_chassis_coord_start_vec; //2*1向量 底盘
	
	fp32 frame_chassis_coord_end_raw[2]; //非向量形式, 按数组存
	mat frame_chassis_coord_end_vec; //2*1向量 底盘
	
	// new 当前 最新角度
	fp32 new_frame_chassis_coord_start_raw[2]; //非向量形式, 按数组存
	mat new_frame_chassis_coord_start_vec; //2*1向量 底盘
	
	fp32 new_frame_chassis_coord_end_raw[2]; //非向量形式, 按数组存
	mat new_frame_chassis_coord_end_vec; //2*1向量 底盘
	//
	
	fp32 frame_chassis_rotation_matrix_raw[4]; //原始数据 旋转矩阵
	mat chassis_rotation_matrix; //2*2底盘旋转矩阵
	
	uint16_t frame_chassis_coord_final[4];
	
	//灯条
	fp32 bar_chassis_coord_start_raw[2]; //非向量形式, 按数组存
	mat bar_chassis_coord_start_vec; //2*1向量 底盘
	
	fp32 bar_chassis_coord_end_raw[2]; //非向量形式, 按数组存
	mat bar_chassis_coord_end_vec; //2*1向量 底盘
	
	// new 当前 最新角度
	fp32 new_bar_chassis_coord_start_raw[2]; //非向量形式, 按数组存
	mat new_bar_chassis_coord_start_vec; //2*1向量 底盘
	
	fp32 new_bar_chassis_coord_end_raw[2]; //非向量形式, 按数组存
	mat new_bar_chassis_coord_end_vec; //2*1向量 底盘
	//
	
	uint16_t bar_chassis_coord_final[4];
	
	//底盘 对位线计算 左
	uint16_t chassis_drive_pos_line_left_var_startX;
	uint16_t chassis_drive_pos_line_left_var_startY;
	fp32 chassis_drive_pos_line_left_slope_var;
	uint16_t chassis_drive_pos_line_left_var_endX;
	uint16_t chassis_drive_pos_line_left_var_endY;
	//底盘 对位线计算 右
	uint16_t chassis_drive_pos_line_right_var_startX;
	uint16_t chassis_drive_pos_line_right_var_startY;
	fp32 chassis_drive_pos_line_right_slope_var;
	uint16_t chassis_drive_pos_line_right_var_endX;
	uint16_t chassis_drive_pos_line_right_var_endY;
	
	//Error Code 各种设备的错误 代码
	char chassis_error_code[8];
	uint8_t chassis_error_flag; //0 no err; 1 at least one err
	
	char gimbal_error_code[8];
	uint8_t gimbal_error_flag; //0 no err; 1 at least one err
	
	char shoot_error_code[8]; //feeding error code
	uint8_t shoot_error_flag; //0 no err; 1 at least one err
	
	char superCap_error_code[8];
	uint8_t superCap_error_flag; //0 no err; 1 at least one err
	
} interactive_ui_info_t; //动态的UI信息 结构体 对象 属于 Interactive Data的一部分

void UI_Delete(u8 Del_Operate,u8 Del_Layer);
void Line_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y);
int UI_ReFresh(int cnt,...);
int Delete_ReFresh(UI_Data_Delete delete_Data);
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
void Circle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius);
void Rectangle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y);
void Float_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float);
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,char *Char_Data);
int Char_ReFresh(String_Data string_Data);
void Arc_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length);

// 用户UI绘制函数
void UI_init(Referee_Interactive_info_t* Referee_Interactive_info);
void static_UI_func(void);
void some_mode_change_check(void);
void dynamic_UI_func(uint32_t graph_operation);

void set_ui_ammoBox_sts(ui_ammoBox_sts_e set_val);

#endif

