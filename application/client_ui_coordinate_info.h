/*************************************************************

RM自定义UI协议       基于RM2020学生串口通信协议V1.3

UI坐标标定

弗吉尼亚理工 Virginia Tech; RoboGrinder

**************************************************************/

#ifndef __CLIENT_UI_COORDINATE_INFO__
#define __CLIENT_UI_COORDINATE_INFO__

/*坐标 标定*/
#define Center_X 960
#define	Center_Y 540

//中间 关键警告信息
#define Robot_Warning_Msg_Font_Size 30
#define Robot_Warning_Msg_X (Center_X-150)
#define Robot_Warning_Msg_Y 840
#define Robot_Warning_Msg_X_Offset 220
#define Robot_Warning_Msg_Y_Offset 30

#define Robot_Warning_Spin_X Robot_Warning_Msg_X
#define Robot_Warning_Spin_Y Robot_Warning_Msg_Y

#define Robot_Warning_Fric_X Robot_Warning_Spin_X+Robot_Warning_Msg_X_Offset
#define Robot_Warning_Fric_Y Robot_Warning_Spin_Y
//REF掉线了
#define Robot_Warning_REF_X Robot_Warning_Fric_X
#define Robot_Warning_REF_Y Robot_Warning_Fric_Y-Robot_Warning_Msg_Y_Offset

//机器人 对位线 中间考下
//左侧对位线
#define Chassis_Drive_Pos_Line_Left_Start_X 548
#define Chassis_Drive_Pos_Line_Left_Start_Y 140
#define Chassis_Drive_Pos_Line_Left_Slope 1.00f
#define Chassis_Drive_Pos_Line_Left_End_X 720 //停止X 坐标
#define Chassis_Drive_Pos_Line_Left_End_Y 312 //草稿 不直接使用, 使用slope + start 计算
//以左边为准 右边自动标定
#define Chassis_Drive_Pos_Line_Right_Start_X (Center_X - Chassis_Drive_Pos_Line_Left_Start_X + Center_X)
#define Chassis_Drive_Pos_Line_Right_Start_Y Chassis_Drive_Pos_Line_Left_Start_Y
#define Chassis_Drive_Pos_Line_Right_Slope (-1.00f)
#define Chassis_Drive_Pos_Line_Right_End_X (Center_X - Chassis_Drive_Pos_Line_Left_End_X + Center_X) //停止X 坐标
#define Chassis_Drive_Pos_Line_Right_End_Y 312 //草稿 不直接使用, 使用slope + start 计算

/*右下角 小陀螺和底盘 云台Yaw夹角指示器; 框和灯条 其局部坐标系 的具体地址*/
#define Chassis_Frame_Coord_Center_X 1200 //1300 //Center_X
#define Chassis_Frame_Coord_Center_Y 120 //200 //Center_Y

//炮塔 球
#define Turret_Cir_Start_X Chassis_Frame_Coord_Center_X
#define Turret_Cir_Start_Y Chassis_Frame_Coord_Center_Y
#define Turret_Cir_Radius 22
#define Turret_Cir_Pen 3

//枪口 线
#define Gun_Line_Length 120
#define Gun_Line_Start_X Turret_Cir_Start_X
#define Gun_Line_Start_Y Turret_Cir_Start_Y
#define Gun_Line_End_X Gun_Line_Start_X
#define Gun_Line_End_Y (Turret_Cir_Start_Y + Gun_Line_Length)
#define Gun_Line_Pen 3
/* 5-20-2023注释:
动态图层会覆盖静态的, 图层并不是所有情况都有用, 一旦change了, 动态的就会跑到上面, 
得在刷新动态的时候 捆绑的静态一起刷新
*/
//以下信息为 底盘 框 局部的坐标 -----------------------------
#define Chassis_Frame_Width  100 //75 //150
#define Chassis_Frame_Height 134 //100 //200
#define Chassis_Frame_Height_Pen Chassis_Frame_Height

#define Chassis_Frame_Uniform_Shift 67 //50 //90

#define Chassis_Frame_Start_X (-Chassis_Frame_Width/2)
#define Chassis_Frame_Start_Y (-Chassis_Frame_Height/2 + Chassis_Frame_Uniform_Shift)

#define Chassis_Frame_End_X (Chassis_Frame_Width/2) //(-Chassis_Frame_Start_X) //(-Chassis_Frame_Width/2)
#define Chassis_Frame_End_Y (-Chassis_Frame_Height/2 + Chassis_Frame_Uniform_Shift) //Chassis_Frame_Start_Y //(Chassis_Frame_Height/2)
// End 底盘 框 局部坐标

// 灯条 局部坐标
#define Chassis_Frame_Light_Bar_Height_Pen 10
#define Chassis_Frame_Light_Bar_Downward_Shift (Chassis_Frame_Light_Bar_Height_Pen/2)
#define Chassis_Frame_Light_Bar_Start_X (Chassis_Frame_Start_X+5)
#define Chassis_Frame_Light_Bar_Start_Y (Chassis_Frame_Start_Y - (Chassis_Frame_Height/2) - Chassis_Frame_Light_Bar_Downward_Shift)

#define Chassis_Frame_Light_Bar_End_X (Chassis_Frame_End_X-5)
#define Chassis_Frame_Light_Bar_End_Y (Chassis_Frame_Start_Y - (Chassis_Frame_Height/2) - Chassis_Frame_Light_Bar_Downward_Shift)
// End 灯条

/* 中间 靠下方  ------ 超级电容状态 ------   相关位置 */
//静态的框
#define Center_Bottom_SuperCap_Frame_Start_X (Center_X - 200) //960-20
#define Center_Bottom_SuperCap_Frame_Start_Y 230 //200
#define Center_Bottom_SuperCap_Frame_Height 20
#define Center_Bottom_SuperCap_Frame_End_X (Center_X + 200)
#define Center_Bottom_SuperCap_Frame_End_Y (Center_Bottom_SuperCap_Frame_Start_Y - Center_Bottom_SuperCap_Frame_Height)

//移动长度的线 指示容量
#define Center_Bottom_SuperCap_Line_Downward_Shift 10
#define Center_Bottom_SuperCap_Line_Start_X (Center_Bottom_SuperCap_Frame_Start_X) //(Center_X - 38) //960-20
#define Center_Bottom_SuperCap_Line_Start_Y (Center_Bottom_SuperCap_Frame_Start_Y - Center_Bottom_SuperCap_Line_Downward_Shift)  //(150-2) //200
#define Center_Bottom_SuperCap_Line_Width Center_Bottom_SuperCap_Frame_Height
#define Center_Bottom_SuperCap_Line_End_X (Center_Bottom_SuperCap_Frame_End_X)
#define Center_Bottom_SuperCap_Line_Length_Max (Center_Bottom_SuperCap_Line_End_X - Center_Bottom_SuperCap_Line_Start_X)
#define Center_Bottom_SuperCap_Line_End_Y Center_Bottom_SuperCap_Line_Start_Y

//中间 靠下方 电容 volt电压  和 能量 半分比数字 坐标
#define Center_Bottom_SuperCap_VOLT_Font_Size 40
#define Center_Bottom_SuperCap_VOLT_NUM_X_COORD (960-220)
#define Center_Bottom_SuperCap_VOLT_NUM_Y_COORD (Center_Bottom_SuperCap_Frame_Start_Y - 30) //(200-30)
// pct 数字 坐标
#define Center_Bottom_SuperCap_PCT_Font_Size 20
#define Center_Bottom_SuperCap_PCT_NUM_X_COORD (960-220)
#define Center_Bottom_SuperCap_PCT_NUM_Y_COORD (Center_Bottom_SuperCap_Frame_Start_Y - 80)//(250-30)

//右上角 字符串 起始坐标:
#define	TopRight_String_Start_X 1440
#define TopRight_String_Start_Y 840
#define TopRight_static_Y_offset 30
//String未使用了
#define CAP_PCT_X TopRight_String_Start_X //1440
#define CAP_PCT_Y TopRight_String_Start_Y //840
//String未使用了
#define CAP_VOLT_X TopRight_String_Start_X //1440
#define CAP_VOLT_Y (TopRight_String_Start_Y-TopRight_static_Y_offset) //810

//2023 error dcode position
#define ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR 60
//底盘
#define CHASSIS_ERROR_CODE_X TopRight_String_Start_X
#define CHASSIS_ERROR_CODE_Y TopRight_String_Start_Y
//云台
#define GIMBAL_ERROR_CODE_X TopRight_String_Start_X
#define GIMBAL_ERROR_CODE_Y (CHASSIS_ERROR_CODE_Y-TopRight_static_Y_offset)
//发射机构
#define SHOOT_ERROR_CODE_X TopRight_String_Start_X
#define SHOOT_ERROR_CODE_Y (GIMBAL_ERROR_CODE_Y-TopRight_static_Y_offset)
//超级电容
#define SUPERCAP_ERROR_CODE_X TopRight_String_Start_X
#define SUPERCAP_ERROR_CODE_Y (SHOOT_ERROR_CODE_Y-TopRight_static_Y_offset)
//裁判系统
#define REFEREE_ERROR_CODE_X TopRight_String_Start_X
#define REFEREE_ERROR_CODE_Y (SUPERCAP_ERROR_CODE_Y-TopRight_static_Y_offset)


//中间 NOT右上角 传递动态信息的 字符串
#define CHASSIS_INFO_X_offset 90 //40
#define CHASSIS_INFO_Dynamic_Y_offset 40 //40
#define CHASSIS_STS_X (Center_X - CHASSIS_INFO_X_offset) //TopRight_String_Start_X //1440
#define CHASSIS_STS_Y (Center_Bottom_SuperCap_Frame_Start_Y - 50) //(CAP_VOLT_Y-TopRight_dynamic_Y_offset) //770
#define SPIN_STS_X (Center_X - CHASSIS_INFO_X_offset) //TopRight_String_Start_X //1440
#define SPIN_STS_Y (CHASSIS_STS_Y - CHASSIS_INFO_Dynamic_Y_offset) //730

//左上角起始坐标
#define TopLeft_String_Start_X 150
#define TopLeft_String_Start_Y 840

//左上角 传递动态信息的 字符串
#define TopLeft_dynamic_Y_offset 40
#define CV_STS_X TopLeft_String_Start_X //150
#define CV_STS_Y TopLeft_String_Start_Y //840

#define GUN_STS_X TopLeft_String_Start_X //150
#define GUN_STS_Y CV_STS_Y-TopLeft_dynamic_Y_offset //800

#define AmmoBox_cover_STS_X TopLeft_String_Start_X //150
#define AmmoBox_cover_STS_Y GUN_STS_Y-TopLeft_dynamic_Y_offset //760

//左上角 静态信息 字符串
#define Enemy_dis_STS_X TopLeft_String_Start_X //150
#define Enemy_dis_STS_Y AmmoBox_cover_STS_Y-TopLeft_dynamic_Y_offset //720

#define Projectile_speed_lim_STS_X TopLeft_String_Start_X //150
#define Projectile_speed_lim_STS_Y Enemy_dis_STS_Y-TopLeft_dynamic_Y_offset //680


//动态方框 的长和宽 左下角为0,0 向上为正, 向右为正
#define UI_REC_LENGTH 95
#define UI_REC_WIDTH 35

//右上角 动态方框 CHASSIS_STS
#define TopRight_REC_START_offset_X 10 //相比于 要框上去的 字符串的offset
#define TopRight_REC_START_offset_Y 10

//右上角方框起始 未使用
#define TopRight_BOX_START_X 1430
#define TopRight_BOX_START_Y 780
#define TopRight_BOX_END_X TopRight_BOX_START_X+UI_REC_LENGTH //1525
#define TopRight_BOX_END_Y TopRight_BOX_START_Y-UI_REC_WIDTH //745

#define TopRight_REC_on_NORM_START_X CHASSIS_STS_X-TopRight_REC_START_offset_X //1430
#define TopRight_REC_on_NORM_START_Y CHASSIS_STS_Y+TopRight_REC_START_offset_Y //780
#define TopRight_REC_on_NORM_END_X TopRight_REC_on_NORM_START_X+UI_REC_LENGTH //1525
#define TopRight_REC_on_NORM_END_Y TopRight_REC_on_NORM_START_Y-UI_REC_WIDTH //745

#define TopRight_REC_on_BOOST_START_X TopRight_REC_on_NORM_START_X+UI_REC_LENGTH+5 //1525+5 = 1530
#define TopRight_REC_on_BOOST_START_Y TopRight_REC_on_NORM_START_Y //780
#define TopRight_REC_on_BOOST_END_X TopRight_REC_on_BOOST_START_X+UI_REC_LENGTH //1625
#define TopRight_REC_on_BOOST_END_Y TopRight_REC_on_BOOST_START_Y-UI_REC_WIDTH //745

//中间靠下 NOT右上角(尽管是这样命名的) 动态方框 SPIN_STS
#define TopRight_REC_on_FOLL_START_X SPIN_STS_X-TopRight_REC_START_offset_X //1430
#define TopRight_REC_on_FOLL_START_Y SPIN_STS_Y+TopRight_REC_START_offset_Y //740
#define TopRight_REC_on_FOLL_END_X TopRight_REC_on_FOLL_START_X+UI_REC_LENGTH //1525
#define TopRight_REC_on_FOLL_END_Y TopRight_REC_on_FOLL_START_Y-UI_REC_WIDTH //705

#define TopRight_REC_on_SPIN_START_X TopRight_REC_on_FOLL_START_X+UI_REC_LENGTH+5 //1530
#define TopRight_REC_on_SPIN_START_Y TopRight_REC_on_FOLL_START_Y //740
#define TopRight_REC_on_SPIN_END_X TopRight_REC_on_SPIN_START_X+UI_REC_LENGTH //1625
#define TopRight_REC_on_SPIN_END_Y TopRight_REC_on_SPIN_START_Y-UI_REC_WIDTH //705

//最左上角的第一个动态方框 起始位置
#define TopLeft_BOX_START_X 240
#define TopLeft_BOX_START_Y 850
#define TopLeft_BOX_END_X TopLeft_BOX_START_X+UI_REC_LENGTH //335
#define TopLeft_BOX_END_Y TopLeft_BOX_START_Y-UI_REC_WIDTH //815


#define TopLeft_REC_START_offset_X 10 //相比于 要框上去的 字符串的offset 未用
#define TopLeft_REC_START_offset_Y 10
//左上角 动态方框 CVSts_box
#define TopLeft_REC_on_cv_OFF_START_X TopLeft_BOX_START_X //240
#define TopLeft_REC_on_cv_OFF_START_Y TopLeft_BOX_START_Y //850
#define TopLeft_REC_on_cv_OFF_END_X TopLeft_BOX_END_X //335
#define TopLeft_REC_on_cv_OFF_END_Y TopLeft_BOX_END_Y //815

#define TopLeft_REC_on_cv_AID_START_X TopLeft_REC_on_cv_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_cv_AID_START_Y TopLeft_REC_on_cv_OFF_START_Y //850
#define TopLeft_REC_on_cv_AID_END_X TopLeft_REC_on_cv_AID_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_cv_AID_END_Y TopLeft_REC_on_cv_AID_START_Y-UI_REC_WIDTH //815

#define TopLeft_REC_on_cv_LOCK_START_X TopLeft_REC_on_cv_AID_START_X+UI_REC_LENGTH+5 //440
#define TopLeft_REC_on_cv_LOCK_START_Y TopLeft_REC_on_cv_OFF_START_Y //850
#define TopLeft_REC_on_cv_LOCK_END_X TopLeft_REC_on_cv_LOCK_START_X+UI_REC_LENGTH //535
#define TopLeft_REC_on_cv_LOCK_END_Y TopLeft_REC_on_cv_LOCK_START_Y-UI_REC_WIDTH //815
//CV指示球
#define TopLeft_Cir_on_cv_DET_START_X (Center_X - 100) //580
#define TopLeft_Cir_on_cv_DET_START_Y (Center_Y + 50)
#define TopLeft_Cir_on_cv_DET_radius 10 //10
#define TopLeft_Cir_on_cv_DET_Pen_Size 16 //28 //10

#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X TopLeft_REC_on_cv_OFF_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y TopLeft_REC_on_cv_OFF_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X TopLeft_REC_on_cv_OFF_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y TopLeft_REC_on_cv_OFF_END_Y-8

#define TopLeft_CV_FEEDBACK_STATUS_on_AID_START_X TopLeft_REC_on_cv_AID_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_START_Y TopLeft_REC_on_cv_AID_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_END_X TopLeft_REC_on_cv_AID_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_AID_END_Y TopLeft_REC_on_cv_AID_END_Y-8

#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_X TopLeft_REC_on_cv_LOCK_START_X-8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_Y TopLeft_REC_on_cv_LOCK_START_Y+8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_X TopLeft_REC_on_cv_LOCK_END_X+8
#define TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_Y TopLeft_REC_on_cv_LOCK_END_Y-8

//左上角 动态方框 GUNSts_box
#define TopLeft_REC_on_gun_OFF_START_X TopLeft_REC_on_cv_OFF_START_X //240
#define TopLeft_REC_on_gun_OFF_START_Y TopLeft_REC_on_cv_OFF_START_Y-TopLeft_dynamic_Y_offset //810
#define TopLeft_REC_on_gun_OFF_END_X TopLeft_REC_on_gun_OFF_START_X+UI_REC_LENGTH //335
#define TopLeft_REC_on_gun_OFF_END_Y TopLeft_REC_on_gun_OFF_START_Y-UI_REC_WIDTH //810-35 = 775

#define TopLeft_REC_on_gun_SEMI_START_X TopLeft_REC_on_gun_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_gun_SEMI_START_Y TopLeft_REC_on_gun_OFF_START_Y //810
#define TopLeft_REC_on_gun_SEMI_END_X TopLeft_REC_on_gun_SEMI_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_gun_SEMI_END_Y TopLeft_REC_on_gun_SEMI_START_Y-UI_REC_WIDTH //775

#define TopLeft_REC_on_gun_AUTO_START_X TopLeft_REC_on_gun_SEMI_START_X+UI_REC_LENGTH+5 //440
#define TopLeft_REC_on_gun_AUTO_START_Y TopLeft_REC_on_gun_OFF_START_Y //810
#define TopLeft_REC_on_gun_AUTO_END_X TopLeft_REC_on_gun_AUTO_START_X+UI_REC_LENGTH //535
#define TopLeft_REC_on_gun_AUTO_END_Y TopLeft_REC_on_gun_AUTO_START_Y-UI_REC_WIDTH //775

//左上角 动态方框 Ammo Box Cover Sts_box
#define TopLeft_REC_on_ammo_OFF_START_X TopLeft_REC_on_cv_OFF_START_X //240
#define TopLeft_REC_on_ammo_OFF_START_Y TopLeft_REC_on_gun_OFF_START_Y-TopLeft_dynamic_Y_offset //770
#define TopLeft_REC_on_ammo_OFF_END_X TopLeft_REC_on_ammo_OFF_START_X+UI_REC_LENGTH //335
#define TopLeft_REC_on_ammo_OFF_END_Y TopLeft_REC_on_ammo_OFF_START_Y-UI_REC_WIDTH //735

#define TopLeft_REC_on_ammo_OPEN_START_X TopLeft_REC_on_ammo_OFF_START_X+UI_REC_LENGTH+5 //340
#define TopLeft_REC_on_ammo_OPEN_START_Y TopLeft_REC_on_ammo_OFF_START_Y //770
#define TopLeft_REC_on_ammo_OPEN_END_X TopLeft_REC_on_ammo_OPEN_START_X+UI_REC_LENGTH //435
#define TopLeft_REC_on_ammo_OPEN_END_Y TopLeft_REC_on_ammo_OPEN_START_Y-UI_REC_WIDTH // 735

#endif //__CLIENT_UI_COORDINATE_INFO__

