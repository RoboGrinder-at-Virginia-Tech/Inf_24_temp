1-8-2023 修改: 
修改AHRS.h AHRS.lib文件(之前一直在RG嵌入式代码中使用)
增加MahonyAHRS.c 和 MahonyAHRS.h使得 AHRS_update(...) get_angle(...) 不再是库函数中的了, 可以访问源码

AHRS.h AHRS.lib修改:
不使用库中的 AHRS_update(...) get_angle(...) +
其它的get_pitch get_roll get_yaw还希望可以继续使用

INS_task.c .h修改:
为了使用可访问源码的AHRS_update(...) get_angle(...), 将其重命名为:
AHRS_init_ins(....)
AHRS_update_ins(....)
get_angle_ins(....)