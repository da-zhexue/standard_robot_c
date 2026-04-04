/* 配置文件，给keil编译使用，使用cmake编译时在CMakeLists里有相同宏定义 */
#ifndef STANDARD_ROBOT_C_ROBOT_CONFIG_H
#define STANDARD_ROBOT_C_ROBOT_CONFIG_H

#ifndef USE_CBOARD_IMU
//#define USE_CBOARD_IMU
#endif

#ifndef CHASSIS
#define CHASSIS
#endif

#ifndef GIMBAL
//#define GIMBAL
#endif

#ifndef OMNI
#define OMNI
#endif

#endif //STANDARD_ROBOT_C_ROBOT_CONFIG_H