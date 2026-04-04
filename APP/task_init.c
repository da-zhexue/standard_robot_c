#include "task_init.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "gimbal.h"
#include "daemon.h"
#include "robot_config.h"
osThreadId chassis_taskHandle;
osThreadId gimbal_taskHandle;
osThreadId daemon_taskHandle;
void task_init()
{
#ifdef CHASSIS
    osThreadDef(ChassisTask, Chassis_Task, osPriorityNormal, 0, 256);
    chassis_taskHandle = osThreadCreate(osThread(ChassisTask), NULL);
#endif

#ifdef GIMBAL
    osThreadDef(GimbalTask, Gimbal_Task, osPriorityNormal, 0, 256);
    gimbal_taskHandle = osThreadCreate(osThread(GimbalTask), NULL);
#endif

    osThreadDef(DaemonTask, Daemon_Task, osPriorityLow, 0, 128);
    daemon_taskHandle = osThreadCreate(osThread(DaemonTask), NULL);

}