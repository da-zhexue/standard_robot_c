#include "task_init.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "robot_config.h"
osThreadId chassis_taskHandle;
osThreadId radar_taskHandle;
void task_init()
{
#ifdef CHASSIS
    osThreadDef(ChassisTask, chassis_task, osPriorityNormal, 0, 256);
    chassis_taskHandle = osThreadCreate(osThread(ChassisTask), NULL);
#endif

}