
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../common.h"

void TaskOne(void *argument);
void TaskTwo(void *argument);
void TaskThree(void *argument);

osThreadId_t TaskOne_Task_Handle;
osThreadId_t TaskTwo_Task_Handle;
osThreadId_t TaskThree_Task_Handle;
osSemaphoreId_t CalcSemaphore;


#ifdef __cplusplus
}
#endif



#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
