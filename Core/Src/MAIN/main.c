/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "cmsis_os.h"

/**
 * @brief  The application entry point.
 * @retval int
 */

char msg[50];

int main(void)
{
	set();
//	HAL_NVIC_SystemReset();



	const osThreadAttr_t TaskOne_Task_attributes = {
			.name = "TaskOne",
			.stack_size = 256 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};


	const osThreadAttr_t TaskTwo_Task_attributes = {
			.name = "TaskTwo",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t TaskThree_Task_attributes = {
			.name = "TaskThree",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osSemaphoreAttr_t CalcSemaphore_attributes = {
			.name = "CalcSemaphore"
	};

	osKernelInitialize();

	TaskOne_Task_Handle = osThreadNew(TaskOne, NULL, &TaskOne_Task_attributes);
	TaskTwo_Task_Handle = osThreadNew(TaskTwo, NULL, &TaskTwo_Task_attributes);
	TaskThree_Task_Handle = osThreadNew(TaskThree, NULL, &TaskThree_Task_attributes);
	CalcSemaphore = osSemaphoreNew(1, 0, &CalcSemaphore);

	osKernelStart();

	while(1){

	}

}


void TaskOne(void *argument){

	while(1){

//		xr = (-ps4.joyL_x*cos(YawAngleAngle*3.14/180)-ps4.joyL_y*sin(YawAngleAngle*3.14/180));
//		yr = (-ps4.joyL_x*sin(YawAngleAngle*3.14/180)+ps4.joyL_y*cos(YawAngleAngle*3.14/180));

//		xr = -ps4.joyL_x;
//		yr =  ps4.joyL_y;

		if (ps4.button & UP) {

			RNSVelocity(0.75,0.75,0.75,0.75,&rns);
			RNSEnquire(RNS_VEL_BOTH, &rns);
			vel1 = rns.RNS_data.common_buffer[0].data;
			vel2 = rns.RNS_data.common_buffer[1].data;
			vel3 = rns.RNS_data.common_buffer[2].data;
			vel4 = rns.RNS_data.common_buffer[3].data;
			sprintf(debug_message,"%0.2f %0.2f %0.2f %0.2f\n",vel1,vel2,vel3,vel4);
			UARTPrintString(&huart2, debug_message);
			RNSEnquire(RNS_X_Y_IMU_LSA, &rns);
			YawAngle = rns.RNS_data.common_buffer[0].data;

		}

//		else if  (ps4.button & DOWN){
//
//			RNSVelocity(-0.75,-0.75,-0.75,-0.75,&rns);
//		}
//		else{
//
//			RNSStop(&rns);
//		}
		xr = -1.5*(ps4.joyL_x);
		yr = 1.5*(ps4.joyL_y);
		wr=(ps4.joyR_2-ps4.joyL_2);

		sprintf(msg, "%f  %f \n", ps4.joyL_x, ps4.joyL_y);
		UARTPrintString(&huart2, msg);

//		if(ps4.joyL_x || ps4.joyL_y){
//
////			xr = -1 * ((-ps4.joyL_x * cos((YawAngle + 180) * 3.14 / 180)) - (ps4.joyL_y * sin((YawAngle + 180) * 3.14 / 180)));
////			yr = -1 * (( ps4.joyL_y * cos((YawAngle + 180) * 3.14 / 180)) - (ps4.joyL_x * sin((YawAngle + 180) * 3.14 / 180)));
//		}
//		else{
//			RNSStop(&rns);
//		}

		MODN(&Modn);

//		if(fabs(v1) + fabs(v2) + fabs(v3) + fabs(v4) > 0.05){
//
			RNSVelocity(v1, v2, v3, v4, &rns);
//
//		}else{
//
//			RNSStop(&rns);
//		}


		osDelay(1);

		}

}


void TaskTwo(void *argument){

	while(1){

		osSemaphoreAcquire(CalcSemaphore, osWaitForever);
		osDelay(1);

	}
}

void TaskThree(void *argument){

	while(1){



		osDelay(1);

	}

}

void TIM7_IRQHandler(void) { //5ms

	osSemaphoreRelease(CalcSemaphore);
//	PSxConnectionHandler(&ps4);
	HAL_TIM_IRQHandler(&htim7);
}

void TIM6_DAC_IRQHandler(void) //20ms
{
	led1 = !led1;
	HAL_TIM_IRQHandler(&htim6);
}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{


}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics END OF FILE*/
