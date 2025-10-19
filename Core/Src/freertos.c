/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


double brightNess;//是小灯的亮度�?1�?�? 0�?�?

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task_main */
osThreadId_t Task_mainHandle;
const osThreadAttr_t Task_main_attributes = {
  .name = "Task_main",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Scan */
osThreadId_t ScanHandle;
const osThreadAttr_t Scan_attributes = {
  .name = "Scan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for binarySem */
osSemaphoreId_t binarySemHandle;
const osSemaphoreAttr_t binarySem_attributes = {
  .name = "binarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Task_main_start(void *argument);
void vTaskScan(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of binarySem */
  binarySemHandle = osSemaphoreNew(1, 1, &binarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_main */
  Task_mainHandle = osThreadNew(Task_main_start, NULL, &Task_main_attributes);

  /* creation of Scan */
  ScanHandle = osThreadNew(vTaskScan, NULL, &Scan_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Task_main_start */
/**
  * @brief  Function implementing the Task_main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_main_start */
void Task_main_start(void *argument)
{
  /* USER CODE BEGIN Task_main_start */
  /* Infinite loop */
  for(;;)
  {

  		for(uint8_t breath_val = 0 ;breath_val<100 ;breath_val++)
    {
      osDelay(60);
      brightNess = sin(breath_val * (3.14159265/100));
    }
      osDelay(1);
  }
  /* USER CODE END Task_main_start */
}

/* USER CODE BEGIN Header_vTaskScan */
/**
* @brief Function implementing the Scan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskScan */
void vTaskScan(void *argument)
{
  /* USER CODE BEGIN vTaskScan */
//	uint8_t key_val = 0;
//	uint16_t row_pins[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};  //行线数组
	
	
  /* Infinite loop */
  for(;;)
  {
		// 阻塞等待信号量（无限等待
		if(osSemaphoreAcquire(binarySemHandle, osWaitForever) == osOK )
		{
				for(uint8_t row = 0; row< 4; row++ )
				{
						//动�?�设置当前行高电平，其他行低电平 
						//Set the current row to a high level while keeping the others at a low level.
						
//						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET); // 所有行拉低
//						HAL_GPIO_WritePin(GPIOA, row_pins[row], GPIO_PIN_SET);	 // 仅当前行拉高  // Only when the current row rises
						
					
						// 信号量触发，弿始扫揿
						if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)
						{
								HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
								break;
						}			
						
				}
			

		}	
    osDelay(1);
  }
  /* USER CODE END vTaskScan */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

