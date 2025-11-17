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
#include "HAL_usart.h"
#include "usart.h"
#include "esp8266.h"
#include "onenet.h"
#include "Mqttkit.h"
#include "stdio.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


double brightNess;    //鏄皬鐏殑浜害锛?1鏈?澶? 0鏈?灏?
extern char PUBLIS_BUF[256];
const char devPubTopic[] = "$sys/nw8lCCjUcu/System/thing/property/post";
uint16_t TimeCount = 0;

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
/* Definitions for usartTask */
osThreadId_t usartTaskHandle;
const osThreadAttr_t usartTask_attributes = {
  .name = "usartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Onenet */
osThreadId_t OnenetHandle;
const osThreadAttr_t Onenet_attributes = {
  .name = "Onenet",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
void vTaskusartTask(void *argument);
void vTaskOnenet(void *argument);

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

  /* creation of usartTask */
  usartTaskHandle = osThreadNew(vTaskusartTask, NULL, &usartTask_attributes);

  /* creation of Onenet */
  OnenetHandle = osThreadNew(vTaskOnenet, NULL, &Onenet_attributes);

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
  /* Infinite loop */
  for(;;)
  {
		
    // 尝试获取二进制信号量，等待时间为无限等待
    if(osSemaphoreAcquire(binarySemHandle, osWaitForever) == osOK )
    {
       
   			// 信号量获取成功，开始扫描流程
        // 读取GPIOA的PIN_1引脚状态
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
        {
            // 如果引脚为高电平，将LED引脚设置为低电平(点亮LED)
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        // 注意：如果引脚不为高电平，此处没有对应的处理逻辑
    }
		
		
    // 延时1毫秒，降低CPU使用率
    osDelay(1);
  }
  /* USER CODE END vTaskScan */
}

/* USER CODE BEGIN Header_vTaskusartTask */
/**
* @brief Function implementing the usartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskusartTask */
void vTaskusartTask(void *argument)
{
  /* USER CODE BEGIN vTaskusartTask */


  /* Infinite loop */
  for(;;)
  {
  
		osDelay(1);
  }

  /* USER CODE END vTaskusartTask */
}

/* USER CODE BEGIN Header_vTaskOnenet */
/**
* @brief Function implementing the Onenet thread.
* @param argument: Not used

* @retval None
*/
/* USER CODE END Header_vTaskOnenet */
void vTaskOnenet(void *argument)
{
  /* USER CODE BEGIN vTaskOnenet */
	
// 在任务开始处 - 添加栈监控代码 
  unsigned int uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  char stack_info[64];
  uint32_t lastPublishTime = 0;
  uint32_t lastPingTime = 0;
  uint8_t connected = 0;
  uint8_t publishErrorCount = 0; // 添加错误计数
  uint32_t lastReconnectTime = 0; // 添加重连时间记录

  HAL_usart_send(&huart3, "网络连接中\r\n");
  ESP8266_Init();
  HAL_Delay(1000);
  
// OneNet平台连接重试循环
  while(OneNet_DevLink())
  {
    // 连接失败后，延时500毫秒再次尝试连接
      osDelay(500); 

    // 检查栈使用情况
      uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      sprintf(stack_info, "连接中栈剩余: %u\r\n", uxHighWaterMark);
      HAL_usart_send(&huart3, stack_info);
  }

  HAL_usart_send(&huart3, "网络连接成功\r\n");

// 连接成功后再次检查栈使用情况
  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  sprintf(stack_info, "连接成功后栈剩余: %u\r\n", uxHighWaterMark);
  HAL_usart_send(&huart3, stack_info);

  connected = 1;
  lastReconnectTime = HAL_GetTick();
  osDelay(3000);

  /* Infinite loop */
  for(;;)
  {
    /* 检查连接状态 */
      if(!connected)
      {
          HAL_usart_send(&huart3, "重连中...\r\n");
          
        // 1. 完全重置ESP8266，比单纯清除缓冲区更彻底
          ESP8266_Init(); // 重新初始化ESP8266
          HAL_Delay(2000); // 给ESP8266足够的重启时间
          
        // 2. 添加重连超时保护
          uint8_t reconnectAttempts = 0;
          uint8_t maxAttempts = 5;
          
        // 这里具体动用了OneNet_DevLink()进行连接
          while(OneNet_DevLink() && reconnectAttempts < maxAttempts)
          {
              reconnectAttempts++;
              HAL_usart_send(&huart3, "重连尝试...\r\n");
              osDelay(1000); // 延长重连间隔，给更多恢复时间
						
          }
          
        // 3. 检查是否重连成功
          if(reconnectAttempts < maxAttempts)
          {
              HAL_usart_send(&huart3, "重连成功\r\n");
              connected = 1;
              lastPingTime = 0;
              publishErrorCount = 0;
              lastReconnectTime = HAL_GetTick();
						
          }
          
          else
          {
              HAL_usart_send(&huart3, "重连失败，稍后重试\r\n");
            // 重连失败后等待更长时间再重试，确保一定连上
              osDelay(5000); 
						
          }
      }    
    
    /* 每10秒发送一次心跳包 */
      if(HAL_GetTick() - lastPingTime >= 10000)
      {
        // 确保心跳包功能启用
          OneNet_Ping();
          lastPingTime = HAL_GetTick();
				
      }    
    
    /* 每5s进一次这个逻辑 */
      if(HAL_GetTick() - lastPublishTime >= 5000)
      {
          JsonValue(); 
          OneNet_Publish(devPubTopic, PUBLIS_BUF); // 发布数据
          
        // 4. 调整重连策略：降低重连频率
          publishErrorCount++;
          
        // 5. 仅在长时间无重连时触发定期重连，避免过于频繁
          if((HAL_GetTick() - lastReconnectTime >= 30000)) // 30秒
          {
              HAL_usart_send(&huart3, "定期重连...\r\n");
              connected = 0; // 触发重连
						
          }

        // 6. 只有在心跳包也失效的情况下才基于错误计数重连
          else if(publishErrorCount >= 60) // 约5分钟的发布次数
          {
              HAL_usart_send(&huart3, "发布异常，尝试重连...\r\n");
              connected = 0;
						
          }
          
          ESP8266_Clear();
          lastPublishTime = HAL_GetTick();
      }						   
    
    osDelay(100);
  }
  /* USER CODE END vTaskOnenet */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
