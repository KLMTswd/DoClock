#include "HAL_usart.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "usart.h"

/* USER CODE BEGIN 1 */

/* USART init function */


/**
 * @brief HAL串口功能初始化（轻量级，目前可能没什么用，但保留以防后续需要）

 * @param huart UART句柄指针 

 * @return 初始化结果

 */
HAL_StatusTypeDef HAL_usart_init(UART_HandleTypeDef *huart) 
{
  // 仅进行基本参数验证，不保存全局句柄
    if (huart == NULL || huart->Instance == NULL) 
    {
        return HAL_ERROR;
    }
    
  // 可添加其他验证或初始化操作
    return HAL_OK;
}



/**
 * @brief HAL串口发送函数

 * @param huart UART句柄指针，指定要使用的串口
 
 * @param data 要发送的字符串数据
 
 * @note 此函数自动计算字符串长度并发送，使用阻塞方式传输
 
 * @note 超时时间固定为100ms，适用于大多数应用场景
 
 */
void HAL_usart_send(UART_HandleTypeDef *huart, char *data)
{
  /* 参数有效性检查，避免空指针访问 */
    if (huart != NULL)
    {
      /* 调用HAL库函数发送数据，自动将char*转换为uint8_t*
  
       * 自动计算字符串长度，设置100ms超时时间
  
       */
        HAL_UART_Transmit(huart, (uint8_t*)data, strlen(data), 100);
    
    }
}

/* USER CODE END 2 */

