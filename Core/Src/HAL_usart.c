#include "HAL_usart.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "usart.h"


static UART_HandleTypeDef *g_huart = NULL;

/* USER CODE BEGIN 1 */

/* USART init function */
/**
 * @brief  USART初始化函数
 * @param  huart - UART句柄指针，指向已配置好的UART_HandleTypeDef结构体
 * @retval 无
 * @note   该函数用于初始化串口通信模块，将用户提供的UART句柄保存到全局变量中，
 *         供后续的串口发送函数HAL_usart_send使用
 */
void HAL_usart_init(UART_HandleTypeDef *huart)
{
    g_huart = huart;  // 将传入的UART句柄赋值给全局变量g_huart
}

/* USER CODE END 1 */




/**
 * @brief  USART字符串发送函数
 * @param  str - 指向要发送的字符串的指针
 * @retval 无
 * @note   该函数用于通过使用串口发送字符串数据，在发送前会检查串口是否已初始化
 *         函数会自动计算字符串长度，并设置100毫秒的发送超时时间
 */
void HAL_usart_send(char *str)
{
    // 检查UART句柄是否已初始化（不为空）
    if (g_huart != NULL)
    {
       // 调用HAL库函数发送数据：参数依次为UART句柄、数据指针(转换为uint8_t*)、数据长度、超时时间(ms)
        HAL_UART_Transmit(g_huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
    }
    // 如果UART句柄为空，则不执行任何操作，避免空指针异常
}

/* USER CODE END 2 */


