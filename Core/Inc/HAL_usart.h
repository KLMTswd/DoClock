#ifndef __HAL_USART_H__
#define __HAL_USART_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f1xx_hal.h"


HAL_StatusTypeDef HAL_usart_init(UART_HandleTypeDef *huart);
void HAL_usart_send(UART_HandleTypeDef *huart, char *data);

#ifdef __cplusplus
}

#endif
#endif /* __HAL_USART_H__ */

