//单片机头文件
#include "stm32f1xx_hal.h"  // HAL库主头文件


//网络设备驱动
#include "esp8266.h"

//硬件驱动
#include "HAL_usart.h"
#include "usart.h"



//C库
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;


#define ESP8266_WIFI_INFO		"AT+CWJAP=\"Redmi Note 12 Pro\",\"asdfg138\"\r\n"//wifi的账号和密码

#define ESP8266_ONENET_INFO		"AT+CIPSTART=\"TCP\",\"mqtts.heclouds.com\",1883\r\n"


unsigned char esp8266_buf[512];
char PUBLIS_BUF[256];
unsigned short esp8266_cnt = 0, esp8266_cntPre = 0;

extern UART_HandleTypeDef huart2;  // 声明huart2为外部变量，它应该在usart.c中定义
uint8_t aRxBuffer;  // 单字节接收缓冲区，用于HAL_UART_Receive_IT函数

/**
  * @brief  ESP8266接收缓冲区清除函数
  * 
  * @param  无

  * @retval 无

  * @note   该函数用于清除ESP8266的接收缓冲区内容和计数器，
 
	*         通常在发送新命令前调用，确保不会受到之前接收数据的影响
 
  */
void ESP8266_Clear(void)
{
	// 使用memset函数将接收缓冲区所有字节设置为0
    memset(esp8266_buf, 0, sizeof(esp8266_buf));

	// 重置接收计数器为0
    esp8266_cnt = 0;
}


/**
  * @brief  ESP8266等待接收完成函数
  
  * @param  无
 
  * @retval REV_OK: 接收完成
  *         REV_WAIT: 接收未完成或未开始接收
 
  * @note   该函数通过比较当前接收计数与上一次接收计数来判断是否接收完成
 
	*         当连续两次调用时接收计数相同，则认为数据接收完成
 
	*         适用于非中断方式检测ESP8266的响应数据
 
  */
_Bool ESP8266_WaitRecive(void)
{
	if(esp8266_cnt == 0) 							//如果接收计数为0 则说明没有处于接收数据中，所以直接跳出，结束函数
		return REV_WAIT;
		
	if(esp8266_cnt == esp8266_cntPre)				//如果上一次的值和这次相同，则说明接收完毕
	{
		esp8266_cnt = 0;							//清0接收计数
			
		return REV_OK;								//返回接收完成标志
	}
		
	esp8266_cntPre = esp8266_cnt;					//置为相同
	
	return REV_WAIT;								//返回接收未完成标志
}


/**
  * @brief  发送AT命令到ESP8266并等待指定响应

  * @param  cmd: 要发送的AT命令字符串

  * @param  res: 期望接收到的响应关键词

  * @retval _Bool: 0表示成功收到指定响应，1表示超时未收到

  * @note   该函数向ESP8266发送AT命令后，等待指定的响应关键词，超时时间为2秒

  */
_Bool ESP8266_SendCmd(char *cmd, char *res)
{
  /* 设置超时计数，最大等待时间约为200*10ms=2秒 */
    unsigned char timeOut = 200;

  /* 通过USART2向ESP8266模块发送命令 */
    HAL_usart_send(&huart2, (char *)cmd);
    
  /* 循环等待响应，每次检查间隔10ms */
    while(timeOut--)
    {
			
		  // 如果收到数据
        if(ESP8266_WaitRecive() == REV_OK)                      
        {
					// 如果检索到关键词
            if(strstr((const char *)esp8266_buf, res) != NULL)   
            {
							
							// 清空缓存，为下一次通信做准备
                ESP8266_Clear();                               
                
							// 返回0表示成功收到预期响应
                return 0;         							
            }
        }
        
			// 延时10ms，降低CPU占用
        HAL_Delay(10);                                          
    }
    
	// 超时未收到预期响应，返回1	
    return 1;                                                  
}


/**
  * @brief  ESP8266发送数据函数
  
  * @param  data: 要发送的数据缓冲区指针
  
  * @param  len: 要发送的数据长度（字节）
  
  * @retval 无
  
  * @note   该函数用于通过ESP8266模块发送数据，采用AT指令方式
 
	*         先发送AT+CIPSEND命令设置数据长度，然后等待ESP8266响应'>'提示符
 
	*         收到提示符后，再发送实际的数据内容
  
  */
void ESP8266_SendData(unsigned char *data, unsigned short len)
{
  // 用于存储AT命令的缓冲区
    char cmdBuf[150];
    
  // 清空ESP8266接收缓存，防止干扰新的通信过程
    ESP8266_Clear();
    
  // 格式化AT+CIPSEND命令，指定要发送的数据长度
    sprintf(cmdBuf, "AT+CIPSEND=%d\r\n", len);
    
  // 发送AT+CIPSEND命令，并等待ESP8266返回'>'提示符
    if(!ESP8266_SendCmd(cmdBuf, ">"))
    {
			
      // 收到'>'提示符后，通过USART2发送实际的数据内容
       HAL_UART_Transmit(&huart2, data, len, 100);
    }
}

/**
  * @brief  从ESP8266接收缓冲区中提取IPD格式的数据内容
 
  * @param  timeOut: 超时时间(毫秒)
 
  * @retval unsigned char*: 成功返回指向数据内容的指针，失败返回NULL
 
  * @note   ESP8266模块在接收到TCP/UDP数据时，会以"+IPD,长度:数据内容"格式输出
 
	*         此函数负责解析并提取实际的数据内容部分
 
  */
unsigned char *ESP8266_GetIPD(unsigned short timeOut)
{
  /* 定义指针用于定位IPD头信息 */
    char *ptrIPD = NULL;
    ESP8266_Clear();


  /* 在超时时间内循环检测接收缓冲区 */
    do
    { 

			 // 如果接收完成
        if(ESP8266_WaitRecive() == REV_OK)                             
        {
					// 搜索"IPD"头
            ptrIPD = strstr((char *)esp8266_buf, "IPD," );     
					// 如果没找到，可能是IPD头的延迟
            if(ptrIPD == NULL)                                          
            {
							// 调试输出：未找到IPD头
                HAL_usart_send(&huart3, "\"IPD\" not found\r\n");     
            }
            else
            {
							// 找到':'分隔符
                ptrIPD = strchr(ptrIPD, ':');                          
                if(ptrIPD != NULL)
                {
									// 移动指针到数据内容起始位置
                    ptrIPD++;   
									
									// 返回指向数据内容的指针  
                    return (unsigned char *)(ptrIPD);                  
                }
                
								else
									// 格式错误，找不到分隔符
                    return NULL;                                       
                
            }
        }
				
      // 延时5ms后重试
        HAL_Delay(5);  
				
			// 递减超时计数器	
        timeOut--;        
				
    } while(timeOut>0);
    
	// 超时还未找到，返回空指针	
    return NULL;                                                       
}


/**
  * @brief  ESP8266模块初始化函数
  
  * @param  无
 
  * @retval 无
 
  * @note   函数按照固定步骤初始化ESP8266模块，包括：
 
	*         1. 测试AT指令响应
 
	*         2. 设置WiFi模式为Station模式
  
  *         3. 启用DHCP功能
 
	*         4. 连接到指定WiFi网络
  
	*         5. 连接到OneNET服务器
  
	*         每一步都会阻塞等待成功响应，失败则延时后重试

	*         每一步都用到了两个串口，一个用来说明发了什么命令，一个用来和ESP8266通信
            
  */
void ESP8266_Init(void)
{
  // 启动USART2的中断接收，准备接收数据
    HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
    
  /* 清空ESP8266接收缓冲区，准备初始化 */
    ESP8266_Clear();
    
  /* 1. 测试AT指令，确认模块正常工作 */
    HAL_usart_send(&huart3, "1. AT\r\n");
    while(ESP8266_SendCmd("AT\r\n", "OK"))
    osDelay(500);
    
  /* 2. 设置ESP8266为Station模式(模式1) */
    HAL_usart_send(&huart3, "2. CWMODE\r\n");
    while(ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK"))
    HAL_Delay(500);
    
  /* 3. 启用DHCP功能(Station模式下) */
    HAL_usart_send(&huart3, "3. AT+CWDHCP\r\n");
    while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))
    HAL_Delay(500);
    
  /* 4. 连接到指定的WiFi网络，等待获取IP地址 */
    HAL_usart_send(&huart3, "4. CWJAP\r\n");
    while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP"))
    HAL_Delay(500);
    
  /* 5. 建立TCP连接到OneNET服务器 */
    HAL_usart_send(&huart3, "5. CIPSTART\r\n");
    while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
    HAL_Delay(500);
    
  /* 初始化完成，输出成功信息 */
    HAL_usart_send(&huart3, "6. ESP8266 Init OK\r\n");
     
}

/**

 * @brief  UART接收完成回调函数
 
 * @param  huart - UART句柄指针
 
 * @retval 无
 
 * @note   当使用HAL_UART_Receive_IT()时，每接收一个字符就会调用此回调函数
 
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 /* 检查是否是USART2的中断 */
			if (huart == &huart2)
			{
			/* 防止缓冲区溢出，如果计数达到缓冲区大小则重置计数 */
				if (esp8266_cnt >= sizeof(esp8266_buf))
				{
					// 防止串口被刷爆，与原代码逻辑保持一致
						esp8266_cnt = 0;  
				}
			
			/* 将接收到的一个字节数据存入esp8266缓冲区 */
				esp8266_buf[esp8266_cnt++] = aRxBuffer;
				aRxBuffer = 0; 

			/* 重新启动中断接收，这是HAL库的关键：每次接收完成后需要重新开启中断 */
				HAL_UART_Receive_IT(&huart2, &aRxBuffer, 1);
				
  }
}



/**
 * @brief 生成包含温湿度数据的JSON字符串
 * 
 * @details 该函数创建一个标准格式的JSON字符串，包含设备ID和温湿度传感器数据
 
            用于ESP8266模块向服务器发送数据时使用
 */
void JsonValue()
{
    /* 局部变量定义 */

  // 温度值(摄氏度)，范围通常为0-100
  //  uint8_t Temp;  

  // 湿度值(百分比)，范围通常为0-100     
  //  uint8_t Hum = 60;  

  /* 清空发布缓冲区，确保不会有之前的数据残留 */
    memset(PUBLIS_BUF, 0, sizeof(PUBLIS_BUF));

    /* 格式化JSON字符串

       将温度和湿度数据填充到JSON模板中

       JSON格式: {"id":"123","params":{"Temp":{"value":温度值},"Hum":{"value":湿度值}}}

       其中:"123"是固定的设备ID，Temp和Hum分别表示温度和湿度数据点
    */
    sprintf(PUBLIS_BUF, "{\"id\":\"nw8lCCjUcu\",\"params\":{\"Led\":{\"value\":true }}}"); 
  
}

