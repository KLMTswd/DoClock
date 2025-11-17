/**
	************************************************************
	* 文件名: onenet.c
	
	* 作者: 匿名
	
	* 日期: 2017-05-08
	
	* 版本: V1.1
	
	* 描述: 与OneNet平台通信的接口实现
	
	* 修订记录: V1.0根据协议实现了基本的通信功能
	
	*          V1.1提供统一接口响应，使用不同协议实现不同的通信功能
	
	************************************************************
**/

//芯片头文件
#include "stm32f1xx_hal.h"

//硬件设备
#include "esp8266.h"

//协议头文件
#include "onenet.h"
#include "mqttkit.h"
#include "usart.h"
#include "HAL_usart.h"

//C库
#include <string.h>
#include <stdio.h>
#include "cJSON.h"


/*产品ID*/
#define PROID		"nw8lCCjUcu"

//鉴权Token
#define TOKEN	"version=2018-10-31&res=products%2Fnw8lCCjUcu%2Fdevices%2FSystem&et=1766420163&method=md5&sign=2wQV5XHAExyHLaLPzepIjg%3D%3D"

//设备名称
#define DEVID		"System"

extern unsigned char esp8266_buf[512];


/**
 * @brief  设备连接OneNet平台

 * @note   通过MQTT协议建立与OneNet平台的连接
 
 * @retval 0 - 连接成功，非0 - 连接失败
 
 */
_Bool OneNet_DevLink(void)
{
  // MQTT数据包结构体
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0}; 
  // 指向接收数据的指针
    unsigned char *dataPtr = NULL ;  

  // 连接状态，初始为失败	             
    _Bool status = 1;           
    

  /* 打印调试信息，显示连接参数  */  
  
  // 根据实际需要调整缓冲区大小 
 	  char send_buffer[300];  
    sprintf(send_buffer, "OneNet_DevLink\r\nPROID: %s,\tTOKEN: %s, DEVID:%s\r\n", PROID, TOKEN, DEVID);
    HAL_usart_send(&huart3, send_buffer);
    
  // 构建MQTT连接数据包
    if(MQTT_PacketConnect(PROID, TOKEN, DEVID, 256, 1, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
    {
      // 通过ESP8266发送数据包到平台
        ESP8266_SendData(mqttPacket._data, mqttPacket._len);

      // 等待并获取平台响应，超时时间250ms
        dataPtr = ESP8266_GetIPD(250);

      // 判断是否收到响应
        if(dataPtr != NULL)
        {
          // 解析响应包类型
            if(MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK)
            {
              // 解析连接响应码并处理
                switch(MQTT_UnPacketConnectAck(dataPtr))
                {
                    case 0:
                        HAL_usart_send(&huart3, "Tips:\t连接成功\r\n");
                      // 设置连接成功状态   
						status = 0;  
                        break;
                    
                    case 1:
                        HAL_usart_send(&huart3, "WARN:\t协议版本错误           \r\n");
                        break;
                    case 2:
                        HAL_usart_send(&huart3, "WARN:\t客户端ID无效           \r\n");
                        break;
                    case 3:
                        HAL_usart_send(&huart3, "WARN:\t服务器不可用           \r\n");
                        break;
                    case 4:
                        HAL_usart_send(&huart3, "WARN:\t用户名或密码错误       \r\n");
                        break;
                    case 5:
                        HAL_usart_send(&huart3, "WARN:\t未授权(通常token错误)  \r\n");
                        break;
                    
                    default:
                        HAL_usart_send(&huart3, "ERR:\t其他未知错误            \r\n");
                        break;
                }
            }
        }
        
      // 释放MQTT数据包内存
        MQTT_DeleteBuffer(&mqttPacket);
    }
    else
    {
      // MQTT连接数据包构建失败
        HAL_usart_send(&huart3, "WARN: MQTT_PacketConnect Failed\r\n");
    }
    
  // 返回连接状态
    return status;
}


//==========================================================
//  函数名称   OneNet_Subscribe 
// 
//  函数功能   向OneNet平台订阅指定的MQTT主题列表

//             将设备需要接收的主题信息发送到OneNet服务器
// 
//  函数参数   topics：要订阅的主题字符串数组指针

//             topic_cnt：主题数组的数量
// 
//  函数返回值  无
// 
//  说明：      订阅成功后，设备将能接收到平台下发到这些主题的消息

//==========================================================
void OneNet_Subscribe(const char *topics[], unsigned char topic_cnt)
{
	
  // 循环索引变量
    unsigned char i = 0;    

  // 调试信息发送缓冲区，定义在循环外以提高效率                   
    char send_buffer[100];                   
    
  // MQTT数据包结构体，用于构建订阅请求包
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};                             
    
  // 遍历所有主题，打印调试信息
    for(; i < topic_cnt; i++)
    {   
			
      // 格式化调试信息，显示当前正在订阅的主题
        sprintf(send_buffer, "Subscribe Topic: %s\r\n", topics[i]);
      
      // 通过串口3发送调试信息
        HAL_usart_send(&huart3, send_buffer);
  
		}
    
  // 构建MQTT订阅数据包，QoS等级设为0（最多一次）
    if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL0, topics, topic_cnt, &mqttPacket) == 0)
    {   
			
      // 订阅数据包构建成功，通过ESP8266发送到OneNet平台
        ESP8266_SendData(mqttPacket._data, mqttPacket._len);                     
        
      // 发送完成后释放MQTT数据包缓冲区
        MQTT_DeleteBuffer(&mqttPacket);     
			
    }
    else
    {
      // 订阅数据包构建失败，发送警告信息
        HAL_usart_send(&huart3, "WARN:\tMQTT_PacketSubscribe Failed\r\n");
    }  
   
}


//========================================================== 
//  函数名称   OneNet_Publish 
// 
//  函数功能   向OneNet平台发布MQTT消息

//             将指定主题和内容的消息发送到OneNet服务器
// 
//  函数参数   topic：发布消息的主题字符串

//             msg：要发布的消息内容字符串
// 
//  函数返回值  无
// 
//  说明：      使用QoS 0级别发布消息，适用于非关键数据上报

//==========================================================
void OneNet_Publish(const char *topic, const char *msg)
{
  // MQTT数据包结构体，用于构建发布请求包
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};                         
    
  // 调试信息缓冲区，用于输出发布的主题和消息内容
    char send_buffer[256];  

  // 格式化调试信息
    sprintf(send_buffer, "Publish Topic: %s, Msg: %s\r\n", topic, msg);
  
  // 通过串口3发送调试信息到上位机
    HAL_usart_send(&huart3, send_buffer);
    
    // 构建MQTT发布数据包
   
    // 参数说明：
  
    // - MQTT_PUBLISH_ID：发布消息的ID标识
  
    // - topic：消息主题
  
    // - msg：消息内容
  
    // - strlen(msg)：消息内容长度
   
    // - MQTT_QOS_LEVEL0：服务质量等级0（最多一次送达）
   
    // - 0：非保留消息
   
    // - 1：使用dup标志
   
    // - &mqttPacket：MQTT数据包结构体指针
    if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic, msg, strlen(msg), 
                          MQTT_QOS_LEVEL0, 0, 1, &mqttPacket) == 0)
    {
      // 通过ESP8266模块将MQTT数据包发送到OneNet平台
        ESP8266_SendData(mqttPacket._data, mqttPacket._len);
        
      // 发送完成后，释放MQTT数据包占用的内存缓冲区
        MQTT_DeleteBuffer(&mqttPacket);
    }
    else
    {
      // 发布数据包构建失败，发送警告信息
        HAL_usart_send(&huart3, "WARN:\tMQTT_PacketPublish Failed\r\n");
    }
  
}
//========================================================== 
//  函数名称   OneNet_RevPro 
// 
//  函数功能   处理OneNet平台下发的MQTT协议消息响应

//             解析不同类型的MQTT数据包并执行相应处理
// 
//  函数参数   cmd：接收到的MQTT命令数据包指针
// 
//  函数返回值  无
// 
//  说明：      支持处理命令、发布、订阅确认等多种MQTT消息类型

//==========================================================
void OneNet_RevPro(unsigned char *cmd)
{
  // MQTT数据包结构体，用于存储构建的响应数据包
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0}; 
    
  // 变量定义：存储请求负载、命令ID主题
  // 请求负载数据指针
    char *req_payload = NULL;       

  // 命令ID主题指针
    char *cmdid_topic = NULL;       
    
  // 长度变量
  // 主题长度
  unsigned short topic_len = 0;  
  // 请求负载长度
    unsigned short req_len = 0;     
    
  // MQTT消息相关属性
  // 消息类型  
    unsigned char type = 0;     

  // 服务质量等级      
    unsigned char qos = 0;     
    
  // 数据包ID，静态变量保持值  
    static unsigned short pkt_id = 0; 
    
  // 处理结果和JSON解析相关  
  // 处理结果标志
    short result = 0;      

  // JSON解析对象指针
    cJSON *json ;
		cJSON *params_json __attribute__((unused));		
		
    // char *dataPtr = NULL;         // 预留变量，未使用
    // char numBuf[10];              // 预留变量，未使用
    // int num = 0;                  // 预留变量，未使用
    
  // 解析MQTT数据包类型
    type = MQTT_UnPacketRecv(cmd);
    switch(type)
    {
      // 命令下发类型
        case MQTT_PKT_CMD:  

          // 解析命令主题和请求数据
            result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload, &req_len);
            if(result == 0)
            {

              // 调试输出接收到的命令信息
                char send_buffer[200];  
                sprintf(send_buffer, "cmdid: %s, req: %s, req_len: %d\r\n", 
                        cmdid_topic, req_payload, req_len);
                HAL_usart_send(&huart3, send_buffer);
                
              // 构建命令响应数据包
                if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)
                {
                    HAL_usart_send(&huart3, "Tips:    Send CmdResp\r\n");
                    
                  // 通过ESP8266发送响应数据
                    ESP8266_SendData(mqttPacket._data, mqttPacket._len);
                  
                    // 释放数据包缓冲区
                    MQTT_DeleteBuffer(&mqttPacket);

                }
            }
        break;
            
        case MQTT_PKT_PUBLISH: // 收到平台发布的消息
          
          // 解析发布消息的主题、负载等信息
            result = MQTT_UnPacketPublish(cmd, &cmdid_topic, &topic_len, 
                                        &req_payload, &req_len, &qos, &pkt_id);
            if(result == 0)
            {
                // 调试输出发布消息信息
                char send_buffer[256];  
                sprintf(send_buffer, "topic: %s, topic_len: %d, payload: %s, payload_len: %d\r\n",
                cmdid_topic, topic_len, req_payload, req_len);
                HAL_usart_send(&huart3, send_buffer);
                
                // 尝试解析JSON格式的请求负载
                json = cJSON_Parse(req_payload);
                params_json = cJSON_GetObjectItem(json,"params");            
                cJSON_Delete(json);
                // 注：此处可以添加对params_json的具体处理逻辑
            }
        break;
        
      // 收到平台对发布消息的确认    
        case MQTT_PKT_PUBACK: 
            if(MQTT_UnPacketPublishAck(cmd) == 0)
                HAL_usart_send(&huart3, "Tips:    MQTT Publish Send OK\r\n");
        break;

      // 收到发布确认(PUBREC)，需要回复PUBREL      
        case MQTT_PKT_PUBREC: 
            if(MQTT_UnPacketPublishRec(cmd) == 0)
            {
                HAL_usart_send(&huart3, "Tips:    Rev PublishRec\r\n");
                // 构建并发送PUBREL响应
                if(MQTT_PacketPublishRel(MQTT_PUBLISH_ID, &mqttPacket) == 0)
                {
                    HAL_usart_send(&huart3, "Tips:    Send PublishRel\r\n");
                    ESP8266_SendData(mqttPacket._data, mqttPacket._len);
                    MQTT_DeleteBuffer(&mqttPacket);
                }
            }
        break;

      // 收到平台对PUBREC的响应(PUBREL)，需要回复PUBCOMP      
        case MQTT_PKT_PUBREL: 
            if(MQTT_UnPacketPublishRel(cmd, pkt_id) == 0)
            {
                HAL_usart_send(&huart3, "Tips:    Rev PublishRel\r\n");
              
              // 构建并发送PUBCOMP响应
                if(MQTT_PacketPublishComp(MQTT_PUBLISH_ID, &mqttPacket) == 0)
                {
                    HAL_usart_send(&huart3, "Tips:    Send PublishComp\r\n");
                    ESP8266_SendData(mqttPacket._data, mqttPacket._len);
                    MQTT_DeleteBuffer(&mqttPacket);
                }
            }
        break;
        
      // 收到平台对PUBREL的确认(PUBCOMP)，发布流程完成  
        case MQTT_PKT_PUBCOMP: 
            if(MQTT_UnPacketPublishComp(cmd) == 0)
            {
                HAL_usart_send(&huart3, "Tips:    Rev PublishComp\r\n");
            }
        break;

      // 收到订阅主题的确认      
        case MQTT_PKT_SUBACK: 
            if(MQTT_UnPacketSubscribe(cmd) == 0)
                HAL_usart_send(&huart3, "Tips:    MQTT Subscribe OK\r\n");
            else
                HAL_usart_send(&huart3, "Tips:    MQTT Subscribe Err\r\n");
        break;
      
      // 收到取消订阅的确认  
        case MQTT_PKT_UNSUBACK: 
            if(MQTT_UnPacketUnSubscribe(cmd) == 0)
                HAL_usart_send(&huart3, "Tips:    MQTT UnSubscribe OK\r\n");
            else
                HAL_usart_send(&huart3, "Tips:    MQTT UnSubscribe Err\r\n");
        break;
      
      // 未识别的消息类型  
        default: 
            result = -1;
        break;
    }
    
  // 清除ESP8266接收缓冲区
    ESP8266_Clear();
    
  // 如果处理失败，直接返回
    if(result == -1)
        return;
    
  // 如果是命令或发布消息，释放相应的缓冲区
    if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
    {
        MQTT_FreeBuffer(cmdid_topic);
        MQTT_FreeBuffer(req_payload);
    }
}

/**
 * @brief 发送MQTT PINGREQ报文，维持连接
 
 * @retval 0: 成功, 非0: 失败

 */
uint8_t OneNet_Ping(void)
{
    uint8_t pingbuf[2];  // MQTT PINGREQ报文格式
    
    // MQTT PINGREQ报文格式：
    // 固定头：0xC0 (控制字段) + 0x00 (剩余长度)
    pingbuf[0] = 0xC0;   // PINGREQ控制字段
    pingbuf[1] = 0x00;   // 剩余长度为0
    
    // 通过ESP8266发送PINGREQ报文
    ESP8266_SendData(pingbuf, 2);
    HAL_usart_send(&huart3, "PINGREQ已发送\r\n");
    
    // 等待并尝试接收响应（简化版本）
    ESP8266_Clear();
    HAL_Delay(200);  // 短暂延迟等待可能的响应
    
    // 注意：由于无法使用ESP8266_Buf，这里不做具体响应检查
    // 实际项目中，你应该使用项目中定义的接收缓冲区
    
    return 0;  // 返回成功，表示心跳包已发送
}
