#ifndef _EVENT_
#define _EVENT_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdint.h>

//CMD: LEVENTRH  Read All Event Log. 读取雷达事件命令，返回SYS_EVENT_LOG结构体，软件需要解析数据包事件类型。
//CMD: LEVENTCH  Clear ALL Event Log 清除雷达事件命令，但是会记录一次清除事件(EVENT_ID_EVENT_CLEARED)。

#define EVENT_MEM_ADDR    0x2020E800  //m_data3_start...
#define EVENT_PACK_HEADER 0xEFDC55AA
#define MAX_EVENT_COUNT   128

#define EVENT_TYPE_SYS_BOOT  	 0   //power boot on event  设备启动类型事件
#define EVENT_ID_BOOT_PORRST   1   //power on reset				上电启动
#define EVENT_ID_BOOT_SOFTRST  2   //software reset				软件复位启动
#define EVENT_ID_BOOT_WDOGRST  3   //watchdog reset				开门狗启动
#define EVENT_ID_BOOT_HARDRST  4   //hardware reset				硬件复位启动

#define EVENT_TYPE_POWER    	 1   //power  event					电源异常事件
#define EVENT_ID_POWER_LOW     1   //power low min  8V		低压事件，每次重启仅记录一次
#define EVENT_ID_POWER_HIGH    2   //power High max 30V		高压事件，每次重启仅记录一次

#define EVENT_TYPE_NET   				2   //net event.					网络异常事件
#define EVENT_ID_NET_LINK_UP   	1		//link up							网络掉线
#define EVENT_ID_NET_LINK_DOWN  2   //link down						网络重连
#define EVENT_ID_NET_SEND_DROP  3   //net send error.			网络发送失败
#define EVENT_ID_NET_STACK_ERR   4  //网络协议栈错误，会导致系统复位

#define EVENT_TYPE_TDC_DATA   		3	  //TDC Board ir data				机头测距模块事件
#define EVENT_ID_TDC_IR_DATA_STOP	  1		//no ir data						无红外数据，每次重启仅记录一次
#define EVENT_ID_TDC_IR_DATA_ERROR  2   //ir uart data error		红外数据误码过多，每次重启仅记录一次
#define EVENT_ID_TDC_APD_HIGHTMP    3   //apd above 85 deg			机头apd高温，每次重启仅记录一次

#define EVENT_TYPE_MAIN_MOTOR   	4		//main motor 							主马达事件
#define EVENT_ID_MAIN_MOTOR_STUCK	1		//motor stop							堵转	
#define EVENT_ID_MAIN_MOTOR_SLOW  2   //motor slow							转速太慢
#define EVENT_ID_MAIN_MOTOR_FAST  3   //motor fast							转速太快

#define EVENT_TYPE_MIRROR_MOTOR   5			//mirror motor					转镜马达事件	
#define EVENT_ID_MIRROR_MOTOR_STUCK	1		//motor stop						堵转
#define EVENT_ID_MIRROR_MOTOR_SLOW  2   //motor slow						转速太慢
#define EVENT_ID_MIRROR_MOTOR_FAST  3   //motor fast						转速太快

#define EVENT_TYPE_IMU   				6		//imu												IMU事件	
#define EVENT_ID_IMU_INITFAILD	1		//imu  init error,id...			IMU芯片初始化检测异常
#define EVENT_ID_IMU_NODATA  		2   //imu  nodata								IMU数据异常（读取失败）
#define EVENT_ID_IMU_TIMESTMP_LEAP   3//imu ts leap							IMU时间戳跳动异常，大于2个正常间隔
#define EVENT_ID_IMU_TIMESTMP_BACKW  4//imu ts backw						IMU时间戳回跳
#define EVENT_ID_IMU_SEND   5					//imu send										IMU网络数据包发送失败

#define EVENT_TYPE_EVENT 				15	//evnet											事件操作记录																	
#define EVENT_ID_EVENT_CLEARED	1		//EVENT CLEAR BY software.	事件被上位机清空。

#pragma pack (push,1)
typedef  struct 
 {
  uint32_t  Time;								  //10ms sec since first power on. 事件发生时间，单位10ms，从第一次冷启动开始
  uint8_t 	Event_Type;						//事件类型
	uint8_t 	Event_ID;							//事件具体ID
 } EVENT_LOG;

typedef  struct 
 {
  uint32_t Header;						  					//0xEFDC55AA
	uint32_t Current_Time_10mSec;   				//10ms sec since first power on,software reset safe. max 490 day. 事件结构体时间，单位10ms，最大490天。从第一次冷启动开始
	uint32_t Current_Time_10mSec_Complement; //for fast check sum，是 0-Current_Time_10mSec的结果。
	uint8_t  Boot_Count;										//设备冷启动后，有多少次热重启
	uint8_t  Event_Count;										//已经记录多少次记录，0-255，最大255，不回滚。但是记录最大为MAX_EVENT_COUNT限制，仅报告时间次数。
	uint8_t  Current_Event_Index; 					//最新一次记录索引，0-MAX_EVENT_COUNT，超过最大记录后会回滚覆盖更新。
	uint8_t  Readed_Flag; 									//读取事件，暂不使用
  EVENT_LOG Event[MAX_EVENT_COUNT];				//单个时间结构体
  uint32_t DCSum;  						    				//data sum from BootCount To Event End. 事件校验和，计算从BootCount开始，到DCsum之前。
 } SYS_EVENT_LOG; 							  //注意不同操作系统的__PACKED。。。
 #pragma pack (pop)
 
#endif
