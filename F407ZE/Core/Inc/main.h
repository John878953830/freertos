/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "usart.h"
#include "semphr.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//打印缓存
extern uint8_t tklog[500];
extern uint32_t ulHighFrequencyTimerTicks;
extern uint32_t queuespace;
//数组定义
//limitsw引脚--->电机映射
extern const uint8_t limitsw_to_motorid[17][2];
//命令到电机号映射
extern const uint8_t command_to_motor[60];

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//调试设置
#define DEBUG_OUTPUT 0



//输出gpio映射
#define ENABLE_MOTOR             0
#define ENABLE_PWM               1
#define ENABLE_DIR               2
#define ENABLE_POSTURE_POWER     3

//错误码
#define ERROR_UNKNOWN           0xFF
#define ERROR_FUNC_BUSY         0xFE
#define ERROR_SEMA_NULL         0x01
#define ERROR_CANNOT_GIVE_SEM   0x02
#define ERROR_MOTOR_ID_ERROR    0x03
#define ERROR_CAN_SEND_FAIL     0x04
#define ERROR_CAN_START_FAIL    0x05

//帧结构掩码
#define MASK_PRIORITY           (uint32_t)(0x07 << 26)
#define MASK_COMMAND            (uint32_t)(0x7F << 9)
#define MASK_IF_LAST            (uint32_t)(0x01 << 8)
#define MASK_IF_RETURN          (uint32_t)(0x01 << 7)
#define MASK_IF_ACK             (uint32_t)(0x01 << 6)
#define MASK_VERSION            (uint32_t)(0x07 << 0)
//电机个数和传感器个数设置
#define MAX_MOTOR_NUMBER        3
#define POSTURE_NUM             6
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void timer_start(void);
uint8_t can_start(void);
uint8_t switchGet(uint8_t motor_id);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
typedef struct list_struct{
	uint8_t command_id;                         //CAN 命令ID
	uint8_t command_status;                     //命令状态， 0： 未执行， 1： 执行中  2：执行完成
	struct LIST_STRUCT * next;
}LIST;
typedef struct can_struct{
	uint32_t exid;                              //扩展ID
	uint8_t  data[8];                           //数据域
}CAN_STRUCT;
typedef struct queue_struct{
	uint8_t property;                           //0: can 1: 485
	uint8_t data[8];                            //数据数组
	uint8_t length;                             //数据个数
	uint8_t can_priority;                       //CAN 消息优先级
	uint8_t can_target;                         //can 消息目标
	uint8_t can_source;                         //can 消息源
	uint8_t can_command;                        //can 命令
	uint8_t can_if_last;                        //can 是否拼接标志
	uint8_t can_if_return;                      //can 是否需要返回值
	uint8_t can_if_ack;                         //can 是否需要ack确认
	uint8_t can_version;                        //can 版本号
	uint8_t modbus_addr;                        //485 功能
	uint8_t modbus_func;                        //485 功能码
	uint16_t modbus_data_addr;                  //485 数据地址
	uint16_t modbus_data;                       //485 数据
	uint16_t modbus_crc;                        //485 CRC
	CAN_RxHeaderTypeDef   RxHeader;             //CAN 接收报头数据
}QUEUE_STRUCT;
typedef struct command_struct{
	uint8_t command_id;                         //当前执行的命令ID， 初始化为0
	CAN_STRUCT command_id_history[20];             //执行的命令的历史数据，该变量备用
	uint8_t command_process_mark;               //0: 并行执行， 命令ID标识为并行执行时，默认并行执行  1： 串行执行， 命令ID标识为串行执行时，此标志置位， 该变量备用
	uint8_t priority;                           //命令优先级
	uint8_t if_ack;                             //是否需要ACK
	uint8_t if_return;                          //是否需要执行完回复
	uint8_t if_last;                            //是否需要拼接
	uint8_t can_version;                        //版本号
	uint16_t command_status;                    //当前执行的命令的状态， 0：未执行， 1： 执行中 2： 执行完成 其他： 错误码
	
}COMMAND_STRUCT;
typedef struct speed{
	int32_t current_speed;                      //当前速度，测速函数在测得当前速度后将速度存入此变量
	int32_t set_speed;                          //设置速度，PID输出调速信息后，将设置的速度存入此变量，即和PWM频率或占空比线性相关
	int32_t default_speed;                      //默认速度，有符号数，单位微米
	int32_t max_speed;                          //最大速度，有符号数，单位微米
	int32_t min_speed;                          //最小速度，有符号数，单位微米
	int32_t max_acc;                            //最大加速度，无符号数，单位微米
	uint32_t scal;                              //导程，单位微米
}SPEED;
typedef struct limit_switch{
	uint8_t type;                               //限位开关类型，常开型初始化为0，常闭型初始化为1
	uint8_t property;                           //限位开关属性，0：正转极限 1： 反转极限  2： 正转0点  3： 反转0点
	uint8_t conflict_dir;                       //限位开关关联的碰撞方向， 0：正转 1： 反转, 运行方向定义和此相同，姿态检测开关此值没有意义
	uint8_t status;                             //限位开关当前状态， 0： 未触发 1：触发
	uint16_t pin_number;                         //限位开关对应的引脚号,gpio_pin_xx
	uint8_t gpio_index;                         //限位开关对应的GPIO结构体数组的索引
	GPIO_TypeDef*  gpio_port;                        //限位开关对应的引脚组
}SWITCH;
typedef struct posture_switch{
	uint8_t type;                               //姿态开关类型，常开型初始化为0，常闭型初始化为1
	uint8_t conflict_dir;                       //姿态开关关联的碰撞方向， 0：正转 1： 反转, 运行方向定义和此相同，姿态检测开关此值没有意义
	uint8_t status;                             //姿态开关当前状态， 0： 未触发 1：触发
	uint16_t pin_number;                         //姿态开关对应的引脚号
	GPIO_TypeDef*  gpio_port;                        //姿态开关对应的引脚组
}POSTURE_SWITCH;
typedef struct pid{
	int32_t kp;                                 //比例系数
	int32_t ki;                                 //积分系数
	int32_t kd;                                 //微分系数
}PID;
typedef struct position{
	int32_t zero_offset;                        //0点距离传感器偏移量
	int32_t position_max;                       //最大位置
	int32_t position_min;                       //最小位置
	int32_t current_position;                   //当前位置，位置获取函数在获取位置后，将位置存入此变量
	int32_t target_position;                    //目标位置，存储CAN指令中的目标位置，仅用于调试单步指令
	int32_t tp1;                                //配置文件位置,根据命令文件映射不同的位置，最多支持8个位置
	int32_t tp2;
	int32_t tp3;
	int32_t tp4;
	int32_t tp5;
	int32_t tp6;
	int32_t tp7;
	int32_t tp8;
}POSITION;
typedef struct dimension{
	int32_t dim_x;                              //电机所推动终端的X方向维度
	int32_t dim_y;                              //电机所推动终端的Y方向维度
	int32_t dim_z;                              //电机所推动终端的Z方向维度
	int32_t cen_x;                              //电机所推动终端的X坐标值
	int32_t cen_y;                              //电机所推动终端的Y坐标值
	int32_t cen_z;                              //电机所推动终端的Z坐标值
}DIM;
typedef struct torque{
	uint8_t if_torque;                          //电机是否具有扭矩检测功能， 0：没有 1：有
	uint32_t current_torque;                    //有扭矩检测功能时，代表当前扭矩，单位mN
	uint32_t max_torque;                        //有扭矩检测功能时，代表最大允许扭矩， 单位同上
	uint32_t min_torque;                        //有扭矩检测功能时，代表最小扭矩，单位同上，备用
	uint32_t stop_torque;                       //代表电机的堵转扭矩或静扭矩
}TORQUE;
typedef struct grating_struct{
	uint8_t total_point;                        //光栅总光点数
	uint8_t dark_point;                         //光栅被遮盖的光点数
	uint8_t max_point;                          //光栅被遮盖的光点数的飞机极限最大值
	uint8_t min_point;                          //光栅被遮盖的光点数的飞机极限最小值
	uint32_t distance;                          //光栅被遮盖光点之间的最大距离
	uint32_t distance_max;                      //飞机的允许最大距离
	uint8_t status;                             //光栅检测飞机夹角是否接近45度，0：不接近 1： 接近45度，需要旋转协作处理
	uint8_t if_have_target;                     //光栅检测飞机是否存在， 0： 不存在， 1： 存在
}GRATING;
	

typedef struct gpio_action{
	GPIO_TypeDef*  gpio_port;                    //GPIO 动作输出组
	uint16_t       pin_number;                   //GPIO 引脚号
	uint8_t        break_status;                 //0: 抱闸未放开  1： 抱闸已放开
}GPIO_ACTION;
typedef struct motor_struct{
	uint8_t id;                                  //电机ID
	COMMAND_STRUCT command;                      //命令结构体
	uint8_t current_status;                      //电机当前状态， 0：停止  1： 运行 2: 堵转 3： 出错
	uint8_t dir;                                 //方向信息和方向配置，  bit 0: 当前运行方向  bit 1： 正反转方向配置  bit 2：坐标系方向配置 bit 3：速度映射方向配置  bit 4： 位置映射方向配置  bit 5：pid方向映射配置  bit 6：零点复位方向配置 bit 7：保留
	SPEED speed_value;                           //速度结构体
	POSITION position_value;                     //位置结构体
	uint8_t limit_sw_number;                     //限位开关个数，最大为8，最小为0
	SWITCH limit_sw[7];                          //限位开关组，最多可关联4个限位开关，0：左极限 1：右极限 2：左0点 3：右0点
	PID pid_value;                               //PID结构体
	TORQUE torque_value;                         //电机扭矩特性
	DIM dim_value;                               //维度结构体
	GPIO_ACTION gpio_output[5];                     //gpio 输出组， 0： 电机使能 1： PWM输出， 2： 方向输出 3： 姿态电源继电器输出
	GPIO_ACTION gpio_input[1];                      //gpio 输入组， 0：电机抱闸输入确认信号
}MOTOR_STRUCT;

typedef struct angle_struct{
	uint8_t dir;                                 //需要旋转的方向
	uint16_t angle;                              //需要旋转的角度，单位： 度
}ANGLE_STRUCT;

typedef struct posture_struct{
	GRATING gratingValue;
	POSTURE_SWITCH postureValue[6];              //姿态开关数组，0： X 左边框 1： X 右边框 2： Y左边框 3： Y 右边框 4： X 中轴 5： Y中轴
}POSTURE_STRUCT;
typedef struct gpio_table{
	GPIO_TypeDef*  gpio_port;                    //gpio 引脚组
	uint16_t gpio_number;                        //gpio 引脚号，应和相应电机的限位开关引脚号相同
	uint8_t motor_id;                            //GPIO引脚对应的电机号
	uint8_t limit_sw_index;                      //限位开关组索引
}GPIO_TABLE;

extern MOTOR_STRUCT motor_array[4];
uint8_t can_send(QUEUE_STRUCT send_struct);
uint8_t modbus_send(QUEUE_STRUCT send_struct);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
