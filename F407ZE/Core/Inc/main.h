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
#include "cmsis_os2.h"
#include "timers.h"
#include "tim.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//´òÓ¡»º´æ
extern uint8_t tklog[500];
extern uint32_t ulHighFrequencyTimerTicks;
extern uint32_t queuespace;
extern uint32_t fifo_level;
//·¢ËÍ¶ÓÁÐ¾ä±ú
extern osMessageQueueId_t send_queueHandle;
//Êý×é¶¨Òå
//limitswÒý½Å--->µç»úÓ³Éä
extern const uint8_t limitsw_to_motorid[17][2];
//ÃüÁîµ½µç»úºÅÓ³Éä
extern const uint8_t command_to_motor[60];
//¶¨Ê±Æ÷¶¨Ê±Ê±¼ä
extern uint32_t timer_period;
extern xTimerHandle broadcast_timer;
//IIC³¤¶È»º´æ
extern uint16_t iic_cache;
//´®¿ÚDMA¾ä±ú
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
//modbus cache
extern uint8_t modbus_send_cache[16];
extern uint8_t rece_cache[16];
extern uint8_t rece_count; 
extern uint8_t modbus_status;
extern uint8_t modbus_read_status;     //modbus ¶ÁÈ¡Ö¸ÁîÍê³É±êÖ¾£¬ 0£º ¿ÕÏÐ 1£º¶ÁÈ¡½øÐÐÖÐ 2£º¶ÁÈ¡Íê³É
extern uint8_t modbus_act_status;      //modbus µç»ú¶¯×÷Íê³É±êÖ¾£¬ 0£º¿ÕÏÐ£¬ 1£º¶¯×÷Ö¸Áî½»»¥ÖÐ 2£º ¶¯×÷Ö¸Áî½»»¥Íê³É
extern uint8_t modbus_time_status;     //modbus ³¬Ê±±êÖ¾£¬ 0£º¿ÕÏÐ£¬ 1£º½»»¥³¬Ê± 2£º½»»¥Íê³É
extern uint8_t modbus_time_flag;       //modbus  ¶¨Ê±Ê±¼ä±êÖ¾  1£º µÚÒ»´Î3.5T¶¨Ê±  1£ºµÚ¶þ´Î3.5T¶¨Ê±

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//µ÷ÊÔÉèÖÃ
#define DEBUG_OUTPUT 0
#define COMPLETE_JUDGE          40            //ÖÍÁôÂö³åÊýÎó²îÏÞ
#define SPEED_JUDGE             40            //ËÙ¶ÈÎó²îÏÞ
//ÃüÁî²ÎÊýÓ³Éä
#define CAN_COMMAND_NUMBER      20


//Êä³ögpioÓ³Éä
#define ENABLE_MOTOR             0
#define ENABLE_PWM               1
#define ENABLE_DIR               2
#define ENABLE_POSTURE_POWER     3

//´íÎóÂë
#define ERROR_UNKNOWN           0xFF
#define ERROR_FUNC_BUSY         0xFE
#define ERROR_SEMA_NULL         0x01
#define ERROR_CANNOT_GIVE_SEM   0x02
#define ERROR_MOTOR_ID_ERROR    0x03
#define ERROR_CAN_SEND_FAIL     0x04
#define ERROR_CAN_START_FAIL    0x05

#define ERROR_COMMAND_0_FAIL    0x06
#define ERROR_COMMAND_1_FAIL    0x07
#define ERROR_COMMAND_2_FAIL    0x08
#define ERROR_COMMAND_3_FAIL    0x09
#define ERROR_COMMAND_4_FAIL    0x0A
#define ERROR_COMMAND_5_FAIL    0x0B
#define ERROR_COMMAND_6_FAIL    0x0C
#define ERROR_COMMAND_7_FAIL    0x0D
#define ERROR_COMMAND_8_FAIL    0x0E
#define ERROR_COMMAND_9_FAIL    0x0F
#define ERROR_COMMAND_10_FAIL   0x10
#define ERROR_COMMAND_11_FAIL   0x11
#define ERROR_COMMAND_12_FAIL   0x12
#define ERROR_COMMAND_13_FAIL   0x13
#define ERROR_COMMAND_14_FAIL   0x14
#define ERROR_COMMAND_15_FAIL   0x15
#define ERROR_COMMAND_16_FAIL   0x16
#define ERROR_COMMAND_17_FAIL   0x17
#define ERROR_COMMAND_18_FAIL   0x18
#define ERROR_COMMAND_19_FAIL   0x19
#define ERROR_COMMAND_20_FAIL   0x1A
#define ERROR_COMMAND_21_FAIL   0x1B
#define ERROR_COMMAND_22_FAIL   0x1C
#define ERROR_COMMAND_23_FAIL   0x1D
#define ERROR_COMMAND_24_FAIL   0x1E
#define ERROR_COMMAND_25_FAIL   0x1F
#define ERROR_COMMAND_26_FAIL   0x20
#define ERROR_COMMAND_27_FAIL   0x21
#define ERROR_COMMAND_28_FAIL   0x22
#define ERROR_COMMAND_29_FAIL   0x23
#define ERROR_EEPROM_FAIL       0x24
#define MODBUS_BUSY             0x25
#define MODBUS_LIST_ERROR       0x26
#define ERROR_COMMAND_11_EEPROM_ERROR   0x27
#define ERROR_COMMAND_12_EEPROM_ERROR   0x28
#define ERROR_COMMAND_13_EEPROM_ERROR   0x29
#define ERROR_COMMAND_14_EEPROM_ERROR   0x2A


//Ö¡½á¹¹ÑÚÂë
#define MASK_PRIORITY           (uint32_t)(0x07 << 26)
#define MASK_COMMAND            (uint32_t)(0x7F << 9)
#define MASK_IF_LAST            (uint32_t)(0x01 << 8)
#define MASK_IF_RETURN          (uint32_t)(0x01 << 7)
#define MASK_IF_ACK             (uint32_t)(0x01 << 6)
#define MASK_VERSION            (uint32_t)(0x07 << 0)
#define MASK_TARGET             (uint32_t)(0x1F << 16)
//µç»ú¸öÊýºÍ´«¸ÐÆ÷¸öÊýÉèÖÃ
#define MAX_MOTOR_NUMBER        3
#define POSTURE_NUM             6

//¹âÕ¤GPIOµçÔ´¿ª¹Ø
#define GRATING_POWER_SW        GPIOE
#define GRATING_POWER_PIN       GPIO_PIN_3

#define IIC_ADDRESS             0xA0
#define EEPROM_CONFIG_LENGTH    0x0A

//µç»úÖ¸ÁîµØÖ·
#define P412_H                  (uint16_t)1824
#define P412_L                  (uint16_t)1825

#define POSITION_CURRENT_ADDR    4004
#define SPEED_CURRENT_ADDR       4000
#define TORQUE_CURRENT_ADDR      4016
#define ERROR_CODE_ADDR          4198
#define TEMPERATURE_ADDR         4026
#define REMAIN_PULSE             4012
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef int (*command)(uint8_t*, uint32_t);
typedef void (*result_parse)(uint8_t* uint8_t);
//ÃüÁî»Øµ÷º¯Êý, 26×é£¬7×éÔ¤ÁôÃüÁî
int command_0(uint8_t* data,uint32_t para);
int command_1(uint8_t* data,uint32_t para);
int command_2(uint8_t* data,uint32_t para);
int command_3(uint8_t* data,uint32_t para);
int command_4(uint8_t* data,uint32_t para);
int command_5(uint8_t* data,uint32_t para);
int command_6(uint8_t* data,uint32_t para);
int command_7(uint8_t* data,uint32_t para);
int command_8(uint8_t* data,uint32_t para);
int command_9(uint8_t* data,uint32_t para);
int command_10(uint8_t* data,uint32_t para);
int command_11(uint8_t* data,uint32_t para);
int command_12(uint8_t* data,uint32_t para);
int command_13(uint8_t* data,uint32_t para);
int command_14(uint8_t* data,uint32_t para);
int command_15(uint8_t* data,uint32_t para);
int command_16(uint8_t* data,uint32_t para);
int command_17(uint8_t* data,uint32_t para);
int command_18(uint8_t* data,uint32_t para);
int command_19(uint8_t* data,uint32_t para);
int command_20(uint8_t* data,uint32_t para);
int command_21(uint8_t* data,uint32_t para);
int command_22(uint8_t* data,uint32_t para);
int command_23(uint8_t* data,uint32_t para);
int command_24(uint8_t* data,uint32_t para);
int command_25(uint8_t* data,uint32_t para);
int command_26(uint8_t* data,uint32_t para);

//½á¹û½âÎöº¯Êý£¬¹²6×é£¬ ¶ÔÓ¦Î»ÖÃ£¬ËÙ¶È£3£¬ Å¤¾Ø£¬ 4£º´íÎóÂë£¬ 5£ºÎÂ¶È£¬ 6£ºÖÍÁôÂö³åÊý¬
void result_parse_1(uint8_t* data, uint8_t num);
void result_parse_2(uint8_t* data, uint8_t num);
void result_parse_3(uint8_t* data, uint8_t num);
void result_parse_4(uint8_t* data, uint8_t num);
void result_parse_5(uint8_t* data, uint8_t num);
void result_parse_6(uint8_t* data, uint8_t num);

void timer_start(void);
uint8_t can_start(void);
uint8_t switchGet(uint8_t motor_id);
uint8_t iic_rw(uint8_t rw_flag, uint8_t addr,uint8_t* data,uint8_t length);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
typedef struct list_struct{
	uint8_t command_id;                         //CAN ÃüÁîID
	uint8_t command_status;                     //ÃüÁî×´Ì¬£¬ 0£º Î´Ö´ÐÐ£¬ 1£º Ö´ÐÐÖÐ  2£ºÖ´ÐÐÍê³É
	struct list_struct* next;
}LIST;
typedef struct can_struct{
	uint32_t exid;                              //À©Õ¹ID
	uint8_t  data[8];                           //Êý¾ÝÓò
}CAN_STRUCT;
typedef struct queue_struct{
	uint8_t property;                           //0: can 1: 485
	uint8_t data[8];                            //Êý¾ÝÊý×é
	uint8_t length;                             //Êý¾Ý¸öÊý
	uint8_t can_priority;                       //CAN ÏûÏ¢ÓÅÏÈ¼¶
	uint8_t can_target;                         //can ÏûÏ¢Ä¿±ê
	uint8_t can_source;                         //can ÏûÏ¢Ô´
	uint8_t can_command;                        //can ÃüÁî
	uint8_t can_if_last;                        //can ÊÇ·ñÆ´½Ó±êÖ¾
	uint8_t can_if_return;                      //can ÊÇ·ñÐèÒª·µ»ØÖµ
	uint8_t can_if_ack;                         //can ÊÇ·ñÐèÒªackÈ·ÈÏ
	uint8_t can_version;                        //can °æ±¾ºÅ
	uint8_t modbus_addr;                        //485 Ä£¿éµØÖ·
	uint8_t modbus_func;                        //485 ¹¦ÄÜÂë
	uint8_t modbus_addr_h;                      //485 Êý¾ÝµØÖ·¸ß×Ö½Ú
	uint8_t modbus_addr_l;
	uint8_t modbus_data_len_h;                    //Êý¾Ý³¤¶È
	uint8_t modbus_data_len_l;
	uint8_t modbus_data_byte;
	uint8_t modbus_data_1;                      //485 Êý¾Ý×îµÍ×Ö½Ú
	uint8_t modbus_data_2; 
	uint8_t modbus_data_3;             
	uint8_t modbus_data_4;   
	uint16_t modbus_crc;                        //485 CRC
	uint8_t modbus_property;                    //±êÊ¶modbusµÄ¹¦ÄÜÂë£¬ 1£º Î»ÖÃ£¬2£¬ ËÙ¶È£¬ 3£¬ Å¤¾Ø£¬ 4£º´íÎóÂë£¬ 5£ºÎÂ¶È£¬ 6£ºÖÍÁôÂö³åÊý
	CAN_RxHeaderTypeDef   RxHeader;             //CAN ½ÓÊÕ±¨Í·Êý¾Ý
}QUEUE_STRUCT;
typedef struct modbus_list{
	QUEUE_STRUCT modbus_element;
	uint8_t if_over;                            //±êÖ¾±¾½ÚµãÊÇ·ñÓÐÐ§
	struct modbus_list* next;
	//uint8_t parameter;                          //±êÖ¾²Ù×÷µÄÊôÐÔÖµ£¬ 1£º Î»ÖÃ£¬ 2£º ËÙ¶È£¬ 3£¬ Å¤¾Ø£¬ 4£º´íÎóÂë£¬ 5£ºÎÂ¶È£¬ 6£ºÖÍÁôÂö³åÊý
}MODBUS_LIST;
typedef struct command_struct{
	uint8_t command_id;                         //µ±Ç°Ö´ÐÐµÄÃüÁîID£¬ ³õÊ¼»¯Îª0
	CAN_STRUCT command_id_history[20];          //Ö´ÐÐµÄÃüÁîµÄÀúÊ·Êý¾Ý£¬¸Ã±äÁ¿±¸ÓÃ
	uint8_t command_process_mark;               //0: ²¢ÐÐÖ´ÐÐ£¬ ÃüÁîID±êÊ¶Îª²¢ÐÐÖ´ÐÐÊ±£¬Ä¬ÈÏ²¢ÐÐÖ´ÐÐ  1£º ´®ÐÐÖ´ÐÐ£¬ ÃüÁîID±êÊ¶Îª´®ÐÐÖ´ÐÐÊ±£¬´Ë±êÖ¾ÖÃÎ»£¬ ¸Ã±äÁ¿±¸ÓÃ
	uint8_t priority;                           //ÃüÁîÓÅÏÈ¼¶
	uint8_t if_ack;                             //ÊÇ·ñÐèÒªACK
	uint8_t if_return;                          //ÊÇ·ñÐèÒªÖ´ÐÐÍê»Ø¸´
	uint8_t if_last;                            //ÊÇ·ñÐèÒªÆ´½Ó
	uint8_t can_version;                        //°æ±¾ºÅ
	uint16_t command_status;                    //µ±Ç°Ö´ÐÐµÄÃüÁîµÄ×´Ì¬£¬ 0£ºÎ´Ö´ÐÐ£¬ 1£º Ö´ÐÐÖÐ 2£º Ö´ÐÐÍê³É ÆäËû£º ´íÎóÂë
	
}COMMAND_STRUCT;
typedef struct speed{
	int32_t current_speed;                      //µ±Ç°ËÙ¶È£¬²âËÙº¯ÊýÔÚ²âµÃµ±Ç°ËÙ¶Èºó½«ËÙ¶È´æÈë´Ë±äÁ¿
	int32_t set_speed;                          //ÉèÖÃËÙ¶È£¬PIDÊä³öµ÷ËÙÐÅÏ¢ºó£¬½«ÉèÖÃµÄËÙ¶È´æÈë´Ë±äÁ¿£¬¼´ºÍPWMÆµÂÊ»òÕ¼¿Õ±ÈÏßÐÔÏà¹Ø
	int32_t default_speed;                      //Ä¬ÈÏËÙ¶È£¬ÓÐ·ûºÅÊý£¬µ¥Î»Î¢Ã×
	int32_t max_speed;                          //×î´óËÙ¶È£¬ÓÐ·ûºÅÊý£¬µ¥Î»Î¢Ã×
	int32_t min_speed;                          //×îÐ¡ËÙ¶È£¬ÓÐ·ûºÅÊý£¬µ¥Î»Î¢Ã×
	int32_t max_acc;                            //×î´ó¼ÓËÙ¶È£¬ÎÞ·ûºÅÊý£¬µ¥Î»Î¢Ã×
	uint32_t scal;                              //µ¼³Ì£¬µ¥Î»Î¢Ã×
}SPEED;
typedef struct limit_switch{
	uint8_t type;                               //ÏÞÎ»¿ª¹ØÀàÐÍ£¬³£¿ªÐÍ³õÊ¼»¯Îª0£¬³£±ÕÐÍ³õÊ¼»¯Îª1
	uint8_t property;                           //ÏÞÎ»¿ª¹ØÊôÐÔ£¬0£ºÕý×ª¼«ÏÞ 1£º ·´×ª¼«ÏÞ  2£º Õý×ª0µã  3£º ·´×ª0µã
	uint8_t conflict_dir;                       //ÏÞÎ»¿ª¹Ø¹ØÁªµÄÅö×²·½Ïò£¬ 0£ºÕý×ª 1£º ·´×ª, ÔËÐÐ·½Ïò¶¨ÒåºÍ´ËÏàÍ¬£¬×ËÌ¬¼ì²â¿ª¹Ø´ËÖµÃ»ÓÐÒâÒå
	uint8_t status;                             //ÏÞÎ»¿ª¹Øµ±Ç°×´Ì¬£¬ 0£º Î´´¥·¢ 1£º´¥·¢
	uint16_t pin_number;                         //ÏÞÎ»¿ª¹Ø¶ÔÓ¦µÄÒý½ÅºÅ,gpio_pin_xx
	uint8_t gpio_index;                         //ÏÞÎ»¿ª¹Ø¶ÔÓ¦µÄGPIO½á¹¹ÌåÊý×éµÄË÷Òý
	GPIO_TypeDef*  gpio_port;                        //ÏÞÎ»¿ª¹Ø¶ÔÓ¦µÄÒý½Å×é
}SWITCH;
typedef struct posture_switch{
	uint8_t type;                               //×ËÌ¬¿ª¹ØÀàÐÍ£¬³£¿ªÐÍ³õÊ¼»¯Îª0£¬³£±ÕÐÍ³õÊ¼»¯Îª1
	uint8_t conflict_dir;                       //×ËÌ¬¿ª¹Ø¹ØÁªµÄÅö×²·½Ïò£¬ 0£ºÕý×ª 1£º ·´×ª, ÔËÐÐ·½Ïò¶¨ÒåºÍ´ËÏàÍ¬£¬×ËÌ¬¼ì²â¿ª¹Ø´ËÖµÃ»ÓÐÒâÒå
	uint8_t status;                             //×ËÌ¬¿ª¹Øµ±Ç°×´Ì¬£¬ 0£º Î´´¥·¢ 1£º´¥·¢
	uint16_t pin_number;                        //×ËÌ¬¿ª¹Ø¶ÔÓ¦µÄÒý½ÅºÅ
	GPIO_TypeDef*  gpio_port;                   //×ËÌ¬¿ª¹Ø¶ÔÓ¦µÄÒý½Å×é
}POSTURE_SWITCH;
typedef struct pid{
	int32_t kp;                                 //±ÈÀýÏµÊý
	int32_t ki;                                 //»ý·ÖÏµÊý
	int32_t kd;                                 //Î¢·ÖÏµÊý
}PID;
typedef struct position{
	int32_t zero_offset;                        //0µã¾àÀë´«¸ÐÆ÷Æ«ÒÆÁ¿
	int32_t position_max;                       //×î´óÎ»ÖÃ
	int32_t position_min;                       //×îÐ¡Î»ÖÃ
	int32_t current_position;                   //µ±Ç°Î»ÖÃ£¬Î»ÖÃ»ñÈ¡º¯ÊýÔÚ»ñÈ¡Î»ÖÃºó£¬½«Î»ÖÃ´æÈë´Ë±äÁ¿
	int32_t target_position;                    //Ä¿±êÎ»ÖÃ£¬´æ´¢CANÖ¸ÁîÖÐµÄÄ¿±êÎ»ÖÃ£¬½öÓÃÓÚµ÷ÊÔµ¥²½Ö¸Áî
	int32_t remain_position;                    //ÖÍÁôÂö³åÊý£¬ÐÂÔö£¬²»×÷Îª³£¹æ¼à²âÁ¿
	int32_t remain_position_pre;                //ÉÏÒ»²ÉÑùÊ±¿ÌµÄÖÍÁôÂö³åÊý
	int32_t remain_position_delta;              //²îÖµ
	int32_t remain_position_delta_pre;          //ÉÏÒ»´ÎµÄ²îÖµ
	int32_t tp[8];                                //ÅäÖÃÎÄ¼þÎ»ÖÃ,¸ù¾ÝÃüÁîÎÄ¼þÓ³Éä²»Í¬µÄÎ»ÖÃ£¬×î¶àÖ§³Ö8¸öÎ»ÖÃ, tp1:µç»ú45¶ÈÎ»ÖÃ£¬³õÊ¼»¯Ê±´Óeeprom¶ÁÈë
	//tp2£¬µç»úÍêÈ«¼Ð½ôÎ»ÖÃ£¬ ³õÊ¼»¯Ê±´Óeeprom¶ÁÈë
	//tp3£¬µç»úÍêÈ«ËÉ¿ªÎ»ÖÃ£¬³õÊ¼»¯Ê±´Óeeprom¶ÁÈë
	uint8_t if_tp_already;                      //±êÖ¾tpÊÇ·ñÓÐÐ§£¬³õÊ¼»¯Ê±Îª0£¬ ³É¹¦¶ÁÈ¡eepromÊý¾ÝºóÖÃ1
}POSITION;
typedef struct dimension{
	int32_t dim_x;                              //µç»úËùÍÆ¶¯ÖÕ¶ËµÄX·½ÏòÎ¬¶È
	int32_t dim_y;                              //µç»úËùÍÆ¶¯ÖÕ¶ËµÄY·½ÏòÎ¬¶È
	int32_t dim_z;                              //µç»úËùÍÆ¶¯ÖÕ¶ËµÄZ·½ÏòÎ¬¶È
	int32_t cen_x;                              //µç»úËùÍÆ¶¯ÖÕ¶ËµÄX×ø±êÖµ
	int32_t cen_y;                              //µç»úËùÍÆ¶¯ÖÕ¶ËµÄY×ø±êÖµ
	int32_t cen_z;                              //µç»úËùÍÆ¶¯ÖÕ¶ËµÄZ×ø±êÖµ
}DIM;
typedef struct torque{
	uint8_t if_torque;                          //µç»úÊÇ·ñ¾ßÓÐÅ¤¾Ø¼ì²â¹¦ÄÜ£¬ 0£ºÃ»ÓÐ 1£ºÓÐ
	uint32_t current_torque;                    //ÓÐÅ¤¾Ø¼ì²â¹¦ÄÜÊ±£¬´ú±íµ±Ç°Å¤¾Ø£¬µ¥Î»mN
	uint32_t max_torque;                        //ÓÐÅ¤¾Ø¼ì²â¹¦ÄÜÊ±£¬´ú±í×î´óÔÊÐíÅ¤¾Ø£¬ µ¥Î»Í¬ÉÏ
	uint32_t min_torque;                        //ÓÐÅ¤¾Ø¼ì²â¹¦ÄÜÊ±£¬´ú±í×îÐ¡Å¤¾Ø£¬µ¥Î»Í¬ÉÏ£¬±¸ÓÃ
	uint32_t stop_torque;                       //´ú±íµç»úµÄ¶Â×ªÅ¤¾Ø»ò¾²Å¤¾Ø
}TORQUE;
typedef struct grating_struct{
	uint8_t total_point;                        //¹âÕ¤×Ü¹âµãÊý
	uint8_t dark_point;                         //¹âÕ¤±»ÕÚ¸ÇµÄ¹âµãÊý
	uint8_t max_point;                          //¹âÕ¤±»ÕÚ¸ÇµÄ¹âµãÊýµÄ·É»ú¼«ÏÞ×î´óÖµ
	uint8_t min_point;                          //¹âÕ¤±»ÕÚ¸ÇµÄ¹âµãÊýµÄ·É»ú¼«ÏÞ×îÐ¡Öµ
	uint32_t distance;                          //¹âÕ¤±»ÕÚ¸Ç¹âµãÖ®¼äµÄ×î´ó¾àÀë
	uint32_t distance_max;                      //·É»úµÄÔÊÐí×î´ó¾àÀë
	uint8_t status;                             //¹âÕ¤¼ì²â·É»ú¼Ð½ÇÊÇ·ñ½Ó½ü45¶È£¬0£º²»½Ó½ü 1£º ½Ó½ü45¶È£¬ÐèÒªÐý×ªÐ­×÷´¦Àí
	uint8_t if_have_target;                     //¹âÕ¤¼ì²â·É»úÊÇ·ñ´æÔÚ£¬ 0£º ²»´æÔÚ£¬ 1£º ´æÔÚ
}GRATING;
	

typedef struct gpio_action{
	GPIO_TypeDef*  gpio_port;                    //GPIO ¶¯×÷Êä³ö×é
	uint16_t       pin_number;                   //GPIO Òý½ÅºÅ
	uint8_t        break_status;                 //0: ±§Õ¢Î´·Å¿ª  1£º ±§Õ¢ÒÑ·Å¿ª£¬Ö»¶ÔÌØ¶¨µç»úµÄÌØ¶¨×éÓÐÒâÒå
}GPIO_ACTION;
typedef struct motor_struct{
	uint8_t id;                                  //µç»úID
	COMMAND_STRUCT command;                      //ÃüÁî½á¹¹Ìå
	uint8_t current_status;                      //µç»úµ±Ç°×´Ì¬£¬ 0£ºÍ£Ö¹  1£º ÔËÐÐ 2: ¶Â×ª 3£º ³ö´í
	uint8_t dir;                                 //·½ÏòÐÅÏ¢ºÍ·½ÏòÅäÖÃ£¬  bit 0: µ±Ç°ÔËÐÐ·½Ïò  bit 1£º Õý·´×ª·½ÏòÅäÖÃ  bit 2£º×ø±êÏµ·½ÏòÅäÖÃ bit 3£ºËÙ¶ÈÓ³Éä·½ÏòÅäÖÃ  bit 4£º Î»ÖÃÓ³Éä·½ÏòÅäÖÃ  bit 5£ºpid·½ÏòÓ³ÉäÅäÖÃ  bit 6£ºÁãµã¸´Î»·½ÏòÅäÖÃ bit 7£º±£Áô
	SPEED speed_value;                           //ËÙ¶È½á¹¹Ìå
	POSITION position_value;                     //Î»ÖÃ½á¹¹Ìå
	uint8_t limit_sw_number;                     //ÏÞÎ»¿ª¹Ø¸öÊý£¬×î´óÎª8£¬×îÐ¡Îª0
	SWITCH limit_sw[7];                          //ÏÞÎ»¿ª¹Ø×é£¬×î¶à¿É¹ØÁª4¸öÏÞÎ»¿ª¹Ø£¬0£º×ó¼«ÏÞ 1£ºÓÒ¼«ÏÞ 2£º×ó0µã 3£ºÓÒ0µã
	PID pid_value;                               //PID½á¹¹Ìå
	TORQUE torque_value;                         //µç»úÅ¤¾ØÌØÐÔ
	DIM dim_value;                               //Î¬¶È½á¹¹Ìå
	GPIO_ACTION gpio_output[5];                     //gpio Êä³ö×é£¬ 0£º µç»úÊ¹ÄÜ 1£º PWMÊä³ö£¬ 2£º ·½ÏòÊä³ö 3£º ×ËÌ¬µçÔ´¼ÌµçÆ÷Êä³ö
	GPIO_ACTION gpio_input[1];                      //gpio ÊäÈë×é£¬ 0£ºµç»ú±§Õ¢ÊäÈëÈ·ÈÏÐÅºÅ
	uint32_t register_move;                         //µç»úÔË¶¯¼Ä´æÆ÷µÄ485µØÖ·
	uint32_t temperature;                         //ÎÂ¶ÈÐÅÏ¢
	uint32_t motor_error_code;                    //µç»úÇý¶¯Æ÷µÄ´íÎóÂë
}MOTOR_STRUCT;

typedef struct angle_struct{
	uint8_t dir;                                 //ÐèÒªÐý×ªµÄ·½Ïò
	uint16_t angle;                              //ÐèÒªÐý×ªµÄ½Ç¶È£¬µ¥Î»£º ¶È
}ANGLE_STRUCT;

typedef struct posture_struct{
	GRATING gratingValue;
	POSTURE_SWITCH postureValue[6];              //×ËÌ¬¿ª¹ØÊý×é£¬0£º X ×ó±ß¿ò 1£º X ÓÒ±ß¿ò 2£º Y×ó±ß¿ò 3£º Y ÓÒ±ß¿ò 4£º X ÖÐÖá 5£º YÖÐÖá
}POSTURE_STRUCT;
typedef struct gpio_table{
	GPIO_TypeDef*  gpio_port;                    //gpio Òý½Å×é
	uint16_t gpio_number;                        //gpio Òý½ÅºÅ£¬Ó¦ºÍÏàÓ¦µç»úµÄÏÞÎ»¿ª¹ØÒý½ÅºÅÏàÍ¬
	uint8_t motor_id;                            //GPIOÒý½Å¶ÔÓ¦µÄµç»úºÅ
	uint8_t limit_sw_index;                      //ÏÞÎ»¿ª¹Ø×éË÷Òý
}GPIO_TABLE;

extern MOTOR_STRUCT motor_array[4];
uint8_t can_send(QUEUE_STRUCT send_struct);
uint8_t modbus_send(QUEUE_STRUCT send_struct);
uint8_t modbus_send_sub(QUEUE_STRUCT send_struct);
uint8_t iic_rw(uint8_t rw_flag, uint8_t addr,uint8_t* data,uint8_t length);
extern int(*command_to_function[27])(uint8_t*,uint32_t);
extern void(*result_to_parameter[7])(uint8_t*, uint8_t);
extern uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen );
extern MODBUS_LIST* modbus_list_head;
extern MODBUS_LIST* modbus_list_tail;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
