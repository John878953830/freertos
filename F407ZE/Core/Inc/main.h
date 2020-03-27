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
//��ӡ����
extern uint8_t tklog[500];
extern uint32_t ulHighFrequencyTimerTicks;
extern uint32_t queuespace;
extern uint32_t fifo_level;
//���Ͷ��о��
extern osMessageQueueId_t send_queueHandle;
//���鶨��
//limitsw����--->���ӳ��
extern const uint8_t limitsw_to_motorid[17][2];
//��������ӳ��
extern const uint8_t command_to_motor[60];
//��ʱ����ʱʱ��
extern uint32_t timer_period;
extern xTimerHandle broadcast_timer;
//IIC���Ȼ���
extern uint16_t iic_cache;
//����DMA���
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
//modbus cache
extern uint8_t modbus_send_cache[16];
extern uint8_t rece_cache[16];
extern uint8_t rece_count; 
extern uint8_t modbus_status;
extern uint8_t modbus_read_status;     //modbus ��ȡָ����ɱ�־�� 0�� ���� 1����ȡ������ 2����ȡ���
extern uint8_t modbus_act_status;      //modbus ���������ɱ�־�� 0�����У� 1������ָ����� 2�� ����ָ������
extern uint8_t modbus_time_status;     //modbus ��ʱ��־�� 0�����У� 1��������ʱ 2���������
extern uint8_t modbus_time_flag;       //modbus  ��ʱʱ���־  1�� ��һ��3.5T��ʱ  1���ڶ���3.5T��ʱ

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//��������
#define DEBUG_OUTPUT 0
#define COMPLETE_JUDGE          40            //���������������
#define SPEED_JUDGE             40            //�ٶ������
//�������ӳ��
#define CAN_COMMAND_NUMBER      20


//���gpioӳ��
#define ENABLE_MOTOR             0
#define ENABLE_PWM               1
#define ENABLE_DIR               2
#define ENABLE_POSTURE_POWER     3

//������
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


//֡�ṹ����
#define MASK_PRIORITY           (uint32_t)(0x07 << 26)
#define MASK_COMMAND            (uint32_t)(0x7F << 9)
#define MASK_IF_LAST            (uint32_t)(0x01 << 8)
#define MASK_IF_RETURN          (uint32_t)(0x01 << 7)
#define MASK_IF_ACK             (uint32_t)(0x01 << 6)
#define MASK_VERSION            (uint32_t)(0x07 << 0)
#define MASK_TARGET             (uint32_t)(0x1F << 16)
//��������ʹ�������������
#define MAX_MOTOR_NUMBER        3
#define POSTURE_NUM             6

//��դGPIO��Դ����
#define GRATING_POWER_SW        GPIOE
#define GRATING_POWER_PIN       GPIO_PIN_3

#define IIC_ADDRESS             0xA0
#define EEPROM_CONFIG_LENGTH    0x0A

//���ָ���ַ
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
//����ص�����, 26�飬7��Ԥ������
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

//���������������6�飬 ��Ӧλ�ã��ٶȣ3�� Ť�أ� 4�������룬 5���¶ȣ� 6�������������
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
	uint8_t command_id;                         //CAN ����ID
	uint8_t command_status;                     //����״̬�� 0�� δִ�У� 1�� ִ����  2��ִ�����
	struct list_struct* next;
}LIST;
typedef struct can_struct{
	uint32_t exid;                              //��չID
	uint8_t  data[8];                           //������
}CAN_STRUCT;
typedef struct queue_struct{
	uint8_t property;                           //0: can 1: 485
	uint8_t data[8];                            //��������
	uint8_t length;                             //���ݸ���
	uint8_t can_priority;                       //CAN ��Ϣ���ȼ�
	uint8_t can_target;                         //can ��ϢĿ��
	uint8_t can_source;                         //can ��ϢԴ
	uint8_t can_command;                        //can ����
	uint8_t can_if_last;                        //can �Ƿ�ƴ�ӱ�־
	uint8_t can_if_return;                      //can �Ƿ���Ҫ����ֵ
	uint8_t can_if_ack;                         //can �Ƿ���Ҫackȷ��
	uint8_t can_version;                        //can �汾��
	uint8_t modbus_addr;                        //485 ģ���ַ
	uint8_t modbus_func;                        //485 ������
	uint8_t modbus_addr_h;                      //485 ���ݵ�ַ���ֽ�
	uint8_t modbus_addr_l;
	uint8_t modbus_data_len_h;                    //���ݳ���
	uint8_t modbus_data_len_l;
	uint8_t modbus_data_byte;
	uint8_t modbus_data_1;                      //485 ��������ֽ�
	uint8_t modbus_data_2; 
	uint8_t modbus_data_3;             
	uint8_t modbus_data_4;   
	uint16_t modbus_crc;                        //485 CRC
	uint8_t modbus_property;                    //��ʶmodbus�Ĺ����룬 1�� λ�ã�2�� �ٶȣ� 3�� Ť�أ� 4�������룬 5���¶ȣ� 6������������
	CAN_RxHeaderTypeDef   RxHeader;             //CAN ���ձ�ͷ����
}QUEUE_STRUCT;
typedef struct modbus_list{
	QUEUE_STRUCT modbus_element;
	uint8_t if_over;                            //��־���ڵ��Ƿ���Ч
	struct modbus_list* next;
	//uint8_t parameter;                          //��־����������ֵ�� 1�� λ�ã� 2�� �ٶȣ� 3�� Ť�أ� 4�������룬 5���¶ȣ� 6������������
}MODBUS_LIST;
typedef struct command_struct{
	uint8_t command_id;                         //��ǰִ�е�����ID�� ��ʼ��Ϊ0
	CAN_STRUCT command_id_history[20];          //ִ�е��������ʷ���ݣ��ñ�������
	uint8_t command_process_mark;               //0: ����ִ�У� ����ID��ʶΪ����ִ��ʱ��Ĭ�ϲ���ִ��  1�� ����ִ�У� ����ID��ʶΪ����ִ��ʱ���˱�־��λ�� �ñ�������
	uint8_t priority;                           //�������ȼ�
	uint8_t if_ack;                             //�Ƿ���ҪACK
	uint8_t if_return;                          //�Ƿ���Ҫִ����ظ�
	uint8_t if_last;                            //�Ƿ���Ҫƴ��
	uint8_t can_version;                        //�汾��
	uint16_t command_status;                    //��ǰִ�е������״̬�� 0��δִ�У� 1�� ִ���� 2�� ִ����� ������ ������
	
}COMMAND_STRUCT;
typedef struct speed{
	int32_t current_speed;                      //��ǰ�ٶȣ����ٺ����ڲ�õ�ǰ�ٶȺ��ٶȴ���˱���
	int32_t set_speed;                          //�����ٶȣ�PID���������Ϣ�󣬽����õ��ٶȴ���˱���������PWMƵ�ʻ�ռ�ձ��������
	int32_t default_speed;                      //Ĭ���ٶȣ��з���������λ΢��
	int32_t max_speed;                          //����ٶȣ��з���������λ΢��
	int32_t min_speed;                          //��С�ٶȣ��з���������λ΢��
	int32_t max_acc;                            //�����ٶȣ��޷���������λ΢��
	uint32_t scal;                              //���̣���λ΢��
}SPEED;
typedef struct limit_switch{
	uint8_t type;                               //��λ�������ͣ������ͳ�ʼ��Ϊ0�������ͳ�ʼ��Ϊ1
	uint8_t property;                           //��λ�������ԣ�0����ת���� 1�� ��ת����  2�� ��ת0��  3�� ��ת0��
	uint8_t conflict_dir;                       //��λ���ع�������ײ���� 0����ת 1�� ��ת, ���з�����ʹ���ͬ����̬��⿪�ش�ֵû������
	uint8_t status;                             //��λ���ص�ǰ״̬�� 0�� δ���� 1������
	uint16_t pin_number;                         //��λ���ض�Ӧ�����ź�,gpio_pin_xx
	uint8_t gpio_index;                         //��λ���ض�Ӧ��GPIO�ṹ�����������
	GPIO_TypeDef*  gpio_port;                        //��λ���ض�Ӧ��������
}SWITCH;
typedef struct posture_switch{
	uint8_t type;                               //��̬�������ͣ������ͳ�ʼ��Ϊ0�������ͳ�ʼ��Ϊ1
	uint8_t conflict_dir;                       //��̬���ع�������ײ���� 0����ת 1�� ��ת, ���з�����ʹ���ͬ����̬��⿪�ش�ֵû������
	uint8_t status;                             //��̬���ص�ǰ״̬�� 0�� δ���� 1������
	uint16_t pin_number;                        //��̬���ض�Ӧ�����ź�
	GPIO_TypeDef*  gpio_port;                   //��̬���ض�Ӧ��������
}POSTURE_SWITCH;
typedef struct pid{
	int32_t kp;                                 //����ϵ��
	int32_t ki;                                 //����ϵ��
	int32_t kd;                                 //΢��ϵ��
}PID;
typedef struct position{
	int32_t zero_offset;                        //0����봫����ƫ����
	int32_t position_max;                       //���λ��
	int32_t position_min;                       //��Сλ��
	int32_t current_position;                   //��ǰλ�ã�λ�û�ȡ�����ڻ�ȡλ�ú󣬽�λ�ô���˱���
	int32_t target_position;                    //Ŀ��λ�ã��洢CANָ���е�Ŀ��λ�ã������ڵ��Ե���ָ��
	int32_t remain_position;                    //����������������������Ϊ��������
	int32_t remain_position_pre;                //��һ����ʱ�̵�����������
	int32_t remain_position_delta;              //��ֵ
	int32_t remain_position_delta_pre;          //��һ�εĲ�ֵ
	int32_t tp[8];                                //�����ļ�λ��,���������ļ�ӳ�䲻ͬ��λ�ã����֧��8��λ��, tp1:���45��λ�ã���ʼ��ʱ��eeprom����
	//tp2�������ȫ�н�λ�ã� ��ʼ��ʱ��eeprom����
	//tp3�������ȫ�ɿ�λ�ã���ʼ��ʱ��eeprom����
	uint8_t if_tp_already;                      //��־tp�Ƿ���Ч����ʼ��ʱΪ0�� �ɹ���ȡeeprom���ݺ���1
}POSITION;
typedef struct dimension{
	int32_t dim_x;                              //������ƶ��ն˵�X����ά��
	int32_t dim_y;                              //������ƶ��ն˵�Y����ά��
	int32_t dim_z;                              //������ƶ��ն˵�Z����ά��
	int32_t cen_x;                              //������ƶ��ն˵�X����ֵ
	int32_t cen_y;                              //������ƶ��ն˵�Y����ֵ
	int32_t cen_z;                              //������ƶ��ն˵�Z����ֵ
}DIM;
typedef struct torque{
	uint8_t if_torque;                          //����Ƿ����Ť�ؼ�⹦�ܣ� 0��û�� 1����
	uint32_t current_torque;                    //��Ť�ؼ�⹦��ʱ������ǰŤ�أ���λmN
	uint32_t max_torque;                        //��Ť�ؼ�⹦��ʱ�������������Ť�أ� ��λͬ��
	uint32_t min_torque;                        //��Ť�ؼ�⹦��ʱ��������СŤ�أ���λͬ�ϣ�����
	uint32_t stop_torque;                       //�������Ķ�תŤ�ػ�Ť��
}TORQUE;
typedef struct grating_struct{
	uint8_t total_point;                        //��դ�ܹ����
	uint8_t dark_point;                         //��դ���ڸǵĹ����
	uint8_t max_point;                          //��դ���ڸǵĹ�����ķɻ��������ֵ
	uint8_t min_point;                          //��դ���ڸǵĹ�����ķɻ�������Сֵ
	uint32_t distance;                          //��դ���ڸǹ��֮���������
	uint32_t distance_max;                      //�ɻ�������������
	uint8_t status;                             //��դ���ɻ��н��Ƿ�ӽ�45�ȣ�0�����ӽ� 1�� �ӽ�45�ȣ���Ҫ��תЭ������
	uint8_t if_have_target;                     //��դ���ɻ��Ƿ���ڣ� 0�� �����ڣ� 1�� ����
}GRATING;
	

typedef struct gpio_action{
	GPIO_TypeDef*  gpio_port;                    //GPIO ���������
	uint16_t       pin_number;                   //GPIO ���ź�
	uint8_t        break_status;                 //0: ��բδ�ſ�  1�� ��բ�ѷſ���ֻ���ض�������ض���������
}GPIO_ACTION;
typedef struct motor_struct{
	uint8_t id;                                  //���ID
	COMMAND_STRUCT command;                      //����ṹ��
	uint8_t current_status;                      //�����ǰ״̬�� 0��ֹͣ  1�� ���� 2: ��ת 3�� ����
	uint8_t dir;                                 //������Ϣ�ͷ������ã�  bit 0: ��ǰ���з���  bit 1�� ����ת��������  bit 2������ϵ�������� bit 3���ٶ�ӳ�䷽������  bit 4�� λ��ӳ�䷽������  bit 5��pid����ӳ������  bit 6����㸴λ�������� bit 7������
	SPEED speed_value;                           //�ٶȽṹ��
	POSITION position_value;                     //λ�ýṹ��
	uint8_t limit_sw_number;                     //��λ���ظ��������Ϊ8����СΪ0
	SWITCH limit_sw[7];                          //��λ�����飬���ɹ���4����λ���أ�0������ 1���Ҽ��� 2����0�� 3����0��
	PID pid_value;                               //PID�ṹ��
	TORQUE torque_value;                         //���Ť������
	DIM dim_value;                               //ά�Ƚṹ��
	GPIO_ACTION gpio_output[5];                     //gpio ����飬 0�� ���ʹ�� 1�� PWM����� 2�� ������� 3�� ��̬��Դ�̵������
	GPIO_ACTION gpio_input[1];                      //gpio �����飬 0�������բ����ȷ���ź�
	uint32_t register_move;                         //����˶��Ĵ�����485��ַ
	uint32_t temperature;                         //�¶���Ϣ
	uint32_t motor_error_code;                    //����������Ĵ�����
}MOTOR_STRUCT;

typedef struct angle_struct{
	uint8_t dir;                                 //��Ҫ��ת�ķ���
	uint16_t angle;                              //��Ҫ��ת�ĽǶȣ���λ�� ��
}ANGLE_STRUCT;

typedef struct posture_struct{
	GRATING gratingValue;
	POSTURE_SWITCH postureValue[6];              //��̬�������飬0�� X ��߿� 1�� X �ұ߿� 2�� Y��߿� 3�� Y �ұ߿� 4�� X ���� 5�� Y����
}POSTURE_STRUCT;
typedef struct gpio_table{
	GPIO_TypeDef*  gpio_port;                    //gpio ������
	uint16_t gpio_number;                        //gpio ���źţ�Ӧ����Ӧ�������λ�������ź���ͬ
	uint8_t motor_id;                            //GPIO���Ŷ�Ӧ�ĵ����
	uint8_t limit_sw_index;                      //��λ����������
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
