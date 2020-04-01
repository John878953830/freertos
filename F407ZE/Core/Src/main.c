/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t timer_period=1000;
xTimerHandle broadcast_timer;
xTimerHandle motor_status_timer;     //���״̬������ʱ��
uint32_t fifo_level=0;
uint16_t iic_cache=0;
//modbus cache
uint8_t modbus_send_cache[16];
uint8_t rece_cache[16];
uint8_t rece_count=0;
uint8_t modbus_status=0;        //modbus ״̬������ 0�� ���� 1��������
uint8_t modbus_read_status;     //modbus ��ȡָ����ɱ�־�� 0�� ���� 1����ȡ������ 2����ȡ���
uint8_t modbus_act_status;      //modbus ���������ɱ�־�� 0�����У� 1������ָ����� 2�� ����ָ������
uint8_t modbus_time_status;     //modbus ��ʱ��־�� 0�����У� 1��������ʱ 2���������
uint8_t modbus_time_flag=0;       //modbus  ��ʱʱ���־  1�� ��һ��3.5T��ʱ  1���ڶ���3.5T��ʱ


MODBUS_LIST* modbus_list_head=NULL;  //head ָ��Ѱ�ҵ��ĵ�һ����Ϊ�յĽڵ�
MODBUS_LIST* modbus_list_tail=NULL;  //tail ָ��Ϊ�յĽڵ����һ���ڵ�
uint16_t modbus_period=89;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t motor_array_init(void)
{
	uint8_t rw_flag=0,addr=0,length=0;
	uint16_t i=0,j=0;
	uint8_t data[4]={0};
	//��ʼ������ṹ���е�һЩ��
	uint8_t tmp_addr=0x00;                      //1�ŵ����ʼ��ַ
	length=4;
	rw_flag=0;
	//1�ŵ��
	for(i=0;i<3;i++)
	{
		if(iic_rw(rw_flag, tmp_addr + i*4,data,length)!=0)
		{
			//��ȡEEPROM����
			break;
			;
		}
		else
		{
			motor_array[0].position_value.tp[i]=((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | ((uint32_t)data[3]);
		}
		//���Դ���
		motor_array[0].position_value.tp[i]=0;
	}
	motor_array[0].position_value.if_tp_already=0x01;
		
	//3�ŵ��
	tmp_addr=0x00;                      //3�ŵ����ʼ��ַ
	for(i=0;i<3;i++)
	{
		if(iic_rw(rw_flag, tmp_addr + i*4,data,length)!=0)
		{
			//��ȡEEPROM����
			break;
			;
		}
		else
		{
			motor_array[2].position_value.tp[i]=((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | ((uint32_t)data[3]);
		}
		//���Դ���
		motor_array[2].position_value.tp[i]=0;
	}
	motor_array[2].position_value.if_tp_already=0x01;
	
	//4�ŵ��
	tmp_addr=0x00;                      //4�ŵ����ʼ��ַ
	for(i=0;i<3;i++)
	{
		if(iic_rw(rw_flag, tmp_addr + i*4,data,length)!=0)
		{
			//��ȡEEPROM����
			break;
			;
		}
		else
		{
			motor_array[3].position_value.tp[i]=((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | ((uint32_t)data[3]);
		}
		//���Դ���
		motor_array[3].position_value.tp[i]=0;
	}
	motor_array[3].position_value.if_tp_already=0x01;
	
	
	
	return 0;
}
//����������
MODBUS_LIST* modbus_list_gen(uint16_t count)
{
	MODBUS_LIST* tmp=(MODBUS_LIST*)pvPortMalloc(sizeof(MODBUS_LIST));
	MODBUS_LIST* head=tmp;
	if(count==1)
	{
		tmp->if_over=0;
		tmp->next=tmp;
		return tmp;
	}
	else
	{
		uint16_t i=0;
		tmp->if_over=0;
		MODBUS_LIST* loop_tmp;
		for(i=0;i<count - 1;i++)
		{
			loop_tmp=(MODBUS_LIST*)pvPortMalloc(sizeof(MODBUS_LIST));
			loop_tmp->if_over=0;
			head->next=loop_tmp;
			head=loop_tmp;
		}
		head->next=tmp;
		return tmp;
	}
}
//CRC��
static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};
//crc ����
uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen )
{
    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    int iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}


//��������
QUEUE_STRUCT command_seq[4]=
{
	{
		.property=1,                          //485 send
		.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
		.modbus_func=0x03,                    //������Ĵ���
		.modbus_addr_h=(uint8_t)(4004>>8),    //����ǰ����λ��
		.modbus_addr_l=(uint8_t)(4004&0xFF),  
		.modbus_data_len_h=0x00,
		.modbus_data_len_l=0x02,
		.modbus_property=1,
	},
	{
		.property=1,                            //485 send
		.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
		.modbus_func=0x10,                    //д����Ĵ���
		.modbus_addr_h=(uint8_t)(3202>>8),
		.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
		.modbus_data_len_h=0x00,
		.modbus_data_len_l=0x02,
		.modbus_data_byte=0x04,
		.modbus_data_1=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		.modbus_data_2=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		.modbus_data_3=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		.modbus_data_4=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
	},
	{
		.property=1,                            //485 send
		.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
		.modbus_func=0x10,                    //д����Ĵ���
		.modbus_addr_h=(uint8_t)(2040>>8),
		.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
		.modbus_data_len_h=0x00,
		.modbus_data_len_l=0x02,
		.modbus_data_byte=0x04,
		.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
		.modbus_data_2=0xFF,                      
		.modbus_data_3=0xFF,                      
		.modbus_data_4=0xFF,                      
	},
	{
		.property=1,                            //485 send
		.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
		.modbus_func=0x10,                    //д����Ĵ���
		.modbus_addr_h=(uint8_t)(2040>>8),
		.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
		.modbus_data_len_h=0x00,
		.modbus_data_len_l=0x02,
		.modbus_data_byte=0x04,
		.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
		.modbus_data_2=0x00,                      
		.modbus_data_3=0x00,                      
		.modbus_data_4=0x00,
	},
};
//0��ָ�ֹͣ���
int command_0(uint8_t* data,uint32_t para)
{
	uint8_t i=0;
	GPIO_PinState status;
	for(i=0;i<MAX_MOTOR_NUMBER+1;i++)
	{
		//�����GPIOʹ������͵�ƽ
		HAL_GPIO_WritePin(motor_array[i].gpio_output[ENABLE_MOTOR].gpio_port,motor_array[i].gpio_output[ENABLE_MOTOR].pin_number,GPIO_PIN_RESET);
		status=HAL_GPIO_ReadPin(motor_array[i].gpio_output[ENABLE_MOTOR].gpio_port,motor_array[i].gpio_output[ENABLE_MOTOR].pin_number);
		if(status!=GPIO_PIN_RESET)
		{
			if(para==1)
			{
				QUEUE_STRUCT tmp;
        tmp.can_command=0x00;          //ָֹͣ��
        tmp.can_if_ack=0x01;           //��ҪACK
				tmp.can_source=0x03;           //��ģ��
				tmp.can_target=0x00;
				tmp.can_priority=0x03;         //�����������֡
				tmp.can_if_last=0x00;
				tmp.can_if_return=0x00;
				tmp.length=4;
				tmp.data[0]=0x00;
				tmp.data[1]=0x00;
				tmp.data[2]=0x00;
				tmp.data[3]=ERROR_COMMAND_0_FAIL;
			  BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
				if(status_q!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 0 error to queue already");
					#endif
				}
				;
			}
			return ERROR_COMMAND_0_FAIL;
		}
	}
	if(para==1)
	{
		QUEUE_STRUCT tmp;
		tmp.property=0x00;             //can send
		tmp.can_command=0x00;          //ָֹͣ��
		tmp.can_if_ack=0x01;           //��ҪACK
		tmp.can_source=0x03;           //��ģ��
		tmp.can_target=0x00;
		tmp.can_priority=0x03;         //�����������֡
		tmp.can_if_last=0x00;
		tmp.can_if_return=0x00;
		tmp.length=4;
		tmp.data[0]=0x00;
		tmp.data[1]=0x00;
		tmp.data[2]=0x00;
		tmp.data[3]=0x00;
		BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
		if(status_q!=pdPASS)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","queue overflow");
			#endif
		}
		else
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","send command 0 success to queue already");
			#endif
		}
	}
	return 0;
}
int command_1(uint8_t* data,uint32_t para)
{
	GPIO_PinState status;
	uint8_t module=data[0];
	uint8_t action=data[1];
	uint8_t if_return=(para>>4)&0x01;
	GPIO_PinState ACT=GPIO_PIN_RESET;
	if(action==1)
	{
		ACT=GPIO_PIN_SET;
	}
	if(module==0)
	{
		HAL_GPIO_WritePin(GRATING_POWER_SW,GRATING_POWER_PIN,ACT);
		status=HAL_GPIO_ReadPin(GRATING_POWER_SW,GRATING_POWER_PIN);
		if(status!=ACT)
		{
			if(if_return==1)
			{
				QUEUE_STRUCT tmp;
				tmp.property=0x00;             //can send
				tmp.can_command=0x01;          //ָֹͣ��
				tmp.can_if_ack=0x01;           //��ҪACK
				tmp.can_source=0x03;           //��ģ��
				tmp.can_target=0x00;
				tmp.can_priority=0x03;         //�����������֡
				tmp.can_if_last=0x00;
				tmp.can_if_return=0x00;
				tmp.length=4;
				tmp.data[0]=0x00;
				tmp.data[1]=0x00;
				tmp.data[2]=0x00;
				tmp.data[3]=ERROR_COMMAND_1_FAIL;
				BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
				if(status_q!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 1 error to queue already");
					#endif
				}
			}
			return ERROR_COMMAND_1_FAIL;
		}
		if(if_return==1)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x01;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=0x00;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
	}		
	return 0;
}
int command_2(uint8_t* data,uint32_t para)
{
	uint8_t if_return=para;
	uint16_t len = EEPROM_CONFIG_LENGTH;
	uint16_t len_already=0;
	QUEUE_STRUCT tmp;
	while(len>8)
	{
		//ȡ��EEPROM����ѭ������
		while(iic_rw(0,len_already,tmp.data,8)!=0)
		{
			vTaskDelay(1);
		}
		//can send
		tmp.property=0;
		tmp.can_command=0x02;
		tmp.can_if_ack=0x01;
		tmp.can_source=0x03;
		tmp.can_target=0x00;
		tmp.can_priority=0x05;
		tmp.can_if_return=0x00;
		tmp.can_if_last=0x01;
		tmp.length=8;
		BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
		if(status_q!=pdPASS)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","queue overflow");
			#endif
		}
		else
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","send command 2 success to queue already");
			#endif
		}
		len-=8;
		len_already+=8;
	}
	//ȡ��EEPROM���ݷ���
	while(iic_rw(0,len_already,tmp.data,len)!=0)
	{
		vTaskDelay(1);
	}
	//can send
	tmp.property=0;
	tmp.can_command=0x02;
	tmp.can_if_ack=0x01;
	tmp.can_source=0x03;
	tmp.can_target=0x00;
	tmp.can_priority=0x05;
	tmp.can_if_return=0x00;
	tmp.can_if_last=0x00;
	tmp.length=len;
	BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
	if(status_q!=pdPASS)
	{
		#ifdef DEBUG_OUTPUT
		printf("%s\n","queue overflow");
		#endif
	}
	else
	{
		#ifdef DEBUG_OUTPUT
		printf("%s\n","send command 2 success to queue already");
		#endif
	}
	if(if_return==1)
	{
		//can send
		tmp.property=0;
		tmp.can_command=0x02;
		tmp.can_if_ack=0x01;
		tmp.can_source=0x03;
		tmp.can_target=0x00;
		tmp.can_priority=0x03;
		tmp.can_if_return=0x00;
		tmp.can_if_last=0x00;
		tmp.length=4;
		tmp.data[0]=0x00;
		tmp.data[1]=0x00;
		tmp.data[2]=0x00;
		tmp.data[3]=0x00;
		BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
		if(status_q!=pdPASS)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","queue overflow");
			#endif
		}
		else
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","send command 2 success to queue already");
			#endif
		}
	}
	return 0;
}
int command_3(uint8_t* data,uint32_t para)
{
	uint8_t if_return=(para>>4)&0x01;
	uint8_t if_last=(para>>5)&0x01;
	//uint16_t len = EEPROM_CONFIG_LENGTH;
	uint8_t left_length=para&0x0F;
	QUEUE_STRUCT tmp;
	//uint16_t addr=0;
	if(if_last==1)
	{
		if(left_length!=8)
		{
			if(if_return==1)
			{
				//can send
				tmp.property=0;
				tmp.can_command=0x03;
				tmp.can_if_ack=0x01;
				tmp.can_source=0x03;
				tmp.can_target=0x00;
				tmp.can_priority=0x03;
				tmp.can_if_return=0x00;
				tmp.can_if_last=0x00;
				tmp.length=4;
				tmp.data[0]=0x00;
				tmp.data[1]=0x00;
				tmp.data[2]=0x00;
				tmp.data[3]=ERROR_COMMAND_3_FAIL;
				BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
				if(status_q!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 2 success to queue already");
					#endif
				}
			}
		}
		else
		{
			while(iic_rw(1,iic_cache,data,8)!=0)
			{
				vTaskDelay(1);
			}
			iic_cache+=8;
		}
		
	}
	if(if_last==0)
	{
		if((iic_cache > EEPROM_CONFIG_LENGTH) || (EEPROM_CONFIG_LENGTH - iic_cache != left_length))
		{
			//���ݸ������󣬷��ش�����
			if(if_return==1)
			{
				//can send
				tmp.property=0;
				tmp.can_command=0x03;
				tmp.can_if_ack=0x01;
				tmp.can_source=0x03;
				tmp.can_target=0x00;
				tmp.can_priority=0x03;
				tmp.can_if_return=0x00;
				tmp.can_if_last=0x00;
				tmp.length=4;
				tmp.data[0]=0x00;
				tmp.data[1]=0x00;
				tmp.data[2]=0x00;
				tmp.data[3]=ERROR_COMMAND_3_FAIL;
				BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
				if(status_q!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 2 success to queue already");
					#endif
				}
			}
		}
		else{
			while(iic_rw(1,iic_cache,data,EEPROM_CONFIG_LENGTH - iic_cache)!=0)
			{
				vTaskDelay(1);
			}
			iic_cache=0;
			if(if_return==1)
			{
				//can send
				tmp.property=0;
				tmp.can_command=0x03;
				tmp.can_if_ack=0x01;
				tmp.can_source=0x03;
				tmp.can_target=0x00;
				tmp.can_priority=0x03;
				tmp.can_if_return=0x00;
				tmp.can_if_last=0x00;
				tmp.length=4;
				tmp.data[0]=0x00;
				tmp.data[1]=0x00;
				tmp.data[2]=0x00;
				tmp.data[3]=0x00;
				BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
				if(status_q!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 2 success to queue already");
					#endif
				}
			}
		}
	}
	return 0;
}
int command_4(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_5(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_6(uint8_t* data,uint32_t para)
{
	uint8_t data_len=(uint8_t)(para & 0x0F);
	uint8_t if_return=(para>>4)&0x01;
	uint8_t if_last=(para>>5)&0x01;
	if(if_last!=0 || data_len!=1)
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x06;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_6_FAIL;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_6_FAIL;
	}
	command_18(data,if_return);
	command_15(data,if_return);
	command_17(data,if_return);
	
	return 0;
}
int command_7(uint8_t* data,uint32_t para)
{
	uint8_t if_return=(para>>4)&0x01;
	int32_t period=(data[0] << 24)|(data[1]<<16)|(data[2]<<8)|data[3];
	if(period==0x00)
	{
		if(if_return==1)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x07;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_7_FAIL;
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 7 error to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_7_FAIL;
	}
	if(xTimerChangePeriod(broadcast_timer,period/portTICK_PERIOD_MS,50)!=pdPASS)
	{
		if(if_return==1)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x07;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_7_FAIL;
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 7 error to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_7_FAIL;
	}
	if(if_return==1)
	{
		QUEUE_STRUCT tmp;
		tmp.property=0x00;             //can send
		tmp.can_command=0x07;          //ָֹͣ��
		tmp.can_if_ack=0x01;           //��ҪACK
		tmp.can_source=0x03;           //��ģ��
		tmp.can_target=0x00;
		tmp.can_priority=0x03;         //�����������֡
		tmp.can_if_last=0x00;
		tmp.can_if_return=0x00;
		tmp.length=4;
		tmp.data[0]=0x00;
		tmp.data[1]=0x00;
		tmp.data[2]=0x00;
		tmp.data[3]=0x00;
	  portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &tmp, 0);
		if(status!=pdPASS)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","queue overflow");
			#endif
		}
		else
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","send command 7 succes to queue already");
			#endif
		}
	}
	return 0;
}
int command_8(uint8_t* data,uint32_t para)
{
	//�����������,д����
	uint8_t if_return=(para>>4)&0x01;
	//int32_t offset=(data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
	QUEUE_STRUCT tmp;
	if(data[0]>4 || data[0]==0)
	{
		if(data[0]==0x11 || data[0]==0x12 || data[0]==0x13
		|| data[0]==0x31 || data[0]==0x32 || data[0]==0x33
		|| data[0]==0x41 || data[0]==0x42 || data[0]==0x43)
		{
			//1�ŵ��ָ�������ֹͣ����������������
			if(data[0]==0x11)
			{
				//������������
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=1;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1008>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1008&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x01;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			if(data[0]==0x12)
			{
				//ֹͣ����������2
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=1;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1008>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1008&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x00;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			if(data[0]==0x13)
			{
				//������������
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=1;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1142>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1142&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x01;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			//ǰ��н��������ֹͣ������0
			if(data[0]==0x31)
			{
				//������������
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=3;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1008>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1008&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x01;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			if(data[0]==0x32)
			{
				//ֹͣ����������2
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=3;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1008>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1008&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x00;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			if(data[0]==0x33)
			{
				//������������
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=3;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1142>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1142&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x01;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			//���Ҽн����
			if(data[0]==0x41)
			{
				//������������
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=4;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1008>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1008&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x01;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			if(data[0]==0x42)
			{
				//ֹͣ����������2
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=4;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1008>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1008&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x00;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			if(data[0]==0x43)
			{
				//������������
				QUEUE_STRUCT enable_motor;
	
				enable_motor.property=1;                            //485 send
				enable_motor.modbus_addr=4;
				enable_motor.modbus_func=0x10;                      //д����Ĵ���
				enable_motor.modbus_addr_h=(uint8_t)(1142>>8);
				enable_motor.modbus_addr_l=(uint8_t)(1142&0xFF);                   //���485��ַ
				enable_motor.modbus_data_len_h=0x00;
				enable_motor.modbus_data_len_l=0x02;
				enable_motor.modbus_data_byte=0x04;
				enable_motor.modbus_data_1=0x00;
				enable_motor.modbus_data_2=0x01;
				enable_motor.modbus_data_3=0x00;
				enable_motor.modbus_data_4=0x00;
				
				//modbus_send_sub(enable_motor);
				portBASE_TYPE	status = xQueueSendToBack(send_queueHandle, &enable_motor, 0);
				if(status!=pdPASS)
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","queue overflow");
					#endif
				}
				else
				{
					#ifdef DEBUG_OUTPUT
					printf("%s\n","send command 7 succes to queue already");
					#endif
				}
				;
			}
			;
		}
		else{
			
		//��������
		if(if_return==1)
		{
			tmp.property=0x00;             //can send
			tmp.can_command=0x08;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_8_FAIL;
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 7 error to queue already");
				#endif
			}
		}
	}
		return ERROR_COMMAND_8_FAIL;
	
	}
	else
	{
		//���485���ͽṹ��
		/*
		tmp.property=1;                            //485 send
		tmp.modbus_addr=data[0];
		tmp.modbus_func=0x10;                      //д����Ĵ���
		tmp.modbus_addr_h=(uint8_t)(1008>>8);
		tmp.modbus_addr_l=(uint8_t)(1008&0xFF);                   //���485��ַ
		//tmp.modbus_addr_h=0x03;
		//tmp.modbus_addr_l=0xF2;
		tmp.modbus_data_len_h=0x00;
		tmp.modbus_data_len_l=0x02;
		tmp.modbus_data_byte=0x04;
		tmp.modbus_data_1=data[3];
		tmp.modbus_data_2=data[4];
		tmp.modbus_data_3=data[1];
		tmp.modbus_data_4=data[2];
		*/
		//�������
		command_seq[0].can_command=0x08;
		command_seq[0].modbus_addr=data[0];           //��ַ��ֵΪ�����
		command_seq[1].modbus_addr=data[0];
		command_seq[2].modbus_addr=data[0];
		command_seq[3].modbus_addr=data[0];
		
		//��䶯��Ŀ��
		command_seq[1].modbus_data_1=data[3];
		command_seq[1].modbus_data_2=data[4];
		command_seq[1].modbus_data_3=data[1];
		command_seq[1].modbus_data_4=data[2];
		
		//�����������ṹ��
		motor_array[data[0]-1].command.command_id=0x08;           //����id���Ϊ8
		motor_array[data[0]-1].command.command_status=0x01;       //����ִ����
		motor_array[data[0]-1].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
		
		
		//��������ѹ�����
		//��ȡ�����еĿ���λ������
		uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
		if(space_left<4)
		{
			//���Ͷ���������ֱ�ӷ��ش���,δ���
			;
		}
		else
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq[0], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}

			status = xQueueSendToBack(send_queueHandle, &command_seq[1], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq[2], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq[3], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
		
		
	}
	return 0;
}
int command_9(uint8_t* data,uint32_t para)
{
	//������ԣ���״̬
	uint8_t if_return=(para>>4)&0x01;
	//int32_t offset=(data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
	QUEUE_STRUCT tmp;
	if(data[0]>4 || data[0]==0)
	{
		//��������
		if(if_return==1)
		{
			tmp.property=0x00;             //can send
			tmp.can_command=0x09;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_8_FAIL;
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 7 error to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_8_FAIL;
	}
	else
	{
		//���CAN���ͽṹ�岢����
		tmp.property=0x00;             //can send
		tmp.can_command=0x09;          //ָֹͣ��
		tmp.can_if_ack=0x01;           //��ҪACK
		tmp.can_source=0x03;           //��ģ��
		tmp.can_target=0x00;
		tmp.can_priority=0x05;         //����֡
		tmp.can_if_last=0x00;
		tmp.can_if_return=0x00;
		tmp.length=4;
		tmp.data[0]=(uint8_t)((motor_array[data[0]-1].position_value.current_position>>24) & 0xFF);
		tmp.data[1]=(uint8_t)((motor_array[data[0]-1].position_value.current_position>>16) & 0xFF);
		tmp.data[2]=(uint8_t)((motor_array[data[0]-1].position_value.current_position>>8) & 0xFF);
		tmp.data[3]=(uint8_t)(motor_array[data[0]-1].position_value.current_position & 0xFF);
		portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp, 0);
		if(status!=pdPASS)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","queue overflow");
			#endif
		}
		else
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","send command 7 error to queue already");
			#endif
		}
		if(if_return==1)
		{
			tmp.property=0x00;             //can send
			tmp.can_command=0x09;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=0x00;
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 7 error to queue already");
				#endif
			}
		}
		
	}
	return 0;
}
int command_10(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_11(uint8_t* data,uint32_t para)
{
	//������������
	QUEUE_STRUCT command_seq_l1[4]=
	{
		{
			.property=1,                          //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x03,                    //������Ĵ���
			.modbus_addr_h=(uint8_t)(4004>>8),    //����ǰ����λ��
			.modbus_addr_l=(uint8_t)(4004&0xFF),  
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_property=1,
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//��������
	uint8_t data_len=(uint8_t)(para & 0x0F);
	uint8_t if_return=(para>>4)&0x01;
	uint8_t if_last=(para>>5)&0x01;
	if(if_last!=0x00 || data_len!=1 || data[0]>1)
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x0B;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_11_FAIL;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_11_FAIL;
	}
	if(motor_array[0].position_value.if_tp_already==0x01)
	{
		//��⵱ǰλ���Ƿ�ΪĿ��λ��
		if(data[0]==0x00 || data[0]==0x01)
		{
			if(__fabs(motor_array[0].position_value.tp[1+data[0]] - motor_array[0].position_value.current_position) < COMPLETE_JUDGE)
			{
				//��ǰλ���Ѿ������������
				if(if_return == 0x01)
				{
					QUEUE_STRUCT tmp;
					tmp.property=0x00;             //can send
					tmp.can_command=0x0B;          //ָֹͣ��
					tmp.can_if_ack=0x01;           //��ҪACK
					tmp.can_source=0x03;           //��ģ��
					tmp.can_target=0x00;
					tmp.can_priority=0x03;         //�����������֡
					tmp.can_if_last=0x00;
					tmp.can_if_return=0x00;
					tmp.length=4;
					tmp.data[0]=0x00;
					tmp.data[1]=0x01;              //ִ�н����1���������
					tmp.data[2]=0x01;              //����ţ� 1�����촰����
					tmp.data[3]=0x00;
					BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
					if(status_q!=pdPASS)
					{
						#ifdef DEBUG_OUTPUT
						printf("%s\n","queue overflow");
						#endif
					}
					else
					{
						#ifdef DEBUG_OUTPUT
						printf("%s\n","send command 1 success to queue already");
						#endif
					}
				}
				return 0;
			}
		}
		//���Ͷ�������
		//�������
		command_seq_l1[0].can_command=0x0B;
		command_seq_l1[0].modbus_addr=0x01;           //��ַ��ֵΪ1�ţ��촰���
		command_seq_l1[1].modbus_addr=0x01;
		command_seq_l1[2].modbus_addr=0x01;
		command_seq_l1[3].modbus_addr=0x01;
		
		//��䶯��Ŀ��
		if(data[0]==0x00)//�ر��촰
		{
			command_seq_l1[1].modbus_data_1=(uint8_t)((motor_array[0].position_value.tp[1] >> 8) & 0xFF);
			command_seq_l1[1].modbus_data_2=(uint8_t)((motor_array[0].position_value.tp[1]) & 0xFF);
			command_seq_l1[1].modbus_data_3=(uint8_t)((motor_array[0].position_value.tp[1] >> 24) & 0xFF);
			command_seq_l1[1].modbus_data_4=(uint8_t)((motor_array[0].position_value.tp[1] >> 16) & 0xFF);
		}
		if(data[0]==0x01)
		{
			command_seq_l1[1].modbus_data_1=(uint8_t)((motor_array[0].position_value.tp[2] >> 8) & 0xFF);
		  command_seq_l1[1].modbus_data_2=(uint8_t)((motor_array[0].position_value.tp[2]) & 0xFF);
		  command_seq_l1[1].modbus_data_3=(uint8_t)((motor_array[0].position_value.tp[2] >> 24) & 0xFF);
		  command_seq_l1[1].modbus_data_4=(uint8_t)((motor_array[0].position_value.tp[2] >> 16) & 0xFF);
		}
			
		
		//�����������ṹ��
		motor_array[0].command.command_id=0x0B;           //����id���Ϊ8
		motor_array[0].command.command_status=0x01;       //����ִ����
		motor_array[0].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
		
		
		//��������ѹ�����
		//��ȡ�����еĿ���λ������
		uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
		if(space_left<4)
		{
			//���Ͷ���������ֱ�ӷ��ش���,δ���
			;
		}
		else
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq_l1[0], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}

			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[1], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[2], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[3], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
	}
	else
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x0B;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_11_EEPROM_ERROR;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_11_EEPROM_ERROR;
	}
	return 0;
}
int command_12(uint8_t* data,uint32_t para)
{
	QUEUE_STRUCT tmp;
	tmp.property=0x00;             //can send
	tmp.can_command=0x0C;          //ָֹͣ��
	tmp.can_if_ack=0x01;           //��ҪACK
	tmp.can_source=0x03;           //��ģ��
	tmp.can_target=0x00;
	tmp.can_priority=0x03;         //�����������֡
	tmp.can_if_last=0x00;
	tmp.can_if_return=0x00;
	tmp.length=4;
	tmp.data[0]=0x00;
	tmp.data[1]=0x00;
	tmp.data[2]=0x00;
	tmp.data[3]=ERROR_COMMAND_12_FAIL;
	BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
	if(status_q!=pdPASS)
	{
		#ifdef DEBUG_OUTPUT
		printf("%s\n","queue overflow");
		#endif
	}
	else
	{
		#ifdef DEBUG_OUTPUT
		printf("%s\n","send command 1 success to queue already");
		#endif
	}
	/*
	//������������
	QUEUE_STRUCT command_seq_l1[4]=
	{
		{
			.property=1,                          //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x03,                    //������Ĵ���
			.modbus_addr_h=(uint8_t)(4004>>8),    //����ǰ����λ��
			.modbus_addr_l=(uint8_t)(4004&0xFF),  
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_property=1,
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//��������
	uint8_t data_len=(uint8_t)(para & 0x0F);
	uint8_t if_return=(para>>4)&0x01;
	uint8_t if_last=(para>>5)&0x01;
	if(if_last!=0x00 || data_len!=1 || data[0]>1)
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x0C;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_12_FAIL;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_12_FAIL;
	}
	if(motor_array[1].position_value.if_tp_already==0x01)
	{
		//��⵱ǰλ���Ƿ�ΪĿ��λ��
		if(data[0]==0x00 || data[0]==0x01)
		{
			if(__fabs(motor_array[1].position_value.tp[1+data[0]] - motor_array[1].position_value.current_position) < COMPLETE_JUDGE)
			{
				//��ǰλ���Ѿ������������
				if(if_return == 0x01)
				{
					QUEUE_STRUCT tmp;
					tmp.property=0x00;             //can send
					tmp.can_command=0x0C;          //ָֹͣ��
					tmp.can_if_ack=0x01;           //��ҪACK
					tmp.can_source=0x03;           //��ģ��
					tmp.can_target=0x00;
					tmp.can_priority=0x03;         //�����������֡
					tmp.can_if_last=0x00;
					tmp.can_if_return=0x00;
					tmp.length=4;
					tmp.data[0]=0x00;
					tmp.data[1]=0x01;              //ִ�н����1���������
					tmp.data[2]=0x02;              //����ţ� 1�����촰����
					tmp.data[3]=0x00;
					BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
					if(status_q!=pdPASS)
					{
						#ifdef DEBUG_OUTPUT
						printf("%s\n","queue overflow");
						#endif
					}
					else
					{
						#ifdef DEBUG_OUTPUT
						printf("%s\n","send command 1 success to queue already");
						#endif
					}
				}
				return 0;
			}
		}
		//���Ͷ�������
		//�������
		command_seq_l1[0].can_command=0x0C;
		command_seq_l1[0].modbus_addr=0x02;           //��ַ��ֵΪ2�ţ��촰�ҵ��
		command_seq_l1[1].modbus_addr=0x02;
		command_seq_l1[2].modbus_addr=0x02;
		command_seq_l1[3].modbus_addr=0x02;
		
		//��䶯��Ŀ��
		if(data[0]==0x00)//�ر��촰
		{
			command_seq_l1[1].modbus_data_1=(uint8_t)((motor_array[1].position_value.tp[1] >> 8) & 0xFF);
			command_seq_l1[1].modbus_data_2=(uint8_t)((motor_array[1].position_value.tp[1]) & 0xFF);
			command_seq_l1[1].modbus_data_3=(uint8_t)((motor_array[1].position_value.tp[1] >> 24) & 0xFF);
			command_seq_l1[1].modbus_data_4=(uint8_t)((motor_array[1].position_value.tp[1] >> 16) & 0xFF);
		}
		if(data[0]==0x01)
		{
			command_seq_l1[1].modbus_data_1=(uint8_t)((motor_array[1].position_value.tp[2] >> 8) & 0xFF);
		  command_seq_l1[1].modbus_data_2=(uint8_t)((motor_array[1].position_value.tp[2]) & 0xFF);
		  command_seq_l1[1].modbus_data_3=(uint8_t)((motor_array[1].position_value.tp[2] >> 24) & 0xFF);
		  command_seq_l1[1].modbus_data_4=(uint8_t)((motor_array[1].position_value.tp[2] >> 16) & 0xFF);
		}
			
		
		//�����������ṹ��
		motor_array[1].command.command_id=0x0C;           //����id���Ϊ8
		motor_array[1].command.command_status=0x01;       //����ִ����
		motor_array[1].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
		
		
		//��������ѹ�����
		//��ȡ�����еĿ���λ������
		uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
		if(space_left<4)
		{
			//���Ͷ���������ֱ�ӷ��ش���,δ���
			;
		}
		else
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq_l1[0], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}

			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[1], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[2], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[3], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
	}
	else
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x0C;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_12_EEPROM_ERROR;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_12_EEPROM_ERROR;
	}
	*/
	return 0;
}
int command_13(uint8_t* data,uint32_t para)
{
	//������������
	QUEUE_STRUCT command_seq_l1[4]=
	{
		{
			.property=1,                          //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x03,                    //������Ĵ���
			.modbus_addr_h=(uint8_t)(4004>>8),    //����ǰ����λ��
			.modbus_addr_l=(uint8_t)(4004&0xFF),  
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_property=1,
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//��������
	uint8_t data_len=(uint8_t)(para & 0x0F);
	uint8_t if_return=(para>>4)&0x01;
	uint8_t if_last=(para>>5)&0x01;
	if(if_last!=0x00 || data_len!=1 || data[0]>1)
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x0D;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_13_FAIL;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_13_FAIL;
	}
	if(motor_array[2].position_value.if_tp_already==0x01)
	{
		//��⵱ǰλ���Ƿ�ΪĿ��λ��
		if(data[0]==0x00 || data[0]==0x01)
		{
			if(__fabs(motor_array[2].position_value.tp[1+data[0]] - motor_array[2].position_value.current_position) < COMPLETE_JUDGE)
			{
				//��ǰλ���Ѿ������������
				if(if_return == 0x01)
				{
					QUEUE_STRUCT tmp;
					tmp.property=0x00;             //can send
					tmp.can_command=0x0D;          //ָֹͣ��
					tmp.can_if_ack=0x01;           //��ҪACK
					tmp.can_source=0x03;           //��ģ��
					tmp.can_target=0x00;
					tmp.can_priority=0x03;         //�����������֡
					tmp.can_if_last=0x00;
					tmp.can_if_return=0x00;
					tmp.length=4;
					tmp.data[0]=0x00;
					tmp.data[1]=0x00;              //ִ�н����1���������
					tmp.data[2]=0x00;              //����ţ� 1�����촰����
					tmp.data[3]=0x00;
					BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
					if(status_q!=pdPASS)
					{
						#ifdef DEBUG_OUTPUT
						printf("%s\n","queue overflow");
						#endif
					}
					else
					{
						#ifdef DEBUG_OUTPUT
						printf("%s\n","send command 1 success to queue already");
						#endif
					}
				}
				return 0;
			}
		}
		//���Ͷ�������
		//�������
		command_seq_l1[0].can_command=0x0D;
		command_seq_l1[0].modbus_addr=0x03;           //��ַ��ֵΪ3�ţ�ǰ��н����
		command_seq_l1[1].modbus_addr=0x03;
		command_seq_l1[2].modbus_addr=0x03;
		command_seq_l1[3].modbus_addr=0x03;
		
		//��䶯��Ŀ��
		if(data[0]==0x00)//�ر��촰
		{
			command_seq_l1[1].modbus_data_1=(uint8_t)((motor_array[2].position_value.tp[1] >> 8) & 0xFF);
			command_seq_l1[1].modbus_data_2=(uint8_t)((motor_array[2].position_value.tp[1]) & 0xFF);
			command_seq_l1[1].modbus_data_3=(uint8_t)((motor_array[2].position_value.tp[1] >> 24) & 0xFF);
			command_seq_l1[1].modbus_data_4=(uint8_t)((motor_array[2].position_value.tp[1] >> 16) & 0xFF);
		}
		if(data[0]==0x01)
		{
			command_seq_l1[1].modbus_data_1=(uint8_t)((motor_array[2].position_value.tp[2] >> 8) & 0xFF);
		  command_seq_l1[1].modbus_data_2=(uint8_t)((motor_array[2].position_value.tp[2]) & 0xFF);
		  command_seq_l1[1].modbus_data_3=(uint8_t)((motor_array[2].position_value.tp[2] >> 24) & 0xFF);
		  command_seq_l1[1].modbus_data_4=(uint8_t)((motor_array[2].position_value.tp[2] >> 16) & 0xFF);
		}
			
		
		//�����������ṹ��
		motor_array[2].command.command_id=0x0D;           //����id���Ϊ8
		motor_array[2].command.command_status=0x01;       //����ִ����
		motor_array[2].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
		
		
		//��������ѹ�����
		//��ȡ�����еĿ���λ������
		uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
		if(space_left<4)
		{
			//���Ͷ���������ֱ�ӷ��ش���,δ���
			;
		}
		else
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq_l1[0], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}

			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[1], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[2], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[3], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
	}
	else
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x0D;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_13_EEPROM_ERROR;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_13_EEPROM_ERROR;
	}
	return 0;
}
int command_14(uint8_t* data,uint32_t para)
{
	//������������
	QUEUE_STRUCT command_seq_l1[4]=
	{
		{
			.property=1,                          //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x03,                    //������Ĵ���
			.modbus_addr_h=(uint8_t)(4004>>8),    //����ǰ����λ��
			.modbus_addr_l=(uint8_t)(4004&0xFF),  
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_property=1,
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=0,                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//��������
	uint8_t data_len=(uint8_t)(para & 0x0F);
	uint8_t if_return=(para>>4)&0x01;
	uint8_t if_last=(para>>5)&0x01;
	if(if_last!=0x00 || data_len!=1 || data[0]>1)
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x0E;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_14_FAIL;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_14_FAIL;
	}
	if(motor_array[3].position_value.if_tp_already==0x01)
	{
		//��⵱ǰλ���Ƿ�ΪĿ��λ��
		if(data[0]==0x00 || data[0]==0x01)
		{
			if(__fabs(motor_array[3].position_value.tp[1+data[0]] - motor_array[3].position_value.current_position) < COMPLETE_JUDGE)
			{
				//��ǰλ���Ѿ������������
				if(if_return == 0x01)
				{
					QUEUE_STRUCT tmp;
					tmp.property=0x00;             //can send
					tmp.can_command=0x0E;          //ָֹͣ��
					tmp.can_if_ack=0x01;           //��ҪACK
					tmp.can_source=0x03;           //��ģ��
					tmp.can_target=0x00;
					tmp.can_priority=0x03;         //�����������֡
					tmp.can_if_last=0x00;
					tmp.can_if_return=0x00;
					tmp.length=4;
					tmp.data[0]=0x00;
					tmp.data[1]=0x00;              //ִ�н����1���������
					tmp.data[2]=0x00;              //����ţ� 1�����촰����
					tmp.data[3]=0x00;
					BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
					if(status_q!=pdPASS)
					{
						#ifdef DEBUG_OUTPUT
						printf("%s\n","queue overflow");
						#endif
					}
					else
					{
						#ifdef DEBUG_OUTPUT
						printf("%s\n","send command 1 success to queue already");
						#endif
					}
				}
				return 0;
			}
		}
		//���Ͷ�������
		//�������
		command_seq_l1[0].can_command=0x0E;
		command_seq_l1[0].modbus_addr=0x04;           //��ַ��ֵΪ4�ţ����Ҽн����
		command_seq_l1[1].modbus_addr=0x04;
		command_seq_l1[2].modbus_addr=0x04;
		command_seq_l1[3].modbus_addr=0x04;
		
		//��䶯��Ŀ��
		if(data[0]==0x00)//�ر��촰
		{
			command_seq_l1[1].modbus_data_1=(uint8_t)((motor_array[3].position_value.tp[1] >> 8) & 0xFF);
			command_seq_l1[1].modbus_data_2=(uint8_t)((motor_array[3].position_value.tp[1]) & 0xFF);
			command_seq_l1[1].modbus_data_3=(uint8_t)((motor_array[3].position_value.tp[1] >> 24) & 0xFF);
			command_seq_l1[1].modbus_data_4=(uint8_t)((motor_array[3].position_value.tp[1] >> 16) & 0xFF);
		}
		if(data[0]==0x01)
		{
			command_seq_l1[1].modbus_data_1=(uint8_t)((motor_array[3].position_value.tp[2] >> 8) & 0xFF);
		  command_seq_l1[1].modbus_data_2=(uint8_t)((motor_array[3].position_value.tp[2]) & 0xFF);
		  command_seq_l1[1].modbus_data_3=(uint8_t)((motor_array[3].position_value.tp[2] >> 24) & 0xFF);
		  command_seq_l1[1].modbus_data_4=(uint8_t)((motor_array[3].position_value.tp[2] >> 16) & 0xFF);
		}
			
		
		//�����������ṹ��
		motor_array[3].command.command_id=0x0E;           //����id���Ϊ8
		motor_array[3].command.command_status=0x01;       //����ִ����
		motor_array[3].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
		
		
		//��������ѹ�����
		//��ȡ�����еĿ���λ������
		uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
		if(space_left<4)
		{
			//���Ͷ���������ֱ�ӷ��ش���,δ���
			;
		}
		else
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq_l1[0], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}

			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[1], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[2], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
			
			status = xQueueSendToBack(send_queueHandle, &command_seq_l1[3], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
	}
	else
	{
		if(if_return == 0x01)
		{
			QUEUE_STRUCT tmp;
			tmp.property=0x00;             //can send
			tmp.can_command=0x0E;          //ָֹͣ��
			tmp.can_if_ack=0x01;           //��ҪACK
			tmp.can_source=0x03;           //��ģ��
			tmp.can_target=0x00;
			tmp.can_priority=0x03;         //�����������֡
			tmp.can_if_last=0x00;
			tmp.can_if_return=0x00;
			tmp.length=4;
			tmp.data[0]=0x00;
			tmp.data[1]=0x00;
			tmp.data[2]=0x00;
			tmp.data[3]=ERROR_COMMAND_14_EEPROM_ERROR;
			BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
			if(status_q!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 1 success to queue already");
				#endif
			}
		}
		return ERROR_COMMAND_14_EEPROM_ERROR;
	}
	return 0;
}
int command_15(uint8_t* data,uint32_t para)
{
	//�촰�����Լ�
	//������������
	//��һ�ζ������У�tp0λ��
	QUEUE_STRUCT command_seq_l1[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //����ţ�1���촰���
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[0].position_value.tp[0] >> 8) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[0].position_value.tp[0]) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[0].position_value.tp[0] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[0].position_value.tp[0] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//�ڶ���
	QUEUE_STRUCT command_seq_l2[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�����,1
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[0].position_value.tp[1] >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[0].position_value.tp[1]) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[0].position_value.tp[1] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[0].position_value.tp[1] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//������
	QUEUE_STRUCT command_seq_l3[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�����,1,�촰���
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[0].position_value.tp[2] >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[0].position_value.tp[2]) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[0].position_value.tp[2] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[0].position_value.tp[2] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	QUEUE_STRUCT command_seq_l0[9];
	//�������
	uint8_t command_seq_counter=0;
	if(__fabs(motor_array[0].position_value.tp[0] - motor_array[0].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l1,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else{
		motor_array[0].self_check_counter++;
	}
	if(__fabs(motor_array[0].position_value.tp[1] - motor_array[0].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l2,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else
	{
		motor_array[0].self_check_counter++;
	}
	if(__fabs(motor_array[0].position_value.tp[2] - motor_array[0].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l3,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else
	{
		motor_array[0].self_check_counter++;
	}
	//��������
	uint8_t if_return=para;     //�Լ�����û�в���
	//��������ѹ�����
	//��ȡ�����еĿ���λ������
	uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
	//��䷵��ԭ��λ�õ���������
	QUEUE_STRUCT tmp_last[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�����,1,�촰���
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[0].position_value.current_position >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[0].position_value.current_position) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[0].position_value.current_position >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[0].position_value.current_position >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x01,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	if(space_left<4)
	{
		//���Ͷ���������ֱ�ӷ��ش���,δ���
		;
	}
	else
	{
		uint8_t isend=0;
		for(isend=0;isend<3*command_seq_counter;isend++)
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq_l0[isend], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
		//ѹ���ʼλ��������
		//ѹ�뷢�Ͷ���
		for(isend=0;isend<3;isend++)
		{
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp_last[isend], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
	}
	//��������״̬Ϊ1
	motor_array[0].command.command_id=0x0F;           //����id���Ϊ8
	motor_array[0].command.command_status=0x01;       //����ִ����
	motor_array[0].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
	//��ʱ����Ϊ8��ʵ���������Ҫ����tp�͵�ǰλ�ü���,����λ�ö��Ǵ���8�������أ������촰����ȫ�رյ㲻Ϊ��翪�ص�ʱ���촰��14�������أ�����Ҳ�����
	motor_array[0].self_check_counter=8;
	return 0;
}


int command_16(uint8_t* data,uint32_t para)
{
	QUEUE_STRUCT tmp;
	tmp.property=0x00;             //can send
	tmp.can_command=0x10;          //ָֹͣ��
	tmp.can_if_ack=0x01;           //��ҪACK
	tmp.can_source=0x03;           //��ģ��
	tmp.can_target=0x00;
	tmp.can_priority=0x03;         //�����������֡
	tmp.can_if_last=0x00;
	tmp.can_if_return=0x00;
	tmp.length=4;
	tmp.data[0]=0x00;
	tmp.data[1]=0x00;
	tmp.data[2]=0x00;
	tmp.data[3]=ERROR_COMMAND_16_FAIL;  //û��16��ָ����
	BaseType_t status_q = xQueueSendToBack(send_queueHandle, &tmp, 0);
	if(status_q!=pdPASS)
	{
		#ifdef DEBUG_OUTPUT
		printf("%s\n","queue overflow");
		#endif
	}
	else
	{
		#ifdef DEBUG_OUTPUT
		printf("%s\n","send command 1 success to queue already");
		#endif
	}
	/*
	//�촰�ҵ���Լ�
	//������������
	//��һ�ζ������У�tp0λ��
	QUEUE_STRUCT command_seq_l1[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //����ţ�2���촰�ҵ��
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[1].position_value.tp[0] >> 8) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[1].position_value.tp[0]) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[1].position_value.tp[0] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[1].position_value.tp[0] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//�ڶ���
	QUEUE_STRUCT command_seq_l2[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�����,2
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[1].position_value.tp[1] >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[1].position_value.tp[1]) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[1].position_value.tp[1] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[1].position_value.tp[1] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//������
	QUEUE_STRUCT command_seq_l3[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�����,2,�촰���
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[1].position_value.tp[2] >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[1].position_value.tp[2]) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[1].position_value.tp[2] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[1].position_value.tp[2] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	QUEUE_STRUCT command_seq_l0[9];
	//�������
	uint8_t command_seq_counter=0;
	if(__fabs(motor_array[1].position_value.tp[0] - motor_array[1].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l1,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else{
		motor_array[1].self_check_counter++;
	}
	if(__fabs(motor_array[1].position_value.tp[1] - motor_array[1].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l2,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else
	{
		motor_array[1].self_check_counter++;
	}
	if(__fabs(motor_array[1].position_value.tp[2] - motor_array[1].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l3,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else
	{
		motor_array[1].self_check_counter++;
	}
	//��������
	uint8_t if_return=para;     //�Լ�����û�в���
	//��������ѹ�����
	//��ȡ�����еĿ���λ������
	uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
	//��䷵��ԭ��λ�õ���������
	QUEUE_STRUCT tmp_last[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�����,1,�촰���
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[1].position_value.current_position >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[1].position_value.current_position) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[1].position_value.current_position >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[1].position_value.current_position >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x02,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	                
	if(space_left<4)
	{
		//���Ͷ���������ֱ�ӷ��ش���,δ���
		;
	}
	else
	{
		uint8_t isend=0;
		for(isend=0;isend<3*command_seq_counter;isend++)
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq_l0[isend], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
		//ѹ���ʼλ��������
		//ѹ�뷢�Ͷ���
		for(isend=0;isend<3;isend++)
		{
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp_last[isend], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
	}
	//��������״̬Ϊ1
	motor_array[1].command.command_id=0x10;           //����id���Ϊ8
	motor_array[1].command.command_status=0x01;       //����ִ����
	motor_array[1].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
	//��ʱ����Ϊ8��ʵ���������Ҫ����tp�͵�ǰλ�ü���,����λ�ö��Ǵ���8�������أ������촰����ȫ�رյ㲻Ϊ��翪�ص�ʱ���촰��14�������أ�����Ҳ�����
	motor_array[1].self_check_counter=8;
	*/
	return 0;
}
int command_17(uint8_t* data,uint32_t para)
{
	//ǰ��н�����Լ�
	//������������
	//��һ�ζ������У�tp0λ��
	QUEUE_STRUCT command_seq_l1[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //����ţ�3��ǰ��н����
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[2].position_value.tp[0] >> 8) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[2].position_value.tp[0]) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[2].position_value.tp[0] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[2].position_value.tp[0] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//�ڶ���
	QUEUE_STRUCT command_seq_l2[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�����,3��ǰ��н����
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[2].position_value.tp[1] >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[2].position_value.tp[1]) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[2].position_value.tp[1] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[2].position_value.tp[1] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//������
	QUEUE_STRUCT command_seq_l3[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�����,3��ǰ��н����
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[2].position_value.tp[2] >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[2].position_value.tp[2]) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[2].position_value.tp[2] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[2].position_value.tp[2] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	QUEUE_STRUCT command_seq_l0[9];
	//�������
	uint8_t command_seq_counter=0;
	if(__fabs(motor_array[2].position_value.tp[0] - motor_array[2].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l1,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else{
		motor_array[2].self_check_counter++;
	}
	if(__fabs(motor_array[2].position_value.tp[1] - motor_array[2].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l2,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else
	{
		motor_array[2].self_check_counter++;
	}
	if(__fabs(motor_array[2].position_value.tp[2] - motor_array[2].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l3,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else
	{
		motor_array[2].self_check_counter++;
	}
	//��������
	uint8_t if_return=para;     //�Լ�����û�в���
	//��������ѹ�����
	//��ȡ�����еĿ���λ������
	uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
	//��䷵��ԭ��λ�õ���������
	QUEUE_STRUCT tmp_last[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�����,3,ǰ��н����
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[2].position_value.current_position >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[2].position_value.current_position) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[2].position_value.current_position >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[2].position_value.current_position >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x03,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	                
	if(space_left<4)
	{
		//���Ͷ���������ֱ�ӷ��ش���,δ���
		;
	}
	else
	{
		uint8_t isend=0;
		for(isend=0;isend<3*command_seq_counter;isend++)
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq_l0[isend], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
		//ѹ���ʼλ��������
		//ѹ�뷢�Ͷ���
		for(isend=0;isend<3;isend++)
		{
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp_last[isend], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
	}
	//��������״̬Ϊ1
	motor_array[2].command.command_id=0x11;           //����id���Ϊ8
	motor_array[2].command.command_status=0x01;       //����ִ����
	motor_array[2].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
	//��ʱ����Ϊ8��ʵ���������Ҫ����tp�͵�ǰλ�ü���,����λ�ö��Ǵ���8�������أ������촰����ȫ�رյ㲻Ϊ��翪�ص�ʱ���촰��14�������أ�����Ҳ�����
	motor_array[2].self_check_counter=8;
	return 0;
}
int command_18(uint8_t* data,uint32_t para)
{
	//���Ҽн�����Լ�
	//������������
	//��һ�ζ������У�tp0λ��
	QUEUE_STRUCT command_seq_l1[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //����ţ�4�����Ҽн����
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[3].position_value.tp[0] >> 8) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[3].position_value.tp[0]) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[3].position_value.tp[0] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[3].position_value.tp[0] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//�ڶ���
	QUEUE_STRUCT command_seq_l2[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�����,4�����Ҽн����
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[3].position_value.tp[1] >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[3].position_value.tp[1]) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[3].position_value.tp[1] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[3].position_value.tp[1] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	//������
	QUEUE_STRUCT command_seq_l3[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�����,4�����Ҽн����
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[3].position_value.tp[2] >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[3].position_value.tp[2]) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[3].position_value.tp[2] >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[3].position_value.tp[2] >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	QUEUE_STRUCT command_seq_l0[9];
	//�������
	uint8_t command_seq_counter=0;
	if(__fabs(motor_array[3].position_value.tp[0] - motor_array[3].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l1,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else{
		motor_array[3].self_check_counter++;
	}
	if(__fabs(motor_array[3].position_value.tp[1] - motor_array[3].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l2,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else
	{
		motor_array[3].self_check_counter++;
	}
	if(__fabs(motor_array[3].position_value.tp[2] - motor_array[3].position_value.current_position)>COMPLETE_JUDGE)
	{
		memcpy(&command_seq_l0[3*command_seq_counter],command_seq_l3,3*sizeof(QUEUE_STRUCT));
		command_seq_counter++;
	}
	else
	{
		motor_array[3].self_check_counter++;
	}
	//��������
	uint8_t if_return=para;     //�Լ�����û�в���
	//��������ѹ�����
	//��ȡ�����еĿ���λ������
	uint32_t space_left=uxQueueSpacesAvailable(send_queueHandle);
	//��䷵��ԭ��λ�õ���������
	QUEUE_STRUCT tmp_last[3]=
	{
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�����,4,���Ҽн����
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(3202>>8),
			.modbus_addr_l=(uint8_t)(3202&0xFF),        //дĿ��λ��
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=(uint8_t)((motor_array[3].position_value.current_position >> 8) & 0xFF),                       //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_2=(uint8_t)((motor_array[3].position_value.current_position) & 0xFF),                            //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_3=(uint8_t)((motor_array[3].position_value.current_position >> 24) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
			.modbus_data_4=(uint8_t)((motor_array[3].position_value.current_position >> 16) & 0xFF),                      //�ȸ�ֵΪ�����е���ֵ����ǰλ�ö�ȡ�ɹ����޸�ֵ
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0xFF,                      //ʹ�ܼĴ���ȫ��дΪFF
			.modbus_data_2=0xFF,                      
			.modbus_data_3=0xFF,                      
			.modbus_data_4=0xFF,                      
		},
		{
			.property=1,                            //485 send
			.modbus_addr=0x04,                       //�������Ҫ���������еĵ���Ÿ�ֵ
			.modbus_func=0x10,                    //д����Ĵ���
			.modbus_addr_h=(uint8_t)(2040>>8),
			.modbus_addr_l=(uint8_t)(2040&0xFF),        //дʹ�ܼĴ���
			.modbus_data_len_h=0x00,
			.modbus_data_len_l=0x02,
			.modbus_data_byte=0x04,
			.modbus_data_1=0x00,                      //ʹ�ܼĴ���ȫ��дΪ00
			.modbus_data_2=0x00,                      
			.modbus_data_3=0x00,                      
			.modbus_data_4=0x00,
		},
	};
	                
	if(space_left<4)
	{
		//���Ͷ���������ֱ�ӷ��ش���,δ���
		;
	}
	else
	{
		uint8_t isend=0;
		for(isend=0;isend<3*command_seq_counter;isend++)
		{
			//ѹ�뷢�Ͷ���
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &command_seq_l0[isend], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
		//ѹ���ʼλ��������
		//ѹ�뷢�Ͷ���
		for(isend=0;isend<3;isend++)
		{
			portBASE_TYPE status = xQueueSendToBack(send_queueHandle, &tmp_last[isend], 0);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send command 8 error to queue already");
				#endif
			}
		}
	}
	//��������״̬Ϊ1
	motor_array[3].command.command_id=0x12;           //����id���Ϊ18
	motor_array[3].command.command_status=0x01;       //����ִ����
	motor_array[3].command.if_return=if_return;       //ִ������Ƿ���Ҫ�ظ�֡
	//��ʱ����Ϊ8��ʵ���������Ҫ����tp�͵�ǰλ�ü���,����λ�ö��Ǵ���8�������أ������촰����ȫ�رյ㲻Ϊ��翪�ص�ʱ���촰��14�������أ�����Ҳ�����
	motor_array[3].self_check_counter=8;
	return 0;
}
int command_19(uint8_t* data,uint32_t para)
{
	//��ת�ǶȻ�ȡ
	return 0;
}
int command_20(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_21(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_22(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_23(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_24(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_25(uint8_t* data,uint32_t para)
{
	return 0;
}
int command_26(uint8_t* data,uint32_t para)
{
	return 0;
}

//�����������
void result_parse_1(uint8_t* data, uint8_t num)
{
	//���λ�ö�ȡ
	if(num==1 || num>4)//���ε�2�ŵ��
	{
		;
	}
	else
	{
		motor_array[num].position_value.current_position=((uint32_t)data[0]<<8) | (uint32_t)data[1] | ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16);
	}
	return;
}
void result_parse_2(uint8_t* data, uint8_t num)
{
	//����ٶȶ�ȡ
	if(num==1 || num>4)
	{
		;
	}
	else
	{
		motor_array[num].speed_value.current_speed=((uint32_t)data[0]<<8) | (uint32_t)data[1] | ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16);
	}
	return;
}
void result_parse_3(uint8_t* data, uint8_t num)
{
	//���Ť�ض�ȡ
	if(num==1 || num>4)
	{
		;
	}
	else
	{
		motor_array[num].torque_value.current_torque=((uint32_t)data[0]<<8) | (uint32_t)data[1] | ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16);
	}
	return;
}
void result_parse_4(uint8_t* data, uint8_t num)
{
	//���������
	if(num==1 || num>4)
	{
		;
	}
	else
	{
		motor_array[num].motor_error_code=((uint32_t)data[0]<<8) | (uint32_t)data[1] | ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16);
	}
	return;
}
void result_parse_5(uint8_t* data, uint8_t num)
{
	//����¶�
	if(num==1 || num>4)
	{
		;
	}
	else
	{
		motor_array[num].temperature=((uint32_t)data[0]<<8) | (uint32_t)data[1] | ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16);
	}
	return;
}
void result_parse_6(uint8_t* data, uint8_t num)
{
	//�������������
	if(num==1 || num>4)
	{
		;
	}
	else
	{
		motor_array[num].position_value.remain_position=((uint32_t)data[0]<<8) | (uint32_t)data[1] | ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16);
	}
	return;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//������Ϣ���
uint8_t tklog[500]={0};
uint32_t ulHighFrequencyTimerTicks=0;
uint32_t queuespace=0;

CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

//�ź�������
//λ�û�ȡ�ź���
StaticSemaphore_t semaPositionGet;
SemaphoreHandle_t xSemaphorePositionGet = NULL;
//λ�������ź���
StaticSemaphore_t semaPositionSet;
SemaphoreHandle_t xSemaphorePositionSet = NULL;
//��������ȡ�ź�����������λ���أ�0�㿪�أ����俪��
StaticSemaphore_t semaSwitchGet;
SemaphoreHandle_t xSemaphoreSwitchGet = NULL;
//��դ״̬��ȡ�ź���
StaticSemaphore_t semaGratingGet;
SemaphoreHandle_t xSemaphoreGratingGet = NULL;

//modbus �ź���
StaticSemaphore_t modbusSend;
SemaphoreHandle_t xSemaphoreModbusSend = NULL;



//�������
MOTOR_STRUCT motor_array[4];
//��̬�ṹ��
POSTURE_STRUCT posture;
//��λ����ӳ�����鶨��
const uint8_t limitsw_to_motorid[17][2]={
	{0,0},//��������
	{1,0},
	{2,0},
	{3,0},
	{4,0},
	{5,1},
	{6,1},
	{7,1},
	{8,1},
	{9,2},
	{10,2},
	{11,2},
	{12,2},
	{13,3},
	{14,3},
	{15,3},
	{16,3},
};

//����ID����ӳ�䶨��
const uint8_t command_to_motor[60]={
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
};
//��������ӳ��
int(*command_to_function[27])(uint8_t*,uint32_t) = {
	command_0,
	command_1,
	command_2,
	command_3,
	command_4,
	command_5,
	command_6,
	command_7,
	command_8,
	command_9,
	command_10,
	command_11,
	command_12,
	command_13,
	command_14,
	command_15,
	command_16,
	command_17,
	command_18,
	command_19,
	command_20,
	command_21,
	command_22,
	command_23,
	command_24,
	command_25,
	command_26,
};

void(*result_to_parameter[7])(uint8_t*, uint8_t) = {
	NULL,
	result_parse_1,
	result_parse_2,
	result_parse_3,
	result_parse_4,
	result_parse_5,
	result_parse_6,
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
//�ض�����
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f) 
#endif /* __GNUC__ */
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

void configureTimerForRunTimeStats(void)
{
	ulHighFrequencyTimerTicks=0;
	return;
}
unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}

//��ʱ����ʼ��
void timer_start()
{
	//���PWM
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	//������ʱ����������
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);
	
	//��������ͳ�ƶ�ʱ��14
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Stop(&htim12);
	//HAL_TIM_Base_Start_IT(&htim12);
	return;
}

uint8_t can_start(void)
{
	HAL_CAN_DeInit(&hcan1);
	CAN_FilterTypeDef  sFilterConfig;
	
	  /*##-1- Configure the CAN peripheral #######################################*/
  hcan1.Instance = CAN1;

  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.Prescaler = 12;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0018;
  sFilterConfig.FilterIdLow = 0x0004;
  sFilterConfig.FilterMaskIdHigh = 0x0018;
  sFilterConfig.FilterMaskIdLow = 0x0004;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
		return ERROR_CAN_START_FAIL;
  }
	
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
		return ERROR_CAN_START_FAIL;
  }
	
	/*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
	/*##-5- Start the Reception process ########################################*/
  //if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 1)
  {
    /* Reception Missing */
   // Error_Handler();
  }

  //if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
   // Error_Handler();
  }
	return 0;
}


uint8_t iic_rw(uint8_t rw_flag, uint8_t addr,uint8_t* data,uint8_t length) //addr �׵�ַ ����ַ���� �� data�� ����ָ�룬 length�� ���ݳ���
{
	//uint8_t* txdata=(uint8_t*)pvPortMalloc(2*length);
	uint8_t txdata[2*EEPROM_CONFIG_LENGTH]={0};
	uint8_t rxdata[2*EEPROM_CONFIG_LENGTH]={0};
	uint8_t tmp_addr=addr;
	uint16_t i=0;
	for(i=0;i<length;i++)
	{
		txdata[i]=tmp_addr++;
	}
	for(i=0;i<length;i++)
	{
		txdata[i+length]=data[i];
	}
		
	if(rw_flag==1)
	{
		//write one by one
		for(i=0;i<length;i++)
		{
			uint8_t txcache[2];
			txcache[0]=txdata[i];
			txcache[1]=txdata[i+length];
			if(HAL_I2C_Master_Transmit(&hi2c1,IIC_ADDRESS,txcache,2,100)!=HAL_OK)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","start tk iic send error");
				#endif
				//vPortFree(txdata);
				return ERROR_EEPROM_FAIL;
			}
			else{
				//vPortFree(txdata);
				#ifdef DEBUG_OUTPUT
				printf("%s\n","start tk iic send success");
				#endif
			}
		}
		
		HAL_Delay(1);
	}
	
	if(rw_flag==0)
	{
		//read
		//uint8_t* rxdata=pvPortMalloc(length);
		uint8_t txcache;
		for(i=0;i<length;i++)
		{
			txcache=txdata[i];
			if(HAL_I2C_Master_Transmit(&hi2c1,IIC_ADDRESS,(uint8_t*)&txcache,1,100)!=HAL_OK)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","start tk iic send 2 error");
				#endif
				//vPortFree(rxdata);
				//vPortFree(txdata);
				return ERROR_EEPROM_FAIL;
			}
			else{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","start tk iic send 2 success");
				#endif
			}
		  HAL_Delay(1);
			//HAL_I2C_Master_Receive_IT(&hi2c1,IIC_ADDRESS,&rxdata[i],1)
			if(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)IIC_ADDRESS, (uint8_t *)&rxdata[i], 1, 100) != HAL_OK)
			{
				
				#ifdef DEBUG_OUTPUT
				printf("%s\n","start tk iic rece error");
				#endif
				//vPortFree(rxdata);
				//vPortFree(txdata);
				return ERROR_EEPROM_FAIL;
			}
			else{
				/*
				for(i=0;i<length;i++)
				{
					data[i]=rxdata[i];
				}
				*/
				data[i]=rxdata[i];
				#ifdef DEBUG_OUTPUT
				printf("%s\n","start tk iic rece success");
				#endif
			}
		}
		//vPortFree(rxdata);
	}
	//vPortFree(txdata);
	return 0;
	;
}
//�ź�����ʼ��
uint8_t lock_init(void)
{
	//�ź�����ʼ��
	//λ�û�ȡ�ź���
	xSemaphorePositionGet=xSemaphoreCreateBinaryStatic(&semaPositionGet);
	configASSERT(xSemaphorePositionGet);
	//λ�������ź���
	xSemaphorePositionSet=xSemaphoreCreateBinaryStatic(&semaPositionSet);
	configASSERT(xSemaphorePositionSet);
	//����״̬��ȡ�ź���
	xSemaphoreSwitchGet=xSemaphoreCreateBinaryStatic(&semaSwitchGet);
	configASSERT(xSemaphoreSwitchGet);
	//��դ״̬��ȡ�ź���
	xSemaphoreGratingGet=xSemaphoreCreateBinaryStatic(&semaGratingGet);
	configASSERT(xSemaphoreGratingGet);
	
	//modbus send�ź�����ʼ��
	xSemaphoreModbusSend=xSemaphoreCreateBinaryStatic(&modbusSend);
	configASSERT(xSemaphoreGratingGet);
	
	//���ź�����ֵ
	if( xSemaphoreGive( xSemaphorePositionGet ) != pdTRUE )
	{
			return ERROR_CANNOT_GIVE_SEM;
	}
	if( xSemaphoreGive( xSemaphorePositionSet ) != pdTRUE )
	{
			return ERROR_CANNOT_GIVE_SEM;
	}
	if( xSemaphoreGive( xSemaphoreSwitchGet ) != pdTRUE )
	{
			return ERROR_CANNOT_GIVE_SEM;
	}
	if( xSemaphoreGive( xSemaphoreGratingGet ) != pdTRUE )
	{
			return ERROR_CANNOT_GIVE_SEM;
	}
	if( xSemaphoreGive( xSemaphoreModbusSend ) != pdTRUE )
	{
			return ERROR_CANNOT_GIVE_SEM;
	}
	return 0;
}

//����״̬����ȡ
uint8_t switchGet(uint8_t motor_id)
{
	if(xSemaphoreSwitchGet != NULL)
	{
		if(xSemaphoreTake(xSemaphoreSwitchGet,(TickType_t)10)==pdTRUE)
		{
			if(motor_id >6)
			{
				if( xSemaphoreGive( xSemaphoreSwitchGet ) != pdTRUE )
				{
						return ERROR_CANNOT_GIVE_SEM;
				}
				return ERROR_MOTOR_ID_ERROR;
			}
			else{
				//GPIO ״̬��ȡ����ȡ����µ���ؽṹ��Ŀ������У������ݿ��ص��������ø��¿���״̬Ϊ�����򲻴���
				GPIO_PinState res=GPIO_PIN_RESET;
				uint8_t i=0,j=0;
				if(motor_id==4)//��ȡȫ������
				{
					
					for(i=0;i<=MAX_MOTOR_NUMBER;i++)
					{
						for(j=0;j<motor_array[i].limit_sw_number;j++)
						{
							res=HAL_GPIO_ReadPin((GPIO_TypeDef *)motor_array[i].limit_sw[j].gpio_port,motor_array[i].limit_sw[j].pin_number);
							//������λ���ش���״̬
							if(res^motor_array[i].limit_sw[j].type)
							{
								motor_array[i].limit_sw[j].status=1;
							}
							else{
								motor_array[i].limit_sw[j].status=0;
							}
						}
					}
					//������̬����״̬
					for(i=0;i<POSTURE_NUM;i++)
					{
						res=HAL_GPIO_ReadPin((GPIO_TypeDef*)posture.postureValue[i].gpio_port,posture.postureValue[i].pin_number);
						if(res^posture.postureValue[i].type)
						{
							posture.postureValue[i].status=1;
						}
						else{
							posture.postureValue[i].status=0;
						}
					}
					
				}else if(motor_id==5){//��ȡ��դ״̬
					//���¹�դ״̬��MODBUS RTU ͨ�ţ� ��ȡ�����Ϣδ���δ���
					
					
					posture.gratingValue.distance=0;//��ֵ�洢�ڵ���Ϣ
					if(posture.gratingValue.distance>posture.gratingValue.distance_max)
					{
						posture.gratingValue.status=1;
					}
					else{
						posture.gratingValue.status=0;
					}
					//�ڵ���ϢΪ0�������û��Ŀ��
					if(posture.gratingValue.dark_point==0)
					{
						posture.gratingValue.if_have_target=0;
					}
					else{
						posture.gratingValue.if_have_target=1;
					}
				}else if(motor_id==4){//��ȡ�����բ״̬
					res=GPIO_PIN_RESET;
					res=HAL_GPIO_ReadPin(motor_array[0].gpio_input[0].gpio_port,motor_array[0].gpio_input[0].pin_number);
					if(res==GPIO_PIN_RESET)
					{
						motor_array[0].gpio_input[0].break_status=0;
					}
					else
					{
						motor_array[0].gpio_input[0].break_status=1;
					}
				}else{//��ȡ�����λ״̬
					res=GPIO_PIN_RESET;
					for(j=0;j<motor_array[motor_id].limit_sw_number;j++)
					{
						res=HAL_GPIO_ReadPin((GPIO_TypeDef *)motor_array[i].limit_sw[j].gpio_port,motor_array[i].limit_sw[j].pin_number);
						//������λ���ش���״̬
						if(res^motor_array[i].limit_sw[j].type)
						{
							motor_array[i].limit_sw[j].status=1;
						}
						else{
							motor_array[i].limit_sw[j].status=0;
						}
					}
				}
			}
			
			//�ͷ��ź�������
			if( xSemaphoreGive( xSemaphoreSwitchGet ) != pdTRUE )
			{
					return ERROR_CANNOT_GIVE_SEM;
			}
			return 0;
		}
		else{
			return ERROR_FUNC_BUSY;
		}
	}
	else
	{
		return ERROR_SEMA_NULL;
	}
}

//���Ŀ���ȡ
uint8_t positionGet(uint8_t motorId,int32_t* position)
{
	if(xSemaphorePositionGet != NULL)
	{
		if(xSemaphoreTake(xSemaphorePositionGet,( TickType_t )10) == pdTRUE)
		{
			//MODBUS RTU ��ȡ���λ��,δ��ɣ���ȡ���λ�ú�Ҫ���µ��λ�õ�����ṹ��ĵ�ǰλ����
			*position=0;
			
			
			xSemaphoreGive(xSemaphorePositionGet);
			return 0;
		}
		else{
			return ERROR_FUNC_BUSY;
		}
	}
	else{
		return ERROR_SEMA_NULL;
	}
}

//can ����
uint8_t can_send(QUEUE_STRUCT send_struct)
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint32_t              TxMailbox;
	
	//�̶�ֵ
	TxHeader.StdId = 0x000;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_EXT;
	TxHeader.TransmitGlobalTime = DISABLE;
	
	//���ͳ���
	if(send_struct.length>8)
	{
		return ERROR_CAN_SEND_FAIL;
	}
	else{
		TxHeader.DLC = send_struct.length;
	}
  
  //��չID
	uint32_t exid=0;
	
	exid|=((send_struct.can_priority & 0x07)<<26);
	
	exid|=((send_struct.can_source & 0x1F)<<21);
	
	exid|=((send_struct.can_target & 0x1F)<<16);
	
	exid|=((send_struct.can_command & 0x7F)<<9);
	
	exid|=((send_struct.can_if_last & 0x01)<<8);
	
	exid|=((send_struct.can_if_return & 0x01)<<7);
	
	exid|=((send_struct.can_if_ack & 0x01)<<6);
	
	//exid|=(send_struct.can_version & 0x07);
	exid|=(0x00 & 0x07);
	
	exid&=0xFFFFFFC0;
	
	TxHeader.ExtId = exid;
	
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, send_struct.data, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
	  Error_Handler();
		return ERROR_CAN_SEND_FAIL;
	}
	return 0;
}

//485 ����
uint8_t modbus_send(QUEUE_STRUCT send_struct)
{
	if(modbus_status==0)
	{
		if(modbus_list_tail!=NULL && modbus_list_tail->if_over==0)
		{
			memcpy(&(modbus_list_tail->modbus_element),&send_struct,sizeof(QUEUE_STRUCT));
			modbus_list_tail->if_over=1;
			modbus_list_tail=modbus_list_tail->next;
		}
		else{
			return MODBUS_LIST_ERROR;
		}
		//д�Ĵ���
		if(send_struct.modbus_func == 0x10)
		{
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
			__NOP();
		  __NOP();
			GPIO_PinState tmpread=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6);
			modbus_send_cache[0]=send_struct.modbus_addr;
			modbus_send_cache[1]=send_struct.modbus_func;
			modbus_send_cache[2]=send_struct.modbus_addr_h;
			modbus_send_cache[3]=send_struct.modbus_addr_l;
			modbus_send_cache[4]=send_struct.modbus_data_len_h;
			modbus_send_cache[5]=send_struct.modbus_data_len_l;
			modbus_send_cache[6]=send_struct.modbus_data_byte;
			modbus_send_cache[7]=send_struct.modbus_data_1;
			modbus_send_cache[8]=send_struct.modbus_data_2;
			modbus_send_cache[9]=send_struct.modbus_data_3;
			modbus_send_cache[10]=send_struct.modbus_data_4;
			send_struct.modbus_crc=usMBCRC16(modbus_send_cache,11);
			modbus_send_cache[11]=(uint8_t)(send_struct.modbus_crc & 0xFF);
			modbus_send_cache[12]=(uint8_t)(send_struct.modbus_crc >> 8);
			HAL_UART_Transmit_DMA(&huart2,(uint8_t*)modbus_send_cache,13);
			modbus_time_flag=1;
			rece_count=8;
			modbus_status=1;
		}
		//���Ĵ���
		if(send_struct.modbus_func==0x03)
		{
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
			__NOP();
		  __NOP();
			modbus_send_cache[0]=send_struct.modbus_addr;
			modbus_send_cache[1]=send_struct.modbus_func;
			modbus_send_cache[2]=send_struct.modbus_addr_h;
			modbus_send_cache[3]=send_struct.modbus_addr_l;
			modbus_send_cache[4]=send_struct.modbus_data_len_h;
			modbus_send_cache[5]=send_struct.modbus_data_len_l;
			send_struct.modbus_crc=usMBCRC16(modbus_send_cache,6);
			modbus_send_cache[6]=(uint8_t)(send_struct.modbus_crc & 0xFF);
			modbus_send_cache[7]=(uint8_t)(send_struct.modbus_crc >> 8);
			HAL_UART_Transmit_DMA(&huart2,(uint8_t*)modbus_send_cache,8);
			modbus_time_flag=1;
			rece_count=9;
			modbus_status=1;
		}
	}
	else
	{
		//modbus����ѹ������
		if(modbus_list_tail!=NULL && modbus_list_tail->if_over==0)
		{
			memcpy(&(modbus_list_tail->modbus_element),&send_struct,sizeof(QUEUE_STRUCT));
			modbus_list_tail->if_over=1;
			modbus_list_tail=modbus_list_tail->next;
			return MODBUS_BUSY;
		}
		else
		{
			return MODBUS_LIST_ERROR;
		}
	}
	return 0;
}

//485��ص�����
uint8_t modbus_send_sub(QUEUE_STRUCT send_struct)
{
	//д�Ĵ���
	if(send_struct.modbus_func == 0x10)
	{
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
		__NOP();
		__NOP();
		modbus_send_cache[0]=send_struct.modbus_addr;
		modbus_send_cache[1]=send_struct.modbus_func;
		modbus_send_cache[2]=send_struct.modbus_addr_h;
		modbus_send_cache[3]=send_struct.modbus_addr_l;
		modbus_send_cache[4]=send_struct.modbus_data_len_h;
		modbus_send_cache[5]=send_struct.modbus_data_len_l;
		modbus_send_cache[6]=send_struct.modbus_data_byte;
		modbus_send_cache[7]=send_struct.modbus_data_1;
		modbus_send_cache[8]=send_struct.modbus_data_2;
		modbus_send_cache[9]=send_struct.modbus_data_3;
		modbus_send_cache[10]=send_struct.modbus_data_4;
		send_struct.modbus_crc=usMBCRC16(modbus_send_cache,11);
		modbus_send_cache[11]=(uint8_t)(send_struct.modbus_crc & 0xFF);
		modbus_send_cache[12]=(uint8_t)(send_struct.modbus_crc >> 8);
		if(HAL_UART_Transmit_DMA(&huart2,(uint8_t*)modbus_send_cache,13)==HAL_BUSY)
		{
			__NOP();
			HAL_UART_Transmit_DMA(&huart2,(uint8_t*)modbus_send_cache,13);
		}
		modbus_time_flag=1;
		rece_count=8;
	}
	//���Ĵ���
	if(send_struct.modbus_func==0x03)
	{
		
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
		__NOP();
		__NOP();
		modbus_send_cache[0]=send_struct.modbus_addr;
		modbus_send_cache[1]=send_struct.modbus_func;
		modbus_send_cache[2]=send_struct.modbus_addr_h;
		modbus_send_cache[3]=send_struct.modbus_addr_l;
		modbus_send_cache[4]=send_struct.modbus_data_len_h;
		modbus_send_cache[5]=send_struct.modbus_data_len_l;
		send_struct.modbus_crc=usMBCRC16(modbus_send_cache,6);
		modbus_send_cache[6]=(uint8_t)(send_struct.modbus_crc & 0xFF);
		modbus_send_cache[7]=(uint8_t)(send_struct.modbus_crc >> 8);
		if(HAL_UART_Transmit_DMA(&huart2,(uint8_t*)modbus_send_cache,8)==HAL_BUSY)
		{
			__nop();
			HAL_UART_Transmit_DMA(&huart2,(uint8_t*)modbus_send_cache,8);
		}
		modbus_time_flag=1;
		rece_count=9;
	}
	return 0;
}


//���Ŀ������
uint8_t positionSet(uint8_t motorId, int32_t * position)
{
	if(xSemaphorePositionSet != NULL)
	{
		if(xSemaphoreTake(xSemaphorePositionSet,(TickType_t)10)==pdTRUE)
		{
			//MODBUS RTU ���õ��λ�ã� δ���,���ú���µ��λ�õ�����ṹ���е�Ŀ��λ���У�ͬʱ���µ�����з���dir�ĵ�0λ
			xSemaphoreGive(xSemaphorePositionSet);
			return 0;
		}
		else{
			return ERROR_FUNC_BUSY;
		}
	}
	else
	{
		return ERROR_SEMA_NULL;
	}
}


//���״̬��ȡ
uint8_t gratingGet(void)
{
	if(xSemaphoreGratingGet != NULL)
	{
		if(xSemaphoreTake(xSemaphoreGratingGet,(TickType_t)10)==pdTRUE)
		{
			//MODBUS RTU ��ȡ��դλ�ã���ȡ����µ���̬�ṹ�壬δ���
			
			
			if( xSemaphoreGive( xSemaphoreGratingGet ) != pdTRUE )
			{
					return ERROR_CANNOT_GIVE_SEM;
			}
			return 0;
		}
		else{
			return ERROR_FUNC_BUSY;
		}
	}
	else
	{
		return ERROR_SEMA_NULL;
	}
}


//��̬��ȡ
ANGLE_STRUCT angleCalculate(GRATING grating)
{
	ANGLE_STRUCT res;
	res.dir=0;
	res.angle=0;
	//����Ƕ�,δ��ɣ������ɨ��λ���ٸ���
	
	return res;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
	MX_DMA_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  
  /* USER CODE BEGIN 2 */
	
	//��ʼ���ź���
	lock_init();
	
	HAL_DMA_DeInit(&hdma_usart2_tx);
	HAL_DMA_Init(&hdma_usart2_tx);
	HAL_DMA_DeInit(&hdma_usart2_rx);
	HAL_DMA_Init(&hdma_usart2_rx);
	
	
	HAL_TIM_Base_DeInit(&htim12);
	HAL_TIM_Base_Init(&htim12);
	
	modbus_list_head=modbus_list_gen(128);
	
	//HAL_TIM_Base_Start_IT(&htim12);
	
	
	if(modbus_list_head!=NULL)
	{
		modbus_list_tail=modbus_list_head;
	}
	else
	{
		printf("%s\n","modbus list error");
	}
	//Ĭ�����ڷ���״̬
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
	printf("%s\n","start free rtos");
	
	//��ʼ������е�λ�ú�һЩ�����Ĺؼ�����
	motor_array_init();
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
