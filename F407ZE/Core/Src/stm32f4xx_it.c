/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
BaseType_t b_tk_default=pdTRUE;
BaseType_t b_tk_sensor_monitor=pdTRUE;
BaseType_t b_tk_pidoutput=pdTRUE;
BaseType_t b_tk_conflict_monitor=pdTRUE;
BaseType_t b_tk_grating_monitor=pdTRUE;
BaseType_t b_tk_posture_monitor=pdTRUE;
BaseType_t b_tk_commu_monitor=pdTRUE;
BaseType_t b_tk_send_order=pdTRUE;
BaseType_t b_tk_master_order=pdTRUE;
BaseType_t b_tk_rece_result=pdTRUE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
extern QueueHandle_t rece_queueHandle;
extern TaskHandle_t defaultTaskHandle;
extern TaskHandle_t sensor_monitorHandle;
extern TaskHandle_t pid_outputHandle;
extern TaskHandle_t conflict_monitorHandle;
extern TaskHandle_t grating_monitorHandle;
extern TaskHandle_t posture_monitorHandle;
extern TaskHandle_t commu_mornitorHandle;
extern TaskHandle_t send_orderHandle;
extern TaskHandle_t master_orderHandle;
extern TaskHandle_t result_processHandle;
extern TaskHandle_t result_processHandle_rece;
extern TaskHandle_t result_processHandle_send;

extern TaskHandle_t result_processHandle_rece_5;
extern TaskHandle_t result_processHandle_rece_5_sendover;
extern TaskHandle_t result_processHandle_rece_5_timeout;
extern uint16_t modbus_period;
extern uint32_t fifo_level;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[0].limit_sw[0].status=HAL_GPIO_ReadPin(motor_array[0].limit_sw[0].gpio_port,motor_array[0].limit_sw[0].pin_number);
		if(motor_array[0].command.command_id==15)
		{
			motor_array[0].self_check_counter--;
		}
	}

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[0].limit_sw[1].status=HAL_GPIO_ReadPin(motor_array[0].limit_sw[1].gpio_port,motor_array[0].limit_sw[1].pin_number);
		if(motor_array[0].command.command_id==15)
		{
			motor_array[0].self_check_counter--;
		}
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[0].limit_sw[2].status=HAL_GPIO_ReadPin(motor_array[0].limit_sw[2].gpio_port,motor_array[0].limit_sw[2].pin_number);
		if(motor_array[0].command.command_id==15)
		{
			motor_array[0].self_check_counter--;
		}
	}
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[0].limit_sw[3].status=HAL_GPIO_ReadPin(motor_array[0].limit_sw[3].gpio_port,motor_array[0].limit_sw[3].pin_number);
		if(motor_array[0].command.command_id==15)
		{
			motor_array[0].self_check_counter--;
		}
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[2].limit_sw[0].status=HAL_GPIO_ReadPin(motor_array[2].limit_sw[0].gpio_port,motor_array[2].limit_sw[0].pin_number);
		if(motor_array[2].command.command_id==17)
		{
			motor_array[2].self_check_counter--;
		}
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
	
  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */
	
  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	if(master_orderHandle!=NULL)
	{
		QUEUE_STRUCT can_rece;
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &(can_rece.RxHeader), can_rece.data) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}
		if(rece_queueHandle!=NULL)
		{
			portBASE_TYPE status;
			//timer_period=uxQueueSpacesAvailable(rece_queueHandle);
			status = xQueueSendToBackFromISR(rece_queueHandle, &can_rece, &b_tk_master_order);
			if(status!=pdPASS)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","queue overflow");
				#endif
			}
			else
			{
				#ifdef DEBUG_OUTPUT
				printf("%s\n","send message to queue already");
				#endif
			}
			
		}
		else
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","send master order queue error");
			#endif
		}
		//xTaskNotifyFromISR(master_orderHandle,0x0001,eSetBits,&b_tk_master_order);
		portYIELD_FROM_ISR( b_tk_master_order );
	}

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */
	
  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 SCE interrupt.
  */
void CAN1_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_SCE_IRQn 0 */

  /* USER CODE END CAN1_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_SCE_IRQn 1 */

  /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[2].limit_sw[1].status=HAL_GPIO_ReadPin(motor_array[2].limit_sw[1].gpio_port,motor_array[2].limit_sw[1].pin_number);
		if(motor_array[2].command.command_id==17)
		{
			motor_array[2].self_check_counter--;
		}
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[2].limit_sw[2].status=HAL_GPIO_ReadPin(motor_array[2].limit_sw[2].gpio_port,motor_array[2].limit_sw[2].pin_number);
		if(motor_array[2].command.command_id==17)
		{
			motor_array[2].self_check_counter--;
		}
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[3].limit_sw[0].status=HAL_GPIO_ReadPin(motor_array[3].limit_sw[0].gpio_port,motor_array[3].limit_sw[0].pin_number);
		if(motor_array[3].command.command_id==18)
		{
			motor_array[3].self_check_counter--;
		}
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[3].limit_sw[1].status=HAL_GPIO_ReadPin(motor_array[3].limit_sw[1].gpio_port,motor_array[3].limit_sw[1].pin_number);
		if(motor_array[3].command.command_id==18)
		{
			motor_array[3].self_check_counter--;
		}
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9)!=RESET)
	{
		//FALLING TRIGGER
		motor_array[3].limit_sw[2].status=HAL_GPIO_ReadPin(motor_array[3].limit_sw[2].gpio_port,motor_array[3].limit_sw[2].pin_number);
		if(motor_array[3].command.command_id==18)
		{
			motor_array[3].self_check_counter--;
		}
	}
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */
	if(grating_monitorHandle !=NULL)
	{
		xTaskNotifyFromISR(grating_monitorHandle,0x0001,eSetBits,&b_tk_grating_monitor);
		portYIELD_FROM_ISR( b_tk_grating_monitor );
	}
  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
	
  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	if(__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE) != RESET)
	{
		if(commu_mornitorHandle !=NULL)
		{
			xTaskNotifyFromISR(commu_mornitorHandle,0x0001,eSetBits,&b_tk_commu_monitor);
			portYIELD_FROM_ISR(b_tk_commu_monitor);
		}
	}
	

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */
	if(pid_outputHandle !=NULL)
	{
		xTaskNotifyFromISR(pid_outputHandle,0x0001,eSetBits,&b_tk_pidoutput);
		portYIELD_FROM_ISR(b_tk_pidoutput);
	}
  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=RESET && modbus_time_flag==1)
	{
		modbus_time_flag=0;
		if(result_processHandle_send !=NULL)
		{
			xTaskNotifyFromISR(result_processHandle_send,0x0001,eSetBits,&b_tk_rece_result);
			portYIELD_FROM_ISR( b_tk_rece_result );
		}
		HAL_UART_DMAStop(&huart2);
	}
	if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)!=RESET)
	{
		if(modbus_time_flag!=0)
		{
			if(result_processHandle_rece !=NULL)
			{
				xTaskNotifyFromISR(result_processHandle_rece,0x0002,eSetBits,&b_tk_rece_result);
				portYIELD_FROM_ISR( b_tk_rece_result );
			}
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);
			modbus_time_flag=0;
			HAL_UART_DMAStop(&huart2);
			HAL_TIM_Base_Stop(&htim12);
			TIM12->CNT=0;
		}
	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */
	if(__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_UPDATE) != RESET)
	{
		if(result_processHandle!=NULL)
		{
			xTaskNotifyFromISR(result_processHandle,0x0021,eSetBits,&b_tk_rece_result);
			portYIELD_FROM_ISR( b_tk_rece_result );
		}
		__HAL_TIM_CLEAR_FLAG(&htim12,TIM_FLAG_UPDATE);
		TIM12->CNT=0;
	}
  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */
	//HAL_TIM_Base_Start_IT(&htim12);
	
	HAL_TIM_Base_Stop(&htim12);
  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
	if(__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) != RESET)
	{
		if(conflict_monitorHandle !=NULL)
		{
			xTaskNotifyFromISR(conflict_monitorHandle,0x0001,eSetBits,&b_tk_conflict_monitor);
			portYIELD_FROM_ISR(b_tk_conflict_monitor);
		}
	}
	
	if(__HAL_TIM_GET_FLAG(&htim13, TIM_FLAG_UPDATE) != RESET)
	{
		if(result_processHandle_rece_5_timeout!=NULL)
		{
			xTaskNotifyFromISR(result_processHandle_rece_5_timeout,0x0021,eSetBits,&b_tk_rece_result);
			portYIELD_FROM_ISR( b_tk_rece_result );
		}
		__HAL_TIM_CLEAR_FLAG(&htim13,TIM_FLAG_UPDATE);
		TIM13->CNT=0;
	}
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */
	HAL_TIM_Base_Stop(&htim13);
  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
	ulHighFrequencyTimerTicks++;

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */

  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	//触发传感器监测
	if(sensor_monitorHandle !=NULL)
	{
		xTaskNotifyFromISR(sensor_monitorHandle,0x0001,eSetBits,&b_tk_sensor_monitor);
		portYIELD_FROM_ISR(b_tk_sensor_monitor);
	}
	__HAL_TIM_CLEAR_FLAG(&htim5,TIM_FLAG_UPDATE);
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	//触发打印线程执行情况,执行默认任务
	if(defaultTaskHandle !=NULL)
	{
		xTaskNotifyFromISR(defaultTaskHandle,0x0001,eSetBits,&b_tk_default);
		//xTaskNotifyFromISR(send_orderHandle,0x0009,eSetBits,&b_tk_send_order);
		portYIELD_FROM_ISR(b_tk_default);
	}
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_TC)!=RESET && modbus_time_flag_5==1)
	{
		modbus_time_flag_5=0;
		if(result_processHandle_rece_5_sendover !=NULL)
		{
			xTaskNotifyFromISR(result_processHandle_rece_5_sendover,0x0001,eSetBits,&b_tk_rece_result);
			portYIELD_FROM_ISR( b_tk_rece_result );
		}
		HAL_UART_DMAStop(&huart6);
	}
	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)
	{
		if(modbus_time_flag_5!=0)
		{
			if(result_processHandle_rece_5 !=NULL)
			{
				xTaskNotifyFromISR(result_processHandle_rece_5,0x0002,eSetBits,&b_tk_rece_result);
				portYIELD_FROM_ISR( b_tk_rece_result );
			}
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);
			modbus_time_flag_5=0;
			HAL_UART_DMAStop(&huart6);
			HAL_TIM_Base_Stop(&htim13);
			TIM13->CNT=0;
		}
	}
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
