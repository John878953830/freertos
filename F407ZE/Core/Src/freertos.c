/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId_t pid_outputHandle;
uint32_t pid_outputBuffer[ 512 ];
osStaticThreadDef_t pid_outputControlBlock;
osThreadId_t send_orderHandle;
uint32_t send_orderBuffer[ 512 ];
osStaticThreadDef_t send_orderControlBlock;
osThreadId_t commu_mornitorHandle;
uint32_t commu_mornitorBuffer[ 512 ];
osStaticThreadDef_t commu_mornitorControlBlock;
osThreadId_t conflict_monitorHandle;
uint32_t conflict_monitorBuffer[ 512 ];
osStaticThreadDef_t conflict_monitorControlBlock;
osThreadId_t sensor_monitorHandle;
uint32_t sensor_monitorBuffer[ 512 ];
osStaticThreadDef_t sensor_monitorControlBlock;
osThreadId_t grating_monitorHandle;
uint32_t grating_monitorBuffer[ 128 ];
osStaticThreadDef_t grating_monitorControlBlock;
osThreadId_t posture_monitorHandle;
uint32_t posture_monitorBuffer[ 128 ];
osStaticThreadDef_t posture_monitorControlBlock;
osThreadId_t zero_swHandle;
uint32_t zero_swBuffer[ 128 ];
osStaticThreadDef_t zero_swControlBlock;
osThreadId_t co_orderHandle;
uint32_t co_orderBuffer[ 128 ];
osStaticThreadDef_t co_orderControlBlock;
osThreadId_t master_orderHandle;
uint32_t master_orderBuffer[ 512 ];
osStaticThreadDef_t master_orderControlBlock;
osThreadId_t limit_swHandle;
uint32_t limit_swBuffer[ 256 ];
osStaticThreadDef_t limit_swControlBlock;
osMessageQueueId_t send_queueHandle;
uint8_t send_queueBuffer[ 256 * sizeof( uint32_t ) ];
osStaticMessageQDef_t send_queueControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void start_tk_pidoutput(void *argument);
void start_tk_send_order(void *argument);
void start_tk_commu_monitor(void *argument);
void start_tk_conflict_monitor(void *argument);
void start_tk_sensor_monitor(void *argument);
void start_tk_grating_monitor(void *argument);
void start_tk_posture_monitor(void *argument);
void start_tk_zero_monitor(void *argument);
void start_tk_co_order(void *argument);
void start_tk_master_order(void *argument);
void start_tk_limit_sw(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of send_queue */
  const osMessageQueueAttr_t send_queue_attributes = {
    .name = "send_queue",
    .cb_mem = &send_queueControlBlock,
    .cb_size = sizeof(send_queueControlBlock),
    .mq_mem = &send_queueBuffer,
    .mq_size = sizeof(send_queueBuffer)
  };
  send_queueHandle = osMessageQueueNew (256, sizeof(uint32_t), &send_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_mem = &defaultTaskBuffer[0],
    .stack_size = sizeof(defaultTaskBuffer),
    .cb_mem = &defaultTaskControlBlock,
    .cb_size = sizeof(defaultTaskControlBlock),
    .priority = (osPriority_t) osPriorityNormal,
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* definition and creation of pid_output */
  const osThreadAttr_t pid_output_attributes = {
    .name = "pid_output",
    .stack_mem = &pid_outputBuffer[0],
    .stack_size = sizeof(pid_outputBuffer),
    .cb_mem = &pid_outputControlBlock,
    .cb_size = sizeof(pid_outputControlBlock),
    .priority = (osPriority_t) osPriorityAboveNormal2,
  };
  pid_outputHandle = osThreadNew(start_tk_pidoutput, NULL, &pid_output_attributes);

  /* definition and creation of send_order */
  const osThreadAttr_t send_order_attributes = {
    .name = "send_order",
    .stack_mem = &send_orderBuffer[0],
    .stack_size = sizeof(send_orderBuffer),
    .cb_mem = &send_orderControlBlock,
    .cb_size = sizeof(send_orderControlBlock),
    .priority = (osPriority_t) osPriorityAboveNormal4,
  };
  send_orderHandle = osThreadNew(start_tk_send_order, NULL, &send_order_attributes);

  /* definition and creation of commu_mornitor */
  const osThreadAttr_t commu_mornitor_attributes = {
    .name = "commu_mornitor",
    .stack_mem = &commu_mornitorBuffer[0],
    .stack_size = sizeof(commu_mornitorBuffer),
    .cb_mem = &commu_mornitorControlBlock,
    .cb_size = sizeof(commu_mornitorControlBlock),
    .priority = (osPriority_t) osPriorityHigh1,
  };
  commu_mornitorHandle = osThreadNew(start_tk_commu_monitor, NULL, &commu_mornitor_attributes);

  /* definition and creation of conflict_monitor */
  const osThreadAttr_t conflict_monitor_attributes = {
    .name = "conflict_monitor",
    .stack_mem = &conflict_monitorBuffer[0],
    .stack_size = sizeof(conflict_monitorBuffer),
    .cb_mem = &conflict_monitorControlBlock,
    .cb_size = sizeof(conflict_monitorControlBlock),
    .priority = (osPriority_t) osPriorityHigh3,
  };
  conflict_monitorHandle = osThreadNew(start_tk_conflict_monitor, NULL, &conflict_monitor_attributes);

  /* definition and creation of sensor_monitor */
  const osThreadAttr_t sensor_monitor_attributes = {
    .name = "sensor_monitor",
    .stack_mem = &sensor_monitorBuffer[0],
    .stack_size = sizeof(sensor_monitorBuffer),
    .cb_mem = &sensor_monitorControlBlock,
    .cb_size = sizeof(sensor_monitorControlBlock),
    .priority = (osPriority_t) osPriorityHigh5,
  };
  sensor_monitorHandle = osThreadNew(start_tk_sensor_monitor, NULL, &sensor_monitor_attributes);

  /* definition and creation of grating_monitor */
  const osThreadAttr_t grating_monitor_attributes = {
    .name = "grating_monitor",
    .stack_mem = &grating_monitorBuffer[0],
    .stack_size = sizeof(grating_monitorBuffer),
    .cb_mem = &grating_monitorControlBlock,
    .cb_size = sizeof(grating_monitorControlBlock),
    .priority = (osPriority_t) osPriorityHigh7,
  };
  grating_monitorHandle = osThreadNew(start_tk_grating_monitor, NULL, &grating_monitor_attributes);

  /* definition and creation of posture_monitor */
  const osThreadAttr_t posture_monitor_attributes = {
    .name = "posture_monitor",
    .stack_mem = &posture_monitorBuffer[0],
    .stack_size = sizeof(posture_monitorBuffer),
    .cb_mem = &posture_monitorControlBlock,
    .cb_size = sizeof(posture_monitorControlBlock),
    .priority = (osPriority_t) osPriorityRealtime,
  };
  posture_monitorHandle = osThreadNew(start_tk_posture_monitor, NULL, &posture_monitor_attributes);

  /* definition and creation of zero_sw */
  const osThreadAttr_t zero_sw_attributes = {
    .name = "zero_sw",
    .stack_mem = &zero_swBuffer[0],
    .stack_size = sizeof(zero_swBuffer),
    .cb_mem = &zero_swControlBlock,
    .cb_size = sizeof(zero_swControlBlock),
    .priority = (osPriority_t) osPriorityRealtime1,
  };
  zero_swHandle = osThreadNew(start_tk_zero_monitor, NULL, &zero_sw_attributes);

  /* definition and creation of co_order */
  const osThreadAttr_t co_order_attributes = {
    .name = "co_order",
    .stack_mem = &co_orderBuffer[0],
    .stack_size = sizeof(co_orderBuffer),
    .cb_mem = &co_orderControlBlock,
    .cb_size = sizeof(co_orderControlBlock),
    .priority = (osPriority_t) osPriorityRealtime2,
  };
  co_orderHandle = osThreadNew(start_tk_co_order, NULL, &co_order_attributes);

  /* definition and creation of master_order */
  const osThreadAttr_t master_order_attributes = {
    .name = "master_order",
    .stack_mem = &master_orderBuffer[0],
    .stack_size = sizeof(master_orderBuffer),
    .cb_mem = &master_orderControlBlock,
    .cb_size = sizeof(master_orderControlBlock),
    .priority = (osPriority_t) osPriorityRealtime3,
  };
  master_orderHandle = osThreadNew(start_tk_master_order, NULL, &master_order_attributes);

  /* definition and creation of limit_sw */
  const osThreadAttr_t limit_sw_attributes = {
    .name = "limit_sw",
    .stack_mem = &limit_swBuffer[0],
    .stack_size = sizeof(limit_swBuffer),
    .cb_mem = &limit_swControlBlock,
    .cb_size = sizeof(limit_swControlBlock),
    .priority = (osPriority_t) osPriorityRealtime5,
  };
  limit_swHandle = osThreadNew(start_tk_limit_sw, NULL, &limit_sw_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	//定时器初始化
	timer_start();
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint32_t notify_use=0;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
                         0xffffffff,          /* Clear all bits on exit. */
                         &notify_use, /* Receives the notification value. */
                         2000 );
		if(notify_use!=0)
		{
			notify_use=0;
			//打印线程执行情况
			vTaskList((char *)&tklog);
	    printf("%s\n",tklog);
	    vTaskGetRunTimeStats((char *)&tklog);
	    printf("%s\n",tklog);
			
			;
		}
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_start_tk_pidoutput */
/**
* @brief Function implementing the pid_output thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_pidoutput */
void start_tk_pidoutput(void *argument)
{
  /* USER CODE BEGIN start_tk_pidoutput */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_pidoutput */
}

/* USER CODE BEGIN Header_start_tk_send_order */
/**
* @brief Function implementing the send_order thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_send_order */
void start_tk_send_order(void *argument)
{
  /* USER CODE BEGIN start_tk_send_order */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_send_order */
}

/* USER CODE BEGIN Header_start_tk_commu_monitor */
/**
* @brief Function implementing the commu_mornitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_commu_monitor */
void start_tk_commu_monitor(void *argument)
{
  /* USER CODE BEGIN start_tk_commu_monitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_commu_monitor */
}

/* USER CODE BEGIN Header_start_tk_conflict_monitor */
/**
* @brief Function implementing the conflict_monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_conflict_monitor */
void start_tk_conflict_monitor(void *argument)
{
  /* USER CODE BEGIN start_tk_conflict_monitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_conflict_monitor */
}

/* USER CODE BEGIN Header_start_tk_sensor_monitor */
/**
* @brief Function implementing the sensor_monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_sensor_monitor */
void start_tk_sensor_monitor(void *argument)
{
  /* USER CODE BEGIN start_tk_sensor_monitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_sensor_monitor */
}

/* USER CODE BEGIN Header_start_tk_grating_monitor */
/**
* @brief Function implementing the grating_monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_grating_monitor */
void start_tk_grating_monitor(void *argument)
{
  /* USER CODE BEGIN start_tk_grating_monitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_grating_monitor */
}

/* USER CODE BEGIN Header_start_tk_posture_monitor */
/**
* @brief Function implementing the posture_monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_posture_monitor */
void start_tk_posture_monitor(void *argument)
{
  /* USER CODE BEGIN start_tk_posture_monitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_posture_monitor */
}

/* USER CODE BEGIN Header_start_tk_zero_monitor */
/**
* @brief Function implementing the zero_sw thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_zero_monitor */
void start_tk_zero_monitor(void *argument)
{
  /* USER CODE BEGIN start_tk_zero_monitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_zero_monitor */
}

/* USER CODE BEGIN Header_start_tk_co_order */
/**
* @brief Function implementing the co_order thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_co_order */
void start_tk_co_order(void *argument)
{
  /* USER CODE BEGIN start_tk_co_order */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_co_order */
}

/* USER CODE BEGIN Header_start_tk_master_order */
/**
* @brief Function implementing the master_order thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_master_order */
void start_tk_master_order(void *argument)
{
  /* USER CODE BEGIN start_tk_master_order */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_master_order */
}

/* USER CODE BEGIN Header_start_tk_limit_sw */
/**
* @brief Function implementing the limit_sw thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_tk_limit_sw */
void start_tk_limit_sw(void *argument)
{
  /* USER CODE BEGIN start_tk_limit_sw */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start_tk_limit_sw */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
