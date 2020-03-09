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
extern uint8_t RxData[8];
extern CAN_RxHeaderTypeDef   RxHeader;

extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern xTimerHandle broadcast_timer;
static void prvAutoReloadTimerCallback( TimerHandle_t xTimer )
{
	//广播
	//发送消息测试，包括CAN MODBUS
	//dummy data
	QUEUE_STRUCT queue_id;
	portBASE_TYPE status;
	queue_id.property=0;             //can send
	queue_id.can_priority=0x05;      //can priority
	queue_id.can_source=0x03;        //can source
	queue_id.can_target=0x00;        //can target
	queue_id.can_command=0x43;       //can command
	queue_id.can_if_last=0x00;       //can if last
	queue_id.can_if_return=0x00;     //can if return
	queue_id.can_if_ack=0x00;        //can if ack
	queue_id.can_version=0x07;       //can version
	queue_id.data[0]=0x57;
	queue_id.data[1]=0x39;
	queue_id.data[2]=0xF6;
	queue_id.data[3]=0x90;
	queue_id.data[4]=0x13;
	queue_id.data[5]=0x52;
	queue_id.data[6]=0x30;
	queue_id.data[7]=0x33;
	queue_id.length=8;
	queuespace= uxQueueSpacesAvailable( send_queueHandle );
	uint8_t tmp=can_send(queue_id);
	status = xQueueSendToBack(send_queueHandle, &queue_id, 0);
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

void start_soft_timer(void)
{
	broadcast_timer = xTimerCreate( "AutoReload",timer_period,pdTRUE,0,prvAutoReloadTimerCallback );
	xTimerStart(broadcast_timer,0);
	;
}
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
uint8_t send_queueBuffer[ 256 * sizeof( QUEUE_STRUCT ) ];
osStaticMessageQDef_t send_queueControlBlock;
osMessageQueueId_t rece_queueHandle;
uint8_t rece_queueBuffer[ 256 * sizeof( QUEUE_STRUCT ) ];
osStaticMessageQDef_t rece_queueControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//QUEUE_STRUCT can_rece;
  /* Get RX message */
  

	/*
	if(rece_queueHandle!=NULL)
	{
		portBASE_TYPE status;
		status = xQueueSendToBack(rece_queueHandle, &can_rece, 0);
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
	*/
	//xTaskNotifyGive( master_orderHandle );
	
}
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
  send_queueHandle = osMessageQueueNew (256, sizeof(QUEUE_STRUCT), &send_queue_attributes);

  /* definition and creation of rece_queue */
  const osMessageQueueAttr_t rece_queue_attributes = {
    .name = "rece_queue",
    .cb_mem = &rece_queueControlBlock,
    .cb_size = sizeof(rece_queueControlBlock),
    .mq_mem = &rece_queueBuffer,
    .mq_size = sizeof(rece_queueBuffer)
  };
  rece_queueHandle = osMessageQueueNew (256, sizeof(QUEUE_STRUCT), &rece_queue_attributes);

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
	//启动软件定时器
	start_soft_timer();
	//定时器初始化
	timer_start();
	can_start();
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
	//QUEUE_STRUCT queue_id;
	//portBASE_TYPE status;
  /* Infinite loop */
  for(;;)
  {
		#ifdef DEBUG_OUTPUT
		//printf("%s\n","start default task");
		#endif
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
	uint32_t notify_use=0;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
                         0xffffffff,          /* Clear all bits on exit. */
                         &notify_use, /* Receives the notification value. */
                         portMAX_DELAY );
		if(notify_use!=0)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","start tk pid output");
			#endif
			
			notify_use=0;
		}
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
	QUEUE_STRUCT id;
  portBASE_TYPE status;
	uint32_t notify_use=0;
  /* Infinite loop */
  for(;;)
  {
    #ifdef DEBUG_OUTPUT
			printf("%s\n","start tk send order");
		#endif	

		//等待发送队列有消息
		status = xQueueReceive(send_queueHandle, &id, portMAX_DELAY);
		if(status==pdPASS)
		{
			//id为发送结构体
			if(id.property==0)
			{
				#ifdef DEBUG_OUTPUT
				printf("%s, id : %d\n","queue receive  can send order",id.can_command);
				#endif
				can_send(id);
			}
			if(id.property==1)
			{
				modbus_send(id);
			}
		}
		
		//接收中断通知进行发送结果判断，CAN发送中断 + RS485发送中断
		//xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
    //                     0xffffffff,          /* Clear all bits on exit. */
     //                    &notify_use, /* Receives the notification value. */
     //                    portMAX_DELAY );
		
		//if(notify_use!=0)
		{
			#ifdef DEBUG_OUTPUT
			//printf("%s,%u\n","send order task receive notify form isr",notify_use);
			#endif
		}
		
		/* Block to wait for prvTask2() to notify this task. */
    //ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		
		notify_use=0;
		
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
	uint32_t notify_use=0;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
                         0xffffffff,          /* Clear all bits on exit. */
                         &notify_use, /* Receives the notification value. */
                         portMAX_DELAY );
		if(notify_use!=0)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","start tk communicate monitor");
			#endif
			
			notify_use=0;
		}
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
	uint32_t notify_use=0;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
                         0xffffffff,          /* Clear all bits on exit. */
                         &notify_use, /* Receives the notification value. */
                         portMAX_DELAY );
		if(notify_use!=0)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","start tk conflict monitor");
			#endif
			
			notify_use=0;
		}
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
	uint32_t notify_use=0;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
                         0xffffffff,          /* Clear all bits on exit. */
                         &notify_use, /* Receives the notification value. */
                         100 );
		if(notify_use!=0)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","start tk sensor monitor");
			#endif
			
			notify_use=0;
		}
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
	uint32_t notify_use=0;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
                         0xffffffff,          /* Clear all bits on exit. */
                         &notify_use, /* Receives the notification value. */
                         portMAX_DELAY );
		if(notify_use!=0)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","start tk grating monitor");
			#endif
			
			notify_use=0;
		}
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
	uint32_t notify_use;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
                         0xffffffff,          /* Clear all bits on exit. */
                         &notify_use, /* Receives the notification value. */
                         portMAX_DELAY );
		if(notify_use!=0)
		{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","start tk posture monitor");
			#endif
			
			notify_use=0;
		}
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
	uint32_t notify_use=0;
	QUEUE_STRUCT id;
  portBASE_TYPE status;
  /* Infinite loop */
  for(;;)
  {
		//xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
    //                     0xffffffff,          /* Clear all bits on exit. */
    //                     &notify_use, /* Receives the notification value. */
    //                     portMAX_DELAY );
		//if(notify_use!=0)
		//{
			#ifdef DEBUG_OUTPUT
			printf("%s\n","start tk master order");
			#endif
			QUEUE_STRUCT tmp;
			//queue space 
			//等待发送队列有消息
			status = xQueueReceive(rece_queueHandle, &id, portMAX_DELAY);
			if(status==pdPASS)
			{
				//解析ID
				id.property=(id.RxHeader.ExtId & MASK_PRIORITY)>>26;
				if(id.property==0)
				{
					//收到的CAN消息有误，不动作
					;
				}else{
					//解析收到的数据，解码并压入发送队列，发送485指令命令电机动作
					uint8_t tmp_command_id=(id.RxHeader.ExtId & MASK_COMMAND)>>9;
					uint8_t tmp_priority=(id.RxHeader.ExtId & MASK_PRIORITY)>>26;
					uint8_t tmp_if_ack=(id.RxHeader.ExtId & MASK_IF_ACK) >> 6;
					uint8_t tmp_if_return=(id.RxHeader.ExtId & MASK_IF_RETURN) >> 7;
					uint8_t tmp_if_last=(id.RxHeader.ExtId & MASK_IF_LAST) >> 8;
					uint8_t tmp_can_version=(id.RxHeader.ExtId & MASK_VERSION) >> 0;
					uint8_t tmp_target=(id.RxHeader.ExtId & MASK_TARGET) >> 16;
					
					//判断ACK是否需要立即回复发送方
					if(tmp_if_ack==1)
					{
						//tmp填充为cansend
						tmp.property=0;                     //can send 
						tmp.can_priority=0x01;              //can priority, ACK帧
						tmp.can_source=0x03;                //can source， 本模块为3号模块
						tmp.can_target=0x00;                //can target， 00：主控
						tmp.can_command=tmp_command_id;     //can command
						tmp.can_if_last=0x00;               //can if last, 0: 最后一帧
						tmp.can_if_return=0x00;             //can if return， 0：保留
						tmp.can_if_ack=0x00;                //can if ack， 0：无需ACK
						tmp.can_version=0x01;               //can version，1：暂定为1
						
						tmp.length=id.RxHeader.DLC;         //ack帧需要返回数据,所有数据原路返回
						memcpy(tmp.data,id.data,id.RxHeader.DLC);
						
						status = xQueueSendToBack(send_queueHandle, &tmp, 0);
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
					
					if(tmp_command_id>CAN_COMMAND_NUMBER)
					{
						//命令ID错误
						#ifdef DEBUG_OUTPUT
						printf("%s\n","command id error");
						#endif
					}
					else
					{
						if(tmp_target==31)
						{
							//广播ID， 只有一组急停指令
							uint8_t data=0;
							uint8_t para=tmp_if_return;
							command_to_function[tmp_command_id](&data,para);
						}
						else
						{
							if(
							tmp_command_id==0  || 
						  tmp_command_id==2  ||
						  tmp_command_id==4  ||
						  tmp_command_id==5  ||
						  tmp_command_id==6  ||
						  tmp_command_id==10 ||
						  tmp_command_id==15 ||
						  tmp_command_id==16 ||
						  tmp_command_id==17 ||
						  tmp_command_id==18 ||
						  tmp_command_id==19)
							{
								uint8_t data=0;
								uint8_t para=tmp_if_return;
								command_to_function[tmp_command_id](&data,para);
							}
							if(tmp_command_id==1  ||
								 tmp_command_id==2  ||
							   tmp_command_id==3  ||
							   tmp_command_id==4  ||
							   tmp_command_id==5  ||
							   tmp_command_id==7  ||
							   tmp_command_id==10 ||
							   tmp_command_id==19)
							{
								uint8_t data[8];
								//memcpy(data,id.data,id.RxHeader.DLC);
								
								data[0]=id.data[0];
								data[1]=id.data[1];
								data[2]=id.data[2];
								data[3]=id.data[3];
								data[4]=id.data[4];
								data[5]=id.data[5];
								data[6]=id.data[6];
								data[7]=id.data[7];
								
								uint32_t para=id.RxHeader.DLC;
								para|=(tmp_if_return << 4);   //bit 4表示是否返回return帧
								para|=(tmp_if_last << 5);     //bit 5表示是否是最后一帧
								command_to_function[tmp_command_id](data,para);
							}
						}
					//根据命令电机映射表找到要动作的电机，并将参数压入电机中的命令结构
					uint8_t motor_id=command_to_motor[tmp_command_id];
					}
				}
			}
			else
			{
				;
			}
			notify_use=0;
			;
		//}
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
	uint32_t notify_use=0;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait( 0x00,               /* Don't clear any bits on entry. */
                         0xffffffff,          /* Clear all bits on exit. */
                         &notify_use, /* Receives the notification value. */
                         portMAX_DELAY );
		//下降沿触发，停止电机
		HAL_GPIO_WritePin(motor_array[notify_use - 1].gpio_output[ENABLE_MOTOR].gpio_port,motor_array[notify_use - 1].gpio_output[ENABLE_MOTOR].pin_number, GPIO_PIN_RESET);
		//更新限位开关状态
		uint8_t switch_status_get_res=0;
		switch_status_get_res=switchGet(notify_use - 1);
		if(switch_status_get_res != 0)
		{
			if(switch_status_get_res==ERROR_FUNC_BUSY)
			{
				while(switch_status_get_res==ERROR_FUNC_BUSY)
				{
					HAL_Delay(10);//忙等待
					switch_status_get_res=switchGet(notify_use - 1);
				}
			}
			;
		}else{
			;
		}
		
    osDelay(1);
  }
  /* USER CODE END start_tk_limit_sw */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
