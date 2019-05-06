/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
portBASE_TYPE xStatusChS;
xTaskHandle xChangeSpeed_Handle;

portBASE_TYPE xStatusF;
xTaskHandle xFan_Handle;

static xQueueHandle SpeedQueue_handle = NULL;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void vChangeSpeed(void const * argument);

void vFan(void const * argument);




/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */


  /* definition and creation of ChangeSpeed task*/
  xStatusChS = xTaskCreate(vChangeSpeed, "Change Speed", 128, NULL, osPriorityNormal, &xChangeSpeed_Handle);
  if(xStatusChS == pdPASS)
  		printf("Task  Change Speed is created!\n");
  	else
  		printf("Task Change Speed is not created\n");

  xStatusF = xTaskCreate(vFan, (signed char*)"Fan", 128, NULL, osPriorityNormal, &xFan_Handle);
    if(xStatusF == pdPASS)
    		printf("Task Fan is created!\n");
    	else
    		printf("Task Fan is not created\n");


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
    SpeedQueue_handle = xQueueCreate(3, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */
}



/* USER CODE BEGIN Header vChangeSpeed */
/**
  * @brief  Function implementing the vChangeSpeed thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header vChangeSpeed */
void vChangeSpeed(void const * argument)
{
  /* USER CODE BEGIN */
  /* Infinite loop */
  uint8_t speed = 0;
  for(;;)
  {
    speed = 0;
    xQueueSendToBack(SpeedQueue_handle, &speed, 100/portTICK_RATE_MS);
    vTaskDelay(2000/portTICK_RATE_MS);
    speed = 100;
    xQueueSendToBack(SpeedQueue_handle, &speed, 100/portTICK_RATE_MS);
    vTaskDelay(2000/portTICK_RATE_MS);
  }
  /* USER CODE END vChangeSpeed */
}



/* USER CODE BEGIN Header vFan */
/**
  * @brief  Function implementing the vFan thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header vFan */
void vFan(void const * argument)
{
  /* USER CODE BEGIN */
  /* Infinite loop */
  uint8_t speed = 0;
  for(;;)
  {
	xQueueReceive(SpeedQueue_handle, &speed, 0);
	HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, GPIO_PIN_SET);
	vTaskDelay(50/portTICK_RATE_MS);
	HAL_GPIO_WritePin(Fan_GPIO_Port, Fan_Pin, GPIO_PIN_RESET);
	vTaskDelay(speed/portTICK_RATE_MS);
  }
  /* USER CODE END vFan */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
