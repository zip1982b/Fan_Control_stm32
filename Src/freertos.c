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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId change_speedHandle;
osThreadId work_fanHandle;
osMessageQId xQueueDIMHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void Start_change_speed(void const * argument);
void Start_work_fan(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* definition and creation of change_speed */
  osThreadDef(change_speed, Start_change_speed, osPriorityNormal, 0, 128);
  change_speedHandle = osThreadCreate(osThread(change_speed), NULL);

  /* definition and creation of work_fan */
  osThreadDef(work_fan, Start_work_fan, osPriorityNormal, 0, 128);
  work_fanHandle = osThreadCreate(osThread(work_fan), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of xQueueDIM */
  osMessageQDef(xQueueDIM, 5, uint8_t);
  xQueueDIMHandle = osMessageCreate(osMessageQ(xQueueDIM), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_Start_change_speed */
/**
  * @brief  Function implementing the change_speed thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Start_change_speed */
void Start_change_speed(void const * argument)
{

  /* USER CODE BEGIN Start_change_speed */
	uint8_t dim = 0;
  /* Infinite loop */
  for(;;)
  {
    //osDelay(1);
	  xQueueReceive(xQueueDIMHandle, &dim, 0);
	  HAL_GPIO_WritePin(Fan_in_GPIO_Port, Fan_in_Pin, GPIO_PIN_SET); //gpio_set_level(Fan_in, 1);
	  vTaskDelay(50 / portTICK_RATE_MS);
	  HAL_GPIO_WritePin(Fan_in_GPIO_Port, Fan_in_Pin, GPIO_PIN_RESET); //gpio_set_level(Fan_in, 0);
	  vTaskDelay(dim / portTICK_RATE_MS);
  }
  /* USER CODE END Start_change_speed */
}

/* USER CODE BEGIN Header_Start_work_fan */
/**
* @brief Function implementing the work_fan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_work_fan */
void Start_work_fan(void const * argument)
{
  /* USER CODE BEGIN Start_work_fan */
	uint8_t dim = 0;
  /* Infinite loop */
  for(;;)
  {
	  dim = 0; // 100%
	  xQueueSendToBack(xQueueDIMHandle, &dim, 100/portTICK_RATE_MS);
	  vTaskDelay(20000 / portTICK_RATE_MS);
	  dim = 100; // 50%
	  xQueueSendToBack(xQueueDIMHandle, &dim, 100/portTICK_RATE_MS);
	  vTaskDelay(20000 / portTICK_RATE_MS);
    //osDelay(1);
  }
  /* USER CODE END Start_work_fan */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
