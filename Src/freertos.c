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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "gpio.h"
#include "Task_manager.h"
#include "Flash_manager.h"
#include "usart.h"
#include "fonts.h"
#include "iwdg.h"
#include "stm32l4xx_hal_flash.h"
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
xSemaphoreHandle 	Semaphore_Modbus_Rx, Semaphore_Modbus_Tx, Semaphore_Update, Semaphore_Jump, Semaphore_Сonfirm_Update;

extern FontDef font_7x12_RU;
extern FontDef font_7x12;
extern FontDef font_8x15_RU;
extern FontDef font_8x14;
extern FontDef font_5x10_RU;
extern FontDef font_5x10;

uint8_t error_crc = 0;
volatile static uint8_t worker_status = 0;
uint8_t status = 0;

uint8_t status1 = 0;
uint8_t status2 = 0;
uint8_t status3 = 0;


volatile uint8_t boot_transmitBuffer[8];
volatile uint8_t boot_receiveBuffer[256];

volatile static uint32_t byte_size = 0;
volatile uint16_t crc_data = 0;
volatile uint16_t byte_bunch = 0;
volatile uint32_t byte_counter = 0;
volatile uint16_t crc_flash = 0;
volatile uint64_t data_to_flash = 0;

volatile uint16_t packet_crc = 0;	
volatile uint16_t calculate_crc = 0;	
volatile uint8_t packet_size = 0;
volatile static uint8_t flash_byte_counter = 0;
volatile uint8_t data_from_modbus[255];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask01Handle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void JumpToApplication(uint32_t ADDRESS);   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Lights_Task(void const * argument);
void Display_Task(void const * argument);
void Modbus_Receive_Task(void const * argument);
void Modbus_Transmit_Task(void const * argument);
void Update_Flash_Task(void const * argument);
void Jump_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

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

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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
		vSemaphoreCreateBinary(Semaphore_Modbus_Rx);
		vSemaphoreCreateBinary(Semaphore_Modbus_Tx);
		vSemaphoreCreateBinary(Semaphore_Update);
		vSemaphoreCreateBinary(Semaphore_Jump);
		vSemaphoreCreateBinary(Semaphore_Сonfirm_Update);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask01 */
  osThreadDef(myTask01, Lights_Task, osPriorityNormal, 0, 128);
  myTask01Handle = osThreadCreate(osThread(myTask01), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, Display_Task, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, Modbus_Receive_Task, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, Modbus_Transmit_Task, osPriorityNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, Update_Flash_Task, osPriorityNormal, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  //osThreadDef(myTask06, Jump_Task, osPriorityNormal, 0, 128);
  //myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	
	Task_manager_Init();
  /* Infinite loop */
  for(;;)
  {
		Task_manager_LoadCPU();		
		HAL_IWDG_Refresh(&hiwdg);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Lights_Task */
/**
* @brief Function implementing the myTask09 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Lights_Task */
void Lights_Task(void const * argument)
{
  /* USER CODE BEGIN Lights_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END Lights_Task */
}

/* USER CODE BEGIN Header_Display_Task */
/**
* @brief Function implementing the myTask11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_Task */
void Display_Task(void const * argument)
{
  /* USER CODE BEGIN Display_Task */
	uint8_t temp_stat = 0;
	char buffer[64];
	// CS# (This pin is the chip select input. (active LOW))
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	
	ssd1306_Init();
	
  /* Infinite loop */
  for(;;)
  {		
		
				if(error_crc == 1)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Ошибка",font_8x15_RU,1);
					ssd1306_SetCursor(0,15);
					ssd1306_WriteString("CRC",font_8x14,1);				

					ssd1306_UpdateScreen();		
				}
				
				if(error_crc == 0 && worker_status == 0)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Загруз",font_8x15_RU,1);
					ssd1306_WriteString("-",font_8x14,1);
					ssd1306_SetCursor(0,15);				
					ssd1306_WriteString("чик",font_8x15_RU,1);

					snprintf(buffer, sizeof buffer, " %.01f", VERSION);				
					ssd1306_WriteString(buffer,font_8x14,1);	
	
					ssd1306_UpdateScreen();		
				}
				
				if (worker_status == 1 || worker_status == 2 || worker_status == 3)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Обнов",font_8x15_RU,1);					
					ssd1306_WriteString("-",font_8x14,1);					
					ssd1306_SetCursor(0,15);	
					ssd1306_WriteString("ление",font_8x15_RU,1);					
					ssd1306_SetCursor(0,30);	
					ssd1306_WriteString("ПО",font_8x15_RU,1);					
					ssd1306_WriteString("...",font_8x14,1);
						
					ssd1306_UpdateScreen();	
				}
				
				if (worker_status == 4)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,15);
					ssd1306_WriteString("УСПЕШНО",font_8x15_RU,1);										
						
					ssd1306_UpdateScreen();	
				}				
			
	
			osDelay(100);
  }
  /* USER CODE END Display_Task */
}

/* USER CODE BEGIN Header_Modbus_Receive_Task */
/**
* @brief Function implementing the myTask13 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Modbus_Receive_Task */
void Modbus_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Receive_Task */
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Modbus_Rx, portMAX_DELAY );					
						
		__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
		HAL_UART_DMAStop(&huart2);
		
							
		
		/* 2: Команда подтверждение того, что контроллер перешел в режим загрузчика */
		if( boot_receiveBuffer[0] == 0xAE && boot_receiveBuffer[1] == 0x3E && boot_receiveBuffer[2] == 0xFC )
		{			
			HAL_UART_Transmit(&huart2, (uint8_t *) &boot_receiveBuffer[0], 3, 500);  
			
			worker_status = 1;			
		}
		
		/* 3: Получаем размер прошивки в байтах */
		if( boot_receiveBuffer[0] == 0xAC && worker_status == 1)
		{				
			calculate_crc = crc16(&boot_receiveBuffer[0], 5);
			packet_crc = (boot_receiveBuffer[6]<<8) + boot_receiveBuffer[5];
			
			if( calculate_crc == packet_crc )
			{
				byte_size = (boot_receiveBuffer[1]<<24) + (boot_receiveBuffer[2]<<16) + (boot_receiveBuffer[3]<<8) + boot_receiveBuffer[4];
				
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *) &boot_receiveBuffer[0], 7); 	
			
				worker_status = 2;			
			}
		}		
				
		/* 4: Получаем команду на очистку флеш */
		if( boot_receiveBuffer[0] == 0xAF && worker_status == 2)
		{				
			calculate_crc = crc16(&boot_receiveBuffer[0], 1);
			packet_crc = (boot_receiveBuffer[2]<<8) + boot_receiveBuffer[1];
			
			if( calculate_crc == packet_crc )
			{			
				
				FLASH_EraseInitTypeDef EraseInitStruct;					
				uint32_t PAGEError = 0;
				EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
				EraseInitStruct.Banks = 1;
				EraseInitStruct.Page = 32;
				EraseInitStruct.NbPages = 40;			
				status = HAL_FLASH_Unlock();	
				vTaskDelay(5);
				//osDelay(5);
				__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);				
				status = HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);				
				//status = HAL_FLASH_GetError();				
				HAL_FLASH_Lock();				
				
				
				HAL_UART_Transmit_DMA(&huart2, (uint8_t *) &boot_receiveBuffer[0], 3); 	
			
				worker_status = 3;			
				
				byte_bunch = 0;
				byte_counter = 0;
				
				
			}
		}	
		
		
		/* 5: Получаем данные прошивки */
		if( worker_status == 3 )
		{
				/* Размер пакета без CRC */
				packet_size = boot_receiveBuffer[0];
				
				calculate_crc = crc16((uint8_t*)&boot_receiveBuffer[0], packet_size+1);
				packet_crc = (boot_receiveBuffer[packet_size + 2]<<8) + boot_receiveBuffer[packet_size + 1];
				
				if( calculate_crc == packet_crc )
				{				
						
						/* Перекладываем из модбас пакета в буффер для записи, чистим данные от crc и размера */
						for(int i = 0; i < packet_size; i++)
						{
							data_from_modbus[i] = boot_receiveBuffer[i+1];
						}
						
						
						data_to_flash = 0x0;
						
						
						/* Программируем флеш с идентификацией крайнего пакета */										
						//if( packet_size % 8 == 0 )
						{
								for(uint16_t i = 0; i < packet_size; i+=8)
								{														
										
										data_to_flash = ((uint64_t) data_from_modbus[i + 0]) + 
										((uint64_t) (data_from_modbus[i + 1]) << 8) + 
										((uint64_t) (data_from_modbus[i + 2]) << 16) + 
										((uint64_t) (data_from_modbus[i + 3]) << 24) + 
										((uint64_t) (data_from_modbus[i + 4]) << 32) + 
										((uint64_t) (data_from_modbus[i + 5]) << 40) + 
										((uint64_t) (data_from_modbus[i + 6]) << 48) + 
										((uint64_t) (data_from_modbus[i + 7]) << 56);							
						
												
								
										HAL_FLASH_Unlock();							
										status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (APP_START_ADDRESS) + 8*byte_bunch, data_to_flash);										
										HAL_FLASH_Lock();							
										 
										byte_counter += 8;	
										byte_bunch++;																				
								}														
						}

						
						/* Если все хорошо высылаем подтверждение */
						boot_transmitBuffer[0] = 0xAE;
						boot_transmitBuffer[1] = 0x3E;
						boot_transmitBuffer[2] = 0xFC;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t *) &boot_transmitBuffer[0], 3); 						
						
						
						if (byte_counter >= byte_size) 
						{
								worker_status = 4;
								vTaskDelay(1000);
								JumpToApplication(APP_START_ADDRESS);											
								
						}	
				}
				else
				{
						/* Если пакет битый просим повторить */				
						HAL_UART_Transmit_DMA(&huart2, (uint8_t *) 0xFF, 1); 
				}			
		}		
		 
		HAL_UART_Receive_DMA(&huart2, boot_receiveBuffer, 255);
		
		//xSemaphoreGive( Semaphore_Update );		    
  }
  /* USER CODE END Modbus_Receive_Task */
}

/* USER CODE BEGIN Header_Modbus_Transmit_Task */
/**
* @brief Function implementing the myTask14 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Modbus_Transmit_Task */
void Modbus_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Transmit_Task */
	
  /* Infinite loop */
  for(;;)
  {
			xSemaphoreTake( Semaphore_Modbus_Tx, portMAX_DELAY );

			HAL_UART_Transmit_DMA(&huart2, boot_transmitBuffer, 8);    
  }
  /* USER CODE END Modbus_Transmit_Task */
}

/* USER CODE BEGIN Header_Update_Flash_Task */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Update_Flash_Task */
void Update_Flash_Task(void const * argument)
{
  /* USER CODE BEGIN Update_Flash_Task */

	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( Semaphore_Update, portMAX_DELAY );					

		if( worker_status == 3 )
		{
				/* Перекладываем из модбас пакета в локальный буффер, для очистки */
				for(uint16_t i = 0; i < byte_size; i++)
				{
					//data_from_modbus[i] = boot_receiveBuffer[i];
				}
//				
//				/* Программируем флеш */
//				//HAL_FLASH_Unlock();
//				for(uint16_t i = 0; i < byte_size-1; i++)
//				{
//						data = ((uint64_t) data_from_modbus[i*8 + 0]) + 
//							((uint64_t) (data_from_modbus[i*8 + 1]) << 8) + 
//							((uint64_t) (data_from_modbus[i*8 + 2]) << 16) + 
//							((uint64_t) (data_from_modbus[i*8 + 3]) << 24) + 
//							((uint64_t) (data_from_modbus[i*8 + 4]) << 32) + 
//							((uint64_t) (data_from_modbus[i*8 + 5]) << 40) + 
//							((uint64_t) (data_from_modbus[i*8 + 6]) << 48) + 
//							((uint64_t) (data_from_modbus[i*8 + 7]) << 56);
//					
//																			
//							//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (APP_START_ADDRESS) + 8*byte_bunch, data);										
//							
//						
//							//byte_bunch++; byte_counter += 8;
//					
//							if (byte_counter >= byte_size) 
//							{
//								//xSemaphoreGive( Semaphore_Jump );		
//							}			
//				}
//				//HAL_FLASH_Lock();	
				
				/* Отправляем подтверждение */
				// xSemaphoreGive(Semaphore_Сonfirm_Update);
		}
	}

  
  /* USER CODE END Update_Flash_Task */
}

/* USER CODE BEGIN Header_Jump_Task */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Jump_Task */
void Jump_Task(void const * argument)
{
  /* USER CODE BEGIN Jump_Task */
  /* Infinite loop */
  for(;;)
  {
					xSemaphoreTake( Semaphore_Jump, portMAX_DELAY );
    				
					crc_flash = flash_crc16(APP_START_ADDRESS, byte_size);
					
					if (crc_flash == crc_data)
					{
						
						FLASH_EraseInitTypeDef EraseInitStruct;							
						uint32_t PAGEError = 0;
						EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
						EraseInitStruct.Banks = 1;
						EraseInitStruct.Page = 7;
						EraseInitStruct.NbPages = 1;					

						
						status = HAL_FLASH_Unlock();	
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);						
						status1 = HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);															
						osDelay(5);
											
						status2 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_CRC_ADR, crc_data);																	
						osDelay(5);
						
						status3 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_SIZE, byte_size);						
						osDelay(5);					
						
						
						HAL_FLASH_Lock();
						
						worker_status = 4;
						osDelay(3000);									
						
						JumpToApplication(APP_START_ADDRESS);											
					}
					else 
					{
//						worker_status = 0;
//						byte_counter = 0;
//						byte_bunch = 0;		
//						error_crc = 1;
						
					}					
					
				
  }
  /* USER CODE END Jump_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void JumpToApplication(uint32_t ADDRESS)
{
		typedef  void (*pFunction)(void);
		uint32_t  JumpAddress = *(__IO uint32_t*)(ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;
        
    HAL_DeInit();
    
    __set_CONTROL(0); 
    __set_MSP(*(__IO uint32_t*) ADDRESS);
		SCB->VTOR = ADDRESS;
		Jump();		 
}     

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

