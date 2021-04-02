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
xSemaphoreHandle 	Semaphore_Modbus_Rx, Semaphore_Modbus_Tx, 
									Semaphore_Update, 
									Semaphore_Jump, 
									Semaphore_Сonfirm_Update,
									TBUS_Semaphore_Modbus_Rx, TBUS_Semaphore_Modbus_Tx; 

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
uint8_t boot_receiveBuffer[256];

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
volatile uint16_t remain = 0;

volatile uint8_t TBUS_boot_transmitBuffer[8];
uint8_t TBUS_boot_receiveBuffer[256];

volatile uint16_t error_crc_counter = 0;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask01Handle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;


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
void TBUS_Modbus_Receive_Task(void const * argument);
static uint16_t local_crc16(const uint8_t * adr_buffer, const uint32_t byte_cnt, const uint16_t * table);
	
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */

uint16_t table[] =
{
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
	
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
		vSemaphoreCreateBinary(TBUS_Semaphore_Modbus_Rx);
		
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
  //osThreadDef(myTask05, Update_Flash_Task, osPriorityNormal, 0, 128);
  //myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Jump_Task, osPriorityNormal, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(myTask07, TBUS_Modbus_Receive_Task, osPriorityNormal, 0, 128);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);
	
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
    vTaskDelay(1000);
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
    vTaskDelay(1000);
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
				
				if (worker_status == 1)
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
				
				if (worker_status == 2)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Размер",font_8x15_RU,1);										
					ssd1306_SetCursor(0,15);	
					ssd1306_WriteString("ПО",font_8x15_RU,1);					
					ssd1306_WriteString(":",font_8x14,1);										
					ssd1306_SetCursor(0,30);					
					snprintf(buffer, sizeof buffer, "%d", byte_size);				
					ssd1306_WriteString(buffer,font_8x14,1);	
						
					ssd1306_UpdateScreen();	
				}				
				
				if (worker_status == 3)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Загруз",font_8x15_RU,1);					
					ssd1306_WriteString("-",font_8x14,1);					
					ssd1306_SetCursor(0,15);	
					ssd1306_WriteString("ка ПО",font_8x15_RU,1);					
					ssd1306_WriteString(":",font_8x14,1);					
					ssd1306_SetCursor(0,30);	
					snprintf(buffer, sizeof buffer, "%d %%", (byte_counter * 100) / byte_size);				
					ssd1306_WriteString(buffer,font_8x14,1);	
						
					ssd1306_UpdateScreen();	
				}					
				
				if (worker_status == 4)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,15);
					ssd1306_WriteString("УСПЕШНО",font_8x15_RU,1);										
						
					ssd1306_UpdateScreen();	
				}	
			
				if (worker_status == 5)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Контр",font_8x15_RU,1);					
					ssd1306_WriteString(".",font_8x14,1);					
					ssd1306_SetCursor(0,15);	
					ssd1306_WriteString("сумма",font_8x15_RU,1);	
					ssd1306_WriteString(":",font_8x14,1);						
					ssd1306_SetCursor(0,30);	
					snprintf(buffer, sizeof buffer, "%X", crc_flash);				
					ssd1306_WriteString(buffer,font_8x14,1);	
						
					ssd1306_UpdateScreen();	
				}				
	
			vTaskDelay(100);
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
		
							
		
		/* 2: Команда на подтверждение того, что контроллер перешел в режим загрузчика */
		if( boot_receiveBuffer[0] == 0xAE && boot_receiveBuffer[1] == 0x3E && boot_receiveBuffer[2] == 0xFC )
		{			
			HAL_UART_Transmit(&huart2, (uint8_t *) &boot_receiveBuffer[0], 3, 500);  
			
			worker_status = 1;			
		}
		
		/* 3: Получаем размер прошивки в байтах */
		if( boot_receiveBuffer[0] == 0xAC && worker_status == 1)
		{				
			calculate_crc = local_crc16(boot_receiveBuffer, 5, table);
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
			calculate_crc = local_crc16(boot_receiveBuffer, 1, table);
			packet_crc = (boot_receiveBuffer[2]<<8) + boot_receiveBuffer[1];
			
			if( calculate_crc == packet_crc )
			{			
				
				FLASH_EraseInitTypeDef EraseInitStruct;					
				uint32_t PAGEError = 0;
				EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
				EraseInitStruct.Banks = 1;
				EraseInitStruct.Page = 32; /* 0x8010000 */
				EraseInitStruct.NbPages = 65;	/* 65 * 2000 = 130000 байт */		
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
				
				calculate_crc = local_crc16(boot_receiveBuffer, packet_size+1, table);
				packet_crc = (boot_receiveBuffer[packet_size + 2]<<8) + boot_receiveBuffer[packet_size + 1];

			
				if( calculate_crc == packet_crc )
				{						
						/* Перекладываем из модбас пакета в буффер для записи, чистим данные от crc и размера */
						for(int i = 0; i < packet_size; i++)
						{
							data_from_modbus[i] = boot_receiveBuffer[i+1];
						}						
						
						/* Программируем флеш */						
						for(uint16_t i = 0; i < packet_size; i+=8)
						{								
								data_to_flash = 0x00;
								
								if( byte_size - byte_counter < 8 ) remain = byte_size - byte_counter;
								else remain = 8;							
							
								for(uint16_t y = 0; y < 8; y++)
								{										
										if( y >= remain ) 
										{
												data_to_flash += ((uint64_t) 0xFF) << y*8;													
										}
										else
										{
												data_to_flash += ((uint64_t) data_from_modbus[i + y]) << y*8;													
										}
								}
						
								HAL_FLASH_Unlock();							
								status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (APP_START_ADDRESS) + 8*byte_bunch, data_to_flash);										
								HAL_FLASH_Lock();							
								 
								byte_counter += 8;	
								byte_bunch++;																				
						}																				
						
						/* Если все хорошо высылаем подтверждение */
						boot_transmitBuffer[0] = 0xAE;
						boot_transmitBuffer[1] = 0x3E;
						boot_transmitBuffer[2] = 0xFC;
						HAL_UART_Transmit_DMA(&huart2, (uint8_t *) &boot_transmitBuffer[0], 3); 						
						
						
						if (byte_counter >= byte_size) 
						{
								worker_status = 4;
								vTaskDelay(3000);
								xSemaphoreGive( Semaphore_Jump );		   
								//JumpToApplication(APP_START_ADDRESS);																			
						}	
				}
				else
				{
						error_crc_counter ++;
					
						/* Если пакет битый просим повторить */				
						HAL_UART_Transmit_DMA(&huart2, (uint8_t *) 0xFF, 1); 
				}			
		}		
		 
		HAL_UART_Receive_DMA(&huart2, boot_receiveBuffer, 255);
		
		 
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
		//xSemaphoreTake( Semaphore_Update, portMAX_DELAY );					

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
					
					if (worker_status == 4)
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
						vTaskDelay(5);
											
						status2 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_CRC_ADR, crc_flash);																	
						vTaskDelay(5);
						
						status3 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_SIZE, byte_size);						
						vTaskDelay(5);						
						
						HAL_FLASH_Lock();
						
						worker_status = 5;
						vTaskDelay(3000);									
						
						JumpToApplication(APP_START_ADDRESS);											
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



void TBUS_Modbus_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Receive_Task */
	
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake( TBUS_Semaphore_Modbus_Rx, portMAX_DELAY );					
						
		__HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_IDLEF); 				
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
		HAL_UART_DMAStop(&huart3);
		
							
		
		/* 2: Команда на подтверждение того, что контроллер перешел в режим загрузчика */
		if( TBUS_boot_receiveBuffer[0] == 0xAE && TBUS_boot_receiveBuffer[1] == 0x3E && TBUS_boot_receiveBuffer[2] == 0xFC )
		{			
			HAL_UART_Transmit(&huart3, (uint8_t *) &TBUS_boot_receiveBuffer[0], 3, 500);  
			
			worker_status = 1;			
		}
		
		/* 3: Получаем размер прошивки в байтах */
		if( TBUS_boot_receiveBuffer[0] == 0xAC && worker_status == 1)
		{			
			
			calculate_crc = local_crc16(TBUS_boot_receiveBuffer, 5, table);
			packet_crc = (TBUS_boot_receiveBuffer[6]<<8) + TBUS_boot_receiveBuffer[5];
			
			if( calculate_crc == packet_crc )
			{
				byte_size = (TBUS_boot_receiveBuffer[1]<<24) + (TBUS_boot_receiveBuffer[2]<<16) + (TBUS_boot_receiveBuffer[3]<<8) + TBUS_boot_receiveBuffer[4];
				
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *) &TBUS_boot_receiveBuffer[0], 7); 	
			
				worker_status = 2;			
			}
		}		
				
		/* 4: Получаем команду на очистку флеш */
		if( TBUS_boot_receiveBuffer[0] == 0xAF && worker_status == 2)
		{				
			calculate_crc = local_crc16(TBUS_boot_receiveBuffer, 1, table);
			packet_crc = (TBUS_boot_receiveBuffer[2]<<8) + TBUS_boot_receiveBuffer[1];
			
			if( calculate_crc == packet_crc )
			{			
				
				FLASH_EraseInitTypeDef EraseInitStruct;					
				uint32_t PAGEError = 0;
				EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
				EraseInitStruct.Banks = 1;
				EraseInitStruct.Page = 32; /* 0x8010000 */
				EraseInitStruct.NbPages = 65;	/* 65 * 2000 = 130000 байт */		
				status = HAL_FLASH_Unlock();	
				vTaskDelay(5);
				//osDelay(5);
				__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);				
				status = HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);				
				//status = HAL_FLASH_GetError();				
				HAL_FLASH_Lock();				
				
				
				HAL_UART_Transmit_DMA(&huart3, (uint8_t *) &TBUS_boot_receiveBuffer[0], 3); 	
			
				worker_status = 3;			
				
				byte_bunch = 0;
				byte_counter = 0;				
				
			}
		}	
		
		
		/* 5: Получаем данные прошивки */
		if( worker_status == 3 )
		{
				/* Размер пакета без CRC */
				packet_size = TBUS_boot_receiveBuffer[0];
				
				calculate_crc = local_crc16(TBUS_boot_receiveBuffer, packet_size + 1, table);
				packet_crc = (TBUS_boot_receiveBuffer[packet_size + 2]<<8) + TBUS_boot_receiveBuffer[packet_size + 1];

			
				if( calculate_crc == packet_crc )
				{						
						/* Перекладываем из модбас пакета в буффер для записи, чистим данные от crc и размера */
						for(int i = 0; i < packet_size; i++)
						{
							data_from_modbus[i] = TBUS_boot_receiveBuffer[i+1];
						}						
						
						/* Программируем флеш */						
						for(uint16_t i = 0; i < packet_size; i+=8)
						{								
								data_to_flash = 0x00;
								
								if( byte_size - byte_counter < 8 ) remain = byte_size - byte_counter;
								else remain = 8;							
							
								for(uint16_t y = 0; y < 8; y++)
								{										
										if( y >= remain ) 
										{
												data_to_flash += ((uint64_t) 0xFF) << y*8;													
										}
										else
										{
												data_to_flash += ((uint64_t) data_from_modbus[i + y]) << y*8;													
										}
								}
						
								HAL_FLASH_Unlock();							
								status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (APP_START_ADDRESS) + 8*byte_bunch, data_to_flash);										
								HAL_FLASH_Lock();							
								 
								byte_counter += 8;	
								byte_bunch++;																				
						}																				
						
						/* Если все хорошо высылаем подтверждение */
						TBUS_boot_transmitBuffer[0] = 0xAE;
						TBUS_boot_transmitBuffer[1] = 0x3E;
						TBUS_boot_transmitBuffer[2] = 0xFC;
						HAL_UART_Transmit_DMA(&huart3, (uint8_t *) &TBUS_boot_transmitBuffer[0], 3); 						
						
						
						if (byte_counter >= byte_size) 
						{
								worker_status = 4;
								vTaskDelay(3000);
								xSemaphoreGive( Semaphore_Jump );		   
								//JumpToApplication(APP_START_ADDRESS);																			
						}	
				}
				else
				{
						error_crc_counter ++;
					
						/* Если пакет битый просим повторить */				
						HAL_UART_Transmit_DMA(&huart3, (uint8_t *) 0xFF, 1); 
				}			
		}		
		 
		HAL_UART_Receive_DMA(&huart3, TBUS_boot_receiveBuffer, 255);
		
		 
  }  
}


	
static uint16_t local_crc16(const uint8_t * adr_buffer, uint32_t byte_cnt, const uint16_t * table)
{
	uint16_t crc = 0xFFFF;


	uint8_t lut;
	/* CRC Generation Function */
	while( byte_cnt--) /* pass through message buffer */
	{
		lut = crc^ *adr_buffer++;
		crc  = (crc >> 8) ^ table[lut];
	}
	return crc;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

