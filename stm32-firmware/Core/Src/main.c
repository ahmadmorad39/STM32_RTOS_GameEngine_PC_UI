/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "rc522_rfid.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CENTER 3300
#define DEADZONE   1100
#define ADC_BUF_SIZE	2
#define RX_BUFFER_SIZE 10
#define NUM_VALID_UIDS (sizeof(valid_uids) / sizeof(valid_uids[0]))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum {
    DIR_NONE,
    DIR_UP,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_CENTER // Pressed
} Direction_t;

typedef struct {
    Direction_t direction;
    uint16_t x_val;
    uint16_t y_val;
} JoystickInput_t;

typedef enum {
    MODE_NONE = 0,
    MODE_SNAKE,
    MODE_DOORLOCK
} AppMode_t;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for joystickTask */
osThreadId_t joystickTaskHandle;
const osThreadAttr_t joystickTask_attributes = {
  .name = "joystickTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for rfidTask */
osThreadId_t rfidTaskHandle;
const osThreadAttr_t rfidTask_attributes = {
  .name = "rfidTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for joystickQueueHandle */
osMessageQueueId_t joystickQueueHandleHandle;
const osMessageQueueAttr_t joystickQueueHandle_attributes = {
  .name = "joystickQueueHandle"
};
/* Definitions for inputMutex */
osMutexId_t inputMutexHandle;
const osMutexAttr_t inputMutex_attributes = {
  .name = "inputMutex"
};
/* Definitions for Binary_Sem */
osSemaphoreId_t Binary_SemHandle;
const osSemaphoreAttr_t Binary_Sem_attributes = {
  .name = "Binary_Sem"
};
/* USER CODE BEGIN PV */
volatile uint16_t 	adcResultsDMA [ADC_BUF_SIZE];
volatile int 		adcConversionComplete= 0 ;
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t cmd_ready_flag = 0;
JoystickInput_t input;
const uint8_t valid_uids[][5] = {
    {0x83, 0x54, 0xAE, 0x27, 0xFD},  // UID 1
    {0x45, 0x0E, 0xAF, 0x02, 0xA5}   // UID 2
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
void Task_UART_Telemetry(void *argument);
void Task_Joystick(void *argument);
void Task_RFID(void *argument);
void Task_Control(void *argument);

/* USER CODE BEGIN PFP */
void Task_action(const char *format, ...);
void initialize_rfid(void);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  initialize_rfid();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of inputMutex */
  inputMutexHandle = osMutexNew(&inputMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Binary_Sem */
  Binary_SemHandle = osSemaphoreNew(1, 1, &Binary_Sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of joystickQueueHandle */
  joystickQueueHandleHandle = osMessageQueueNew (16, sizeof(uint16_t), &joystickQueueHandle_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of uartTask */
  uartTaskHandle = osThreadNew(Task_UART_Telemetry, NULL, &uartTask_attributes);

  /* creation of joystickTask */
  joystickTaskHandle = osThreadNew(Task_Joystick, NULL, &joystickTask_attributes);

  /* creation of rfidTask */
  rfidTaskHandle = osThreadNew(Task_RFID, NULL, &rfidTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(Task_Control, NULL, &controlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  if (HAL_OK != HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcResultsDMA, ADC_BUF_SIZE))
              	  Error_Handler();
  memset(rx_buffer, 0, sizeof(rx_buffer));
  HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
  // Start only control task at first
  osThreadSuspend(joystickTaskHandle);
  osThreadSuspend(uartTaskHandle);
  osThreadSuspend(rfidTaskHandle);
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIR_MOTION_SENSOR_GPIO_Pin */
  GPIO_InitStruct.Pin = PIR_MOTION_SENSOR_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PIR_MOTION_SENSOR_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin RST_Pin */
  GPIO_InitStruct.Pin = CS_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(0)
	{
		osSemaphoreRelease(Binary_SemHandle);
	}
}
void Task_action(const char *format, ...)
{
    char buffer[128]; // Adjust size as needed

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    const char newline[] = "\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)newline, strlen(newline), HAL_MAX_DELAY);
}
void initialize_rfid(void) {
	rfid_configure(&hspi1, CS_GPIO_Port, CS_Pin, RST_GPIO_Port, RST_Pin);
	rfid_self_test();
	rfid_init();
}
// Function to compare UID with valid list
bool is_valid_uid(uint8_t *uid) {
    for (int i = 0; i < NUM_VALID_UIDS; i++) {
        if (memcmp(uid, valid_uids[i], 5) == 0) {
            return true;
        }
    }
    return false;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task_UART_Telemetry */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_UART_Telemetry */
void Task_UART_Telemetry(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	char buffer[64];

	for (;;) {
		if (osMessageQueueGet(joystickQueueHandleHandle, &input, NULL, osWaitForever) == osOK) {
			snprintf(buffer, sizeof(buffer), "{\"x\":%d,\"y\":%d,\"dir\":%d}\r\n",
					input.x_val, input.y_val, input.direction);
			HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_Joystick */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Joystick */
void Task_Joystick(void *argument)
{
  /* USER CODE BEGIN Task_Joystick */
  /* Infinite loop */
	for (;;) {
		while (!adcConversionComplete);
		adcConversionComplete = 0;

		for(uint8_t i=0; i <hadc1.Init.NbrOfConversion; i++)
		  {
			  input.x_val = (uint16_t ) adcResultsDMA[0];
			  // Read VRy (Y)
			  input.y_val = (uint16_t ) adcResultsDMA[1];
		  }

		input.direction = DIR_NONE;
		if (input.x_val < ADC_CENTER - DEADZONE)
			input.direction = DIR_LEFT;
		else if (input.x_val > ADC_CENTER + DEADZONE)
			input.direction = DIR_RIGHT;
		else if (input.y_val < ADC_CENTER - DEADZONE)
			input.direction = DIR_UP;
		else if (input.y_val > ADC_CENTER + DEADZONE)
			input.direction = DIR_DOWN;
		else
			input.direction = DIR_CENTER;

		osMessageQueuePut(joystickQueueHandleHandle, &input, 0, 0);
		osDelay(50);
	}
  /* USER CODE END Task_Joystick */
}

/* USER CODE BEGIN Header_Task_RFID */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_RFID */
void Task_RFID(void *argument)
{
    uint8_t uid[5];
    for (;;) {
        if (rfid_is_new_card()) {
            rfid_status_t status = rfid_anticoll(uid);
            if (status == MI_OK) {
                char uid_str[11];
                snprintf(uid_str, sizeof(uid_str), "%02X%02X%02X%02X%02X",
                         uid[0], uid[1], uid[2], uid[3], uid[4]);

                char json_msg[128];

                if (is_valid_uid(uid)) {
                    // Access granted JSON
                    snprintf(json_msg, sizeof(json_msg),
                             "{\"rfid\": \"%s\", \"status\": \"unlocked\"}\r\n", uid_str);

                    HAL_UART_Transmit(&huart1, (uint8_t *)json_msg, strlen(json_msg), HAL_MAX_DELAY);

                    // Open servo (unlock)
                    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
                    for (int x = 500; x < 2500; x++) {
                        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, x);
                        HAL_Delay(1);
                    }

                    snprintf(json_msg, sizeof(json_msg), "{\"relock\": \"%s\"}\r\n", "5");
                    HAL_UART_Transmit(&huart1, (uint8_t *)json_msg, strlen(json_msg), HAL_MAX_DELAY);

                    // Close servo (lock)
                    for (int x = 2500; x > 500; x--) {
                        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, x);
                        HAL_Delay(1);
                    }

                    // Send relocked status
                    snprintf(json_msg, sizeof(json_msg),
                             "{\"status\": \"locked\"}\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *)json_msg, strlen(json_msg), HAL_MAX_DELAY);

                } else {
                    // Access denied JSON
                    snprintf(json_msg, sizeof(json_msg),
                             "{\"rfid\": \"%s\", \"status\": \"denied\"}\r\n", uid_str);
                    HAL_UART_Transmit(&huart1, (uint8_t *)json_msg, strlen(json_msg), HAL_MAX_DELAY);
                }
            } else {
                const char *err = "{\"error\": \"Failed to read UID\"}\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t *)err, strlen(err), HAL_MAX_DELAY);
            }
        }
        osDelay(70);
    }
}



/* USER CODE BEGIN Header_Task_Control */
/**
* @brief Function implementing the Task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Control */
void Task_Control(void *argument)
{
  /* USER CODE BEGIN Task_Control */
  /* Infinite loop */
	for (;;)
	{
		if (cmd_ready_flag)
		{
			cmd_ready_flag = 0;

			if (strncmp((char *)rx_buffer, "snake_game", 10) == 0)
			{
				osThreadResume(joystickTaskHandle);
				osThreadResume(uartTaskHandle);
				osThreadSuspend(rfidTaskHandle);
				const char *msg = "Snake mode activated\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
			}
			else if (strncmp((char *)rx_buffer, "door_lock ", 10) == 0)
			{
				osThreadResume(rfidTaskHandle);
				osThreadSuspend(joystickTaskHandle);
				osThreadSuspend(uartTaskHandle);
				const char *msg = "Doorlock mode activated\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
			}
			else if (strncmp((char *)rx_buffer, "dashboard ", 10) == 0)
			{
			    osThreadSuspend(rfidTaskHandle);
			    osThreadSuspend(joystickTaskHandle);
			    osThreadSuspend(uartTaskHandle);

			    const char *msg = "Dashboard mode activated\r\n";
			    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
			}

			// Restart UART DMA for next command
			HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
		}

		osDelay(10); // Optional small delay
	}
  /* USER CODE END Task_Control */
}

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
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcConversionComplete = 1;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)  // check which UART triggered
    {
    	cmd_ready_flag = 1; // Just set flag
    }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
