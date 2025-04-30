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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>   // for sprintf
#include <string.h>  // for strlen
#include <stdarg.h>

#include "ssd1306.h"
#include "ssd1306_fonts.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE BEGIN PV */

volatile uint8_t btn2_down = 0;       // 是否正在按住
volatile uint8_t btn2_released = 0;   // 是否剛剛放開（等待處理）
volatile uint32_t btn2_press_time = 0;

// Motor state
volatile uint8_t speed_level = 0; // 預設為0=停止
volatile uint8_t uart_override = 0; // 1=UART in control

// Button state
volatile uint8_t config_mode = 0; // 進入設定模式後為1
volatile uint8_t flag_btn1_pressed = 0;
volatile uint8_t flag_btn2_pressed = 0;
volatile uint32_t last_btn1_time = 0;
volatile uint32_t last_btn2_time = 0;
uint32_t press_time = 0;

// UART RX
uint8_t rx_data;

//Error Message
uint8_t oled_error_flag = 0;
uint32_t oled_error_time = 0;
char oled_msg[22] = "";  // 最多21字元 + 結尾
uint32_t oled_msg_time = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void update_oled_display(void);
void oled_message(const char* fmt, ...);  // <-- 加這行！


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_motor_speed(uint8_t mode)
{
    uint32_t duty = 0;
    char rgb_cmd = '0';  // 預設為呼吸燈

    switch(mode)
    {
        case 1: duty = 20; rgb_cmd = '1'; break;
        case 2: duty = 50; rgb_cmd = '2'; break;
        case 3: duty = 80; rgb_cmd = '3'; break;
        default: duty = 0; rgb_cmd = '0'; break;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

    // ✅ 如果不是設定模式，才送 RGB 指令給 Arduino
    if (!config_mode) {
        HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);
    }

    printf("Speed changed to: %d\r\n", mode);
}




void print_banner() {
    printf("=== TriSpeed MotorX v1.0 ===\r\n");
    printf("BTN1: PB13 - speed cycle\r\n");
    printf("BTN2: PB14 - short=reset, long=stop\r\n");
    printf("UART: 1/2/3/0=set, x=release override\r\n\r\n");
}

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // OLED 初始化
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // 啟動 PWM
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_data, 1);  // 啟動 UART 中斷接收
  print_banner();
  set_motor_speed(speed_level); // 初始為停止
  update_oled_display();

  static uint32_t btn2_release_debounce_time = 0;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  // 拉高 IN1，固定一個方向

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // ----- ⏳ 偵測 BTN2 放開 + 延遲判斷 (debounce)
	      if (btn2_down && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
	      {
	          // 按鈕剛放開，開始計時 debounce
	          if (btn2_release_debounce_time == 0)
	              btn2_release_debounce_time = HAL_GetTick();

	          // 放開後經過 30ms 確認沒彈跳才算真的釋放
	          if (HAL_GetTick() - btn2_release_debounce_time > 30)
	          {
	              btn2_down = 0;
	              btn2_released = 1;
	              btn2_release_debounce_time = 0; // 重設
	          }
	      }
	      else
	      {
	          // 還沒放開 or 仍按著，重設計時器
	          btn2_release_debounce_time = 0;
	      }

	      // ----- BTN1 處理
	      if (flag_btn1_pressed)
	      {
	          if (!config_mode)
	          {
	              speed_level++;
	              if (speed_level > 3) speed_level = 1;
	              set_motor_speed(speed_level);
	              printf("[BTN1] Speed -> %d (Normal Mode)\r\n",speed_level);
	          }
	          else
	          {
	        	  printf("[BTN1] Invalid: In setting mode. Use UART.\r\n");
	        	  oled_message("Error: Not allowed");

	              // ✅ 設定 OLED 錯誤提示
	              oled_error_flag = 1;
	              oled_error_time = HAL_GetTick();

	              // 加強這邊：強制清掉 flag，並忽略
	              flag_btn1_pressed = 0;
	              continue;

	          }

	          update_oled_display();
	          flag_btn1_pressed = 0;
	      }



	      // ----- BTN2 放開後的邏輯
	      if (btn2_released)
	      {
	          btn2_released = 0;
	          uint32_t press_duration = HAL_GetTick() - btn2_press_time;

	          if (press_duration < 1000)
	          {
	        	  speed_level = 1;
	        	      config_mode = 0;
	        	      printf("[BTN2] Short Press: Reset to weak speed. Exit setting mode.\r\n");

	        	      // ✅ 離開設定模式時傳送 'e'
	        	      char rgb_cmd = 'e';
	        	      HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);
	          }
	          else
	          {
	        	  speed_level = 0;
	        	     config_mode = 1;
	        	     printf("[BTN2] Long Press: Enter setting mode. Motor stopped.\r\n");

	        	     set_motor_speed(speed_level);  // 設定為停止
	        	     update_oled_display();

	        	     // ✅ 請傳送 's' 給 Arduino
	        	     char rgb_cmd = 's';
	        	     HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);

	          }

	          set_motor_speed(speed_level);
	          update_oled_display(); // ✅ OLED 更新

	      }

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_IN1_GPIO_Port, Motor_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_IN1_Pin */
  GPIO_InitStruct.Pin = Motor_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Switch1_Pin Switch2_Pin */
  GPIO_InitStruct.Pin = Switch1_Pin|Switch2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_13)
    {
        if (now - last_btn1_time > 150)
        {
            flag_btn1_pressed = 1;
            last_btn1_time = now;
        }
    }

    else if (GPIO_Pin == GPIO_PIN_14)
    {
        if (now - last_btn2_time > 50)
        {
            last_btn2_time = now;

            // 我們只會記錄按下時間，放開交給 main loop 判斷
            if (!btn2_down)
            {
                btn2_down = 1;
                btn2_press_time = now;
            }
        }
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t new_speed = speed_level;
        if (config_mode)
        {
            switch(rx_data)
            {
                case '1': new_speed = 1; break;
                case '2': new_speed = 2; break;
                case '3': new_speed = 3; break;
                case '0': new_speed = 0; break;
                default:
                	printf("[UART ] Invalid input: %c\r\n", rx_data);
                	oled_message("Invalid input: %c", rx_data);
                	break;

            }

            if (new_speed != speed_level) {
                speed_level = new_speed;
                set_motor_speed(speed_level);
                update_oled_display();

                // ✅ 加在這裡
                char rgb_cmd = rx_data; // '1', '2', '3', '0'
                HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);

                printf("[UART ] Speed -> %d (Setting Mode)\r\n", speed_level);
            }
        }
        else
        {
            printf("[UART ] Ignored: Not in setting mode. Long press BTN2 to enable.\r\n");
            oled_message("Not setting mode");
            update_oled_display();  // ✅ 新增這一行！
        }

        // 重新啟用 UART 接收中斷
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}


void update_oled_display(void)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("TriSpeed MotorX", Font_7x10, White);

    ssd1306_SetCursor(0, 16);
    if (config_mode)
        ssd1306_WriteString("Mode: Setting", Font_7x10, White);
    else
        ssd1306_WriteString("Mode: Normal", Font_7x10, White);

    ssd1306_SetCursor(0, 32);
    switch (speed_level)
    {
        case 1: ssd1306_WriteString("Speed: Weak", Font_7x10, White); break;
        case 2: ssd1306_WriteString("Speed: Mid", Font_7x10, White); break;
        case 3: ssd1306_WriteString("Speed: High", Font_7x10, White); break;
        default: ssd1306_WriteString("Speed: Stop", Font_7x10, White); break;
    }

    // ✅ 顯示警告訊息（維持2秒）
    ssd1306_SetCursor(0, 48);
        if (oled_msg[0] != '\0' && HAL_GetTick() - oled_msg_time < 2000) {
            ssd1306_WriteString(oled_msg, Font_7x10, White);
        } else {
            oled_msg[0] = '\0';  // 超過時間自動清除訊息
        }

    ssd1306_UpdateScreen();
}

void oled_message(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(oled_msg, sizeof(oled_msg), fmt, args);
    va_end(args);
    oled_msg_time = HAL_GetTick();
}


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 4 */

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
