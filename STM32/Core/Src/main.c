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

#include <stdio.h>   // For sprintf
#include <string.h>  // For strlen
#include <stdarg.h>

#include "ssd1306.h" // From GitHub https://github.com/afiskon/stm32-ssd1306
#include "ssd1306_fonts.h" // From GitHub https://github.com/afiskon/stm32-ssd1306

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

/*上面註解每次Generate Code都會被清掉移來這裡
 * 開啟I2C, Timer3, UART1
 * 另外UART2是建專案就開起的*/

/* USER CODE BEGIN PV */

volatile uint8_t btn2_down = 0;       //按鍵是否正在被按下（由中斷設為 1，放開時重設為 0）
volatile uint8_t btn2_released = 0;   //是否已經放開按鍵（偵測到釋放後設為 1，處理完後清除）
volatile uint32_t btn2_press_time = 0;//BTN2 按下當下的時間（用於計算按住時間以區分長按／短按）
volatile uint32_t btn2_release_debounce_time = 0;//放開時的防彈跳時間（確保放開維持一段時間才觸發事件）

// Motor state
volatile uint8_t speed_level = 0; //當前馬達轉速等級（0=停止, 1~3 對應弱/中/強）
volatile uint8_t uart_override = 0; //是否由 UART 控制速度（1=UART 控制, 0=按鈕控制）

// Button state
volatile uint8_t config_mode = 0; //是否進入設定模式（1=Config 模式, 只能用 UART 控制）
volatile uint8_t flag_btn1_pressed = 0; //BTN1 中斷觸發後設為 1，主程式內會依此切換速度，處理完後清除
volatile uint8_t flag_btn2_pressed = 0; //保留：目前未使用，可作為 BTN2 事件觸發用
volatile uint32_t last_btn1_time = 0; //BTN1 上次觸發時間（防彈跳用）
volatile uint32_t last_btn2_time = 0; //BTN2 上次觸發時間（防彈跳用）
uint32_t press_time = 0; //非 volatile，暫存按下時間（可與 BTN2 共用）

// UART RX
uint8_t rx_data; //USART2 單位元接收暫存區（配合 UART 中斷）

//Error Message
char oled_msg[22] = "";  //OLED 顯示訊息（最多 21 字元 + 1 個結尾 '\0'）
uint32_t oled_msg_time = 0; //訊息顯示起始時間（2 秒後自動清除）


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* --------------------------------------------------------------------------
 * 自己添加的私有函式宣告區（Private Function Prototypes）
 * 用於 OLED 顯示、錯誤訊息更新等自訂功能
 * -------------------------------------------------------------------------- */

void update_oled_display(void); //更新 OLED 顯示內容（狀態 / 轉速）
void oled_message(const char* fmt, ...);  //顯示錯誤訊息（支援格式化輸入


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*設定馬達轉速並根據狀態傳送對應 RGB 指令給 Arduino
 * mode 馬達速度等級（0=停止, 1=弱速, 2=中速, 3=高速）
 * 對應 PWM duty 為 0%, 20%, 50%, 80%
 */

void set_motor_speed(uint8_t mode)
{
    uint32_t duty = 0; //用於設定 PWM 輸出佔空比
    char rgb_cmd = '0';  //要透過 UART 傳送給 Arduino 的 RGB 控制（預設為 0：呼吸燈）
    //根據 mode 設定 PWM duty 與對應 RGB 指令
    switch(mode)
    {
        case 1: duty = 20; rgb_cmd = '1'; break; //弱速：20%，綠色系燈號
        case 2: duty = 50; rgb_cmd = '2'; break; //中速：50%，橙黃色系燈號
        case 3: duty = 80; rgb_cmd = '3'; break; //高速：80%，紅色系燈號
        default: duty = 0; rgb_cmd = '0'; break; //停止：0%，暖色系呼吸燈（待機）
    }
    //更新PWM的duty,控制馬達轉速
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

    //僅在非設定模式下，主動透過 UART 傳送燈號控制指令給 Arduino（若為設定模式，應由使用者手動下指令控制燈色）
    if (!config_mode) {
        HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);
    }

    //將變更結果印出到 UART2，可透過 PC 端 terminal 觀察（如 TeraTerm、PuTTY、RealTerm）
    printf("Speed changed to: %d\r\n", mode);
}



/* 開機時於 UART2 顯示控制說明
 * 包含：按鈕功能說明、UART 可輸入的控制指令格式等。
 * 可搭配 PC 端 terminal（TeraTerm / PuTTY / RealTerm）查看。
 */
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

  //OLED 初始化與清除畫面
  ssd1306_Init(); //初始化 SSD1306 模組
  ssd1306_Fill(Black); //清除畫面
  ssd1306_UpdateScreen(); //將內容更新至 OLED

  /* USER CODE BEGIN 2 */

  //啟動外設功能
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  //啟動馬達 PWM 控制
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_data, 1);   //啟動 UART2 非阻塞接收（PC端指令）

  //預設值：上電後為 Normal Mode（config_mode = 0）且馬達停止（speed_level = 0）
  config_mode = 0;
  speed_level = 0;  //speed_level = 0 對應「停止 + 呼吸燈」

  //---------- 通知 Arduino 離開設定模式 ----------
  //原本問題：若未主動通知，Arduino 仍保留在設定模式，導致顯示錯誤燈號（如閃爍白燈或藍紫燈）
  //解法：主動送出 'e'，代表退出設定模式（exit）
  char rgb_cmd_exit = 'e';
  HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd_exit, 1, HAL_MAX_DELAY);

  HAL_Delay(100);  //延遲一小段時間，確保 Arduino Serial 有足夠時間完成初始化

  //---------- 傳送預設燈號狀態 ----------
  //speed_level = 0 對應預設燈號（呼吸燈），需傳送 '0' 給 Arduino
  char rgb_cmd_mode = '0';
  HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd_mode, 1, HAL_MAX_DELAY);

  //系統初始狀態處理
  print_banner(); //列印操作說明，建議搭配 PC 端 terminal 使用（TeraTerm / PuTTY / RealTerm）
  set_motor_speed(speed_level); //設定初始速度（預設為 0：停止）
  update_oled_display(); //顯示初始狀態至 OLED

  //BTN2 釋放 debounce 時間初始化（延遲觸發防彈跳）
  static uint32_t btn2_release_debounce_time = 0;

  //拉高馬達 IN1 腳位(接在L298N模組)，固定馬達旋轉方向(只控制一個方向)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	  //---------- BTN1 被按下的處理邏輯 ----------
	      if (flag_btn1_pressed)
	      {
	          if (!config_mode)
	          {

	        	  //Normal 模式中循環切換速度（1 → 2 → 3 → 1）
	              speed_level++;
	              if (speed_level > 3) speed_level = 1;
	              set_motor_speed(speed_level);
	              printf("[BTN1] Speed -> %d (Normal Mode)\r\n",speed_level);
	          }
	          else
	          {

	        	  //設定模式中不允許按 BTN1 切換速度
	        	  printf("[BTN1] Invalid: In setting mode. Use UART.\r\n");
	        	  oled_message("Error: Not allowed");

	              flag_btn1_pressed = 0; //清除事件旗標

	              continue;//跳過更新，重新跑 while-loop

	          }

	          update_oled_display(); //顯示當前速度
	          flag_btn1_pressed = 0; //清除事件旗標
	      }

	      //---------- BTN2 輪詢（已停用，備用作為中斷替代方案） ----------
	      /*
	      if (!btn2_down && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
	      {
	          btn2_down = 1;
	          btn2_press_time = HAL_GetTick();
	          btn2_release_debounce_time = 0; //初始化釋放計時器
	      }

	      //持續輪詢偵測放開動作（需維持高電位超過 100ms 才視為放開）
		  if (btn2_down && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
		  {
			  //第一次偵測放開 → 開始計時
			  if (btn2_release_debounce_time == 0)
				  btn2_release_debounce_time = HAL_GetTick();

			  //放開狀態維持超過 100ms → 視為正式放開
			  if (HAL_GetTick() - btn2_release_debounce_time > 100)
			  {
				  btn2_down = 0;
				  btn2_released = 1;
				  btn2_release_debounce_time = 0; //重設

				  uint32_t press_duration = HAL_GetTick() - btn2_press_time;

				  if (press_duration < 1000)
				  {

					  //短按處理：重設為弱速 + 離開設定模式
					  speed_level = 1;
					  config_mode = 0;
					  printf("[BTN2] Short Press: Reset to weak speed. Exit setting mode.\r\n");

					  //通知 Arduino 離開設定模式（'e'）
					  char rgb_cmd = 'e';
					  HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);
				  }
				  else
				  {

					  //長按處理：進入設定模式 + 馬達停止
					 speed_level = 0;
					 config_mode = 1;
					 printf("[BTN2] Long Press: Enter setting mode. Motor stopped.\r\n");

					 set_motor_speed(speed_level);  //停止馬達
					 update_oled_display(); //顯示 "Mode: Setting"

					 //通知 Arduino 進入設定模式（'s'）
					 char rgb_cmd = 's';
					 HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);

				  }

				  //最後統一更新顯示與速度
				  set_motor_speed(speed_level);
				  update_oled_display();

			  }
		  }

		  else
		  {
			  //若尚未穩定放開（bounce or 持續按壓）→ 重設釋放計時器
			  btn2_release_debounce_time = 0;
		  } */

	      //---------- BTN2 放開偵測（中斷版本用） ----------
	      if (btn2_down && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
	      {
	    	  //第一次偵測放開 → 開始計時
	    	  if (btn2_release_debounce_time == 0)
	              btn2_release_debounce_time = HAL_GetTick();

	          //放開狀態維持超過 100ms → 視為正式放開
	          if (HAL_GetTick() - btn2_release_debounce_time > 100)
	          {
	              btn2_down = 0;
	              btn2_released = 1;
	              printf("[EXTI] BTN2 released after %lu ms\r\n", HAL_GetTick() - btn2_press_time);
	              btn2_release_debounce_time = 0; //重設
	          }
	      }

		  else
		  {
			  //仍然按著 or 彈跳中 → 放開計時重置
			  btn2_release_debounce_time = 0;
		  }

	      //---------- BTN2 放開後的處理邏輯 ----------
	      if (btn2_released)
	      {

	    	  btn2_released = 0;

	          uint32_t press_duration = HAL_GetTick() - btn2_press_time;

	          if (press_duration < 1000)
	          {

	        	  //短按 → 重設為弱速、離開設定模式
	        	  speed_level = 1;
				  config_mode = 0;
				  printf("[BTN2] Short Press: Reset to weak speed. Exit setting mode.\r\n");

				  //通知 Arduino 離開設定模式（'e'）
				  char rgb_cmd = 'e';
				  HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);
	          }
	          else
	          {

	        	 //長按 → 停止馬達並進入設定模式
	        	 speed_level = 0;
				 config_mode = 1;
				 printf("[BTN2] Long Press: Enter setting mode. Motor stopped.\r\n");

				 set_motor_speed(speed_level);  //停止馬達
				 update_oled_display(); //顯示 "Mode: Setting"

				 //通知 Arduino 進入設定模式（'s'）
				 char rgb_cmd = 's';
				 HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);

	          }

	          //統一更新顯示與馬達狀態
	          set_motor_speed(speed_level);
	          update_oled_display();

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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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

/* 外部中斷回呼函式，用來處理 BTN1（PB13）與 BTN2（PB14）按下事件
 *
 * 僅在中斷觸發時紀錄按下時間或設定事件旗標，避免在中斷中執行繁重邏輯。
 * BTN1：速度切換（設定 flag）
 * BTN2：判斷長按/短按（記錄按下時間，放開由主迴圈偵測）
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();  //取得目前時間（單位：ms）

    //---------- BTN1（PB13）按下事件 ----------
    if (GPIO_Pin == GPIO_PIN_13)
    {

    	if(config_mode) //設定模式下，直接略過 BTN1 中斷處理
    		return;

    	//若距離上次觸發已超過 150ms → 防彈跳
    	if (now - last_btn1_time > 150)  //防彈跳間隔 150ms
        {

    		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET){

    			flag_btn1_pressed = 1; //設定旗標，主程式會依此處理速度切換

    			last_btn1_time = now; //更新上次觸發時間

    		}
        }
    }

    //---------- BTN2（PB14）按下事件 ----------
    //註：曾懷疑 speed_level = 3 自動跳回 1 是因馬達 EMI 誤觸中斷所致
    else if (GPIO_Pin == GPIO_PIN_14)
    {
    	 //若距離上次觸發已超過 150ms → 防彈跳
    	if (now - last_btn2_time > 150)
        {
            last_btn2_time = now;

            //確保不是連續觸發，並再次確認當下電平為低（按下狀態）
			if (!btn2_down && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)  //加強判斷
            {
            	btn2_down = 1; //標記為正在按住狀態
                btn2_press_time = now; //記錄按下時間（用於判斷長按 or 短按）
                btn2_release_debounce_time = 0; //初始化放開 debounce 計時
                printf("[EXTI] BTN2 pressed at %lu ms\r\n", now);
            }
        }
    }
}


/* 接收完成中斷回呼函式
 *
 * 當 USART2 接收到 1 個 byte 資料後觸發。僅在進入 Config Mode（設定模式）時允許透過 UART 改變馬達轉速。
 * 否則會拒絕並顯示錯誤訊息。
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//確認此次中斷是來自 USART2（從PC端Terminal傳來的資料）
	if (huart->Instance == USART2)
    {
        uint8_t new_speed = speed_level; //預設為目前速度，後續判斷是否需要變更

        if (config_mode) //僅在設定模式下允許 UART 控速
        {
            switch(rx_data)
            {
                case '1': new_speed = 1; break;
                case '2': new_speed = 2; break;
                case '3': new_speed = 3; break;
                case '0': new_speed = 0; break;

                //非法輸入：顯示錯誤訊息至 Terminal + OLED
                default:
                	printf("[UART ] Invalid input: %c\r\n", rx_data);
                	oled_message("Invalid input: %c", rx_data);
                	break;

            }


            //若有變更速度，則更新 PWM、OLED 與 Arduino RGB
            if (new_speed != speed_level) {

            	speed_level = new_speed; //更新馬達速度

                set_motor_speed(speed_level); //變更 PWM 輸出

                update_oled_display(); //OLED 顯示當前狀態

                //將原始輸入字元透過 UART 傳送給 Arduino，控制 RGB 燈色
                char rgb_cmd = rx_data; // '1', '2', '3', '0'

                HAL_UART_Transmit(&huart1, (uint8_t *)&rgb_cmd, 1, HAL_MAX_DELAY);

                printf("[UART ] Speed -> %d (Setting Mode)\r\n", speed_level);
            }
        }

        else //非設定模式 → 拒絕 UART 控制
        {
            printf("[UART ] Ignored: Not in setting mode. Long press BTN2 to enable.\r\n");

            oled_message("Not setting mode"); //顯示錯誤訊息至 OLED

            update_oled_display();  //刷新 OLED 畫面（有錯誤提示）
        }

        //重新啟用 UART 接收中斷（非一次性，要持續接收）
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

/* 更新 OLED 顯示內容
 *
 * 顯示專案標題、當前模式（Normal / Setting）、速度狀態（Weak / Mid / High / Stop），
 * 並在有錯誤訊息時顯示提示，超過 2 秒自動清除。
 */

void update_oled_display(void)
{
    ssd1306_Fill(Black); //清空螢幕（塗滿黑色背景）

    //顯示標題：TriSpeed MotorX
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("TriSpeed MotorX", Font_7x10, White);

    //顯示目前模式（設定模式或一般模式）
    ssd1306_SetCursor(0, 16);
    if (config_mode)
        ssd1306_WriteString("Mode: Setting", Font_7x10, White); //設定模式
    else
        ssd1306_WriteString("Mode: Normal", Font_7x10, White); //設定模式

    //顯示目前速度狀態
    ssd1306_SetCursor(0, 32);
    switch (speed_level)
    {
    	case 0: ssd1306_WriteString("Speed: Stop", Font_7x10, White); break;
        case 1: ssd1306_WriteString("Speed: Weak", Font_7x10, White); break;
        case 2: ssd1306_WriteString("Speed: Mid", Font_7x10, White); break;
        case 3: ssd1306_WriteString("Speed: High", Font_7x10, White); break;
    }

    //顯示錯誤訊息（如有），最多顯示 2 秒
    ssd1306_SetCursor(0, 48);
        if (oled_msg[0] != '\0' && HAL_GetTick() - oled_msg_time < 2000) {
            ssd1306_WriteString(oled_msg, Font_7x10, White); //顯示訊息
        } else {
            oled_msg[0] = '\0';  //時間到 → 自動清除內容
        }

    ssd1306_UpdateScreen(); //最後統一更新畫面（送出到 OLED）
}

/*顯示格式化錯誤訊息至 OLED，並記錄出現時間（維持顯示 2 秒）
*
* 此函式支援 printf-style 格式化文字，例如：
* oled_message("Invalid input: %c", rx_data);
*
* 實際顯示與時間控制交由 update_oled_display() 處理。
*
* fmt 欲顯示的格式化字串（支援 %d、%c、%s 等格式）
*/

void oled_message(const char* fmt, ...) {
    va_list args; //宣告可變參數清單
    va_start(args, fmt); //初始化參數讀取（從 fmt 開始）
    vsnprintf(oled_msg, sizeof(oled_msg), fmt, args); //將格式化後的字串寫入 oled_msg[]
    va_end(args); //結束參數讀取
    oled_msg_time = HAL_GetTick(); //記錄目前時間，用於後續計算是否超過顯示時間
}


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch) //GCC 編譯器預設的 printf 輸出函式
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f) //其他編譯器（如 KEIL）預設用 fputc
#endif

//將 printf 的輸出字元透過 UART2 傳送出去
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY); //實際傳送字元給電腦端
    return ch; //回傳該字元，讓 printf 繼續下一個字元
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
