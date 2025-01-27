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
#include "stdio.h"


// MPU6050 I2C address
#define MPU6050_ADDR 0xD0 // 7-bit address for MPU6050 (0x68 << 1 for write, 0x69 << 1 for read)



// Function Prototypes
void MPU6050_Init(void);
void MPU6050_Read_Accel(float* ax, float* ay, float* az);
void MPU6050_Read_Gyro(float* gx, float* gy, float* gz);
void MPU6050_Read_Temp(float* tp);
void periph_uart_send_tx_data(const char *tx_buff, uint16_t buffer_len);


static uint8_t tx_buffer_s[100u] = {""};



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../CustomDrivers/mpu6050_reg_map.h"
#include <string.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
	uint8_t buf[12];


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
  MX_I2C1_Init();

  // Initialize MPU6050
  MPU6050_Init();

  float ax, ay, az, tp;
  float gx, gy, gz;
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Read values
      MPU6050_Read_Accel(&ax, &ay, &az);
      MPU6050_Read_Gyro(&gx, &gy, &gz);
      MPU6050_Read_Temp(&tp);

      //Transmit Data (not working yet)
	  uint16_t buffer_len = 0;
	  char ax_string[100u] = "";
	  char ay_string[100u] = "";
	  char az_string[100u] = "";
	  char gx_string[100u] = "";
	  char gy_string[100u] = "";
	  char gz_string[100u] = "";
	  char tp_string[100u] = "";

	  int16_t  ax_disp = ax;
	  int16_t  ay_disp = ay;
	  int16_t  az_disp = az;
	  int16_t  gx_disp = gx;
	  int16_t  gy_disp = gy;
	  int16_t  gz_disp = gz;
	  int16_t  tp_disp = tp;


	  snprintf(ax_string, 100u, "Ax %d    ", ax_disp);
	  snprintf(ay_string, 100u, "Ay %d    ", ay_disp);
	  snprintf(az_string, 100u, "Az %d    ", az_disp);
	  snprintf(gx_string, 100u, "Gx %d    ", gx_disp);
	  snprintf(gy_string, 100u, "Gy %d    ", gy_disp);
	  snprintf(gz_string, 100u, "Gz %d    ", gz_disp);
	  snprintf(tp_string, 100u, "Tp %d\r\n", tp_disp);

	  buffer_len = strlen(ax_string);
	  periph_uart_send_tx_data(ax_string, buffer_len);
	  buffer_len = strlen(ay_string);
	  periph_uart_send_tx_data(ay_string, buffer_len);
	  buffer_len = strlen(az_string);
	  periph_uart_send_tx_data(az_string, buffer_len);

	  buffer_len = strlen(gx_string);
	  periph_uart_send_tx_data(gx_string, buffer_len);
	  buffer_len = strlen(gy_string);
	  periph_uart_send_tx_data(gy_string, buffer_len);
	  buffer_len = strlen(gz_string);
	  periph_uart_send_tx_data(gz_string, buffer_len);

	  buffer_len = strlen(tp_string);
	  periph_uart_send_tx_data(tp_string, buffer_len);


	  //Demonstrate message transmit of basic string
	  //strcpy((char*)buf, "Hello!\r\n");
	  //HAL_UART_Transmit(&huart2,buf,strlen((char*)buf),HAL_MAX_DELAY);


      HAL_Delay(500); // Delay for readability
  }
  /* USER CODE END 3 */
}


void periph_uart_send_tx_data(const char *tx_buff, uint16_t buffer_len)
{
	strcpy((char*)tx_buffer_s, tx_buff);
	HAL_UART_Transmit(&huart2, tx_buffer_s, buffer_len, HAL_MAX_DELAY);

}

void MPU6050_Init(void)
{
	uint8_t check,data;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 1, &check, 1 , 1000);
	if (check == 104)
	{
		//Take sensor out of sleep mode
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, 1000);

		//Set data rate to 1KHz
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_SMPRT_DIV, 1, &data, 1, 1000);

		//Set full range scale for accelerometer and gyroscope
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1, 1000);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 1, &data, 1, 1000);
	}
}

void MPU6050_Read_Accel(float* ax, float* ay, float* az)
{
    uint8_t data[6];
    uint8_t ax_raw, ay_raw, az_raw;

    // Read 6 bytes of accelerometer data to get H and L value of x,y, and z
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

    // Combine high and low bytes to get definitive value
    ax_raw = (int16_t)((data[0] << 8) | data[1]);
    ay_raw = (int16_t)((data[2] << 8) | data[3]);
    az_raw = (int16_t)((data[4] << 8) | data[5]);

	*ax = ax_raw/16384.0;
	*ay = ay_raw/16384.0;
	*az = az_raw/16384.0;
}

void MPU6050_Read_Gyro(float* gx, float* gy, float* gz)
{
    uint8_t data[6];
    uint8_t gx_raw, gy_raw, gz_raw;

    // Read 6 bytes of gyroscope data to get H and L value of x,y, and z
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

    // Combine high and low bytes to get definitive value
    gx_raw = (int16_t)(data[0] << 8 | data[1]);
    gy_raw = (int16_t)(data[2] << 8 | data[3]);
    gz_raw = (int16_t)(data[4] << 8 | data[5]);

    *gx = gx_raw / 131.0;
    *gy = gy_raw / 131.0;
    *gz = gz_raw / 131.0;
}

void MPU6050_Read_Temp(float* tp)
{
    int8_t data[2];
    int16_t tp_raw;

    // Read 2 bytes of temperature data to get H and L values
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_TEMP_OUT_H, 1, data, 2, HAL_MAX_DELAY);

    // Combine high and low bytes
    tp_raw = (int16_t)((data[0] << 8) | data[1]);
    *tp = (tp_raw /340.0 + 36.53);
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
  hi2c1.Init.ClockSpeed = 100000;
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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
