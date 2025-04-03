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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_adresses.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Funkcja do odczytu rejestru z czujnika
uint8_t read_register(uint8_t reg)
{
  uint8_t data = 0;
  HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADRESS << 1, reg, 1, &data, 1, HAL_MAX_DELAY);
  return data;
}

// Funkcja do zapisu do rejestru w czujniku
void write_register(uint8_t reg, uint8_t value)
{
  HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADRESS << 1, reg, 1, &value, 1, HAL_MAX_DELAY);
}

// Inicjalizacja czujnika
void sensor_init()
{
  uint8_t who_am_i = read_register(WHO_AM_I);
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "WHO_AM_I: 0x%02X\r\n", who_am_i);
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
  if (who_am_i != DEVICE_ID)
  {
    // Błąd, nie znaleziono urządzenia
    while (1)
      ;
  }

  // Inicjalizacja czujnika (np. ustawienie trybu pracy)
  write_register(PWR_MGMT_1, 0x01); // Ustawienie trybu pracy (włączony czujnik)
  HAL_Delay(100);

  // Inicjalizacja magnetometru
  write_register(MAG_CTRL_REG1, 0x10); // Włączenie magnetometru (dla MPU9250)
  HAL_Delay(100);
}

// Odczyt wartości z akcelerometru
void read_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
  uint8_t data[6];
  HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADRESS << 1, ACCEL_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

  *accel_x = (int16_t)((data[0] << 8) | data[1]);
  *accel_y = (int16_t)((data[2] << 8) | data[3]);
  *accel_z = (int16_t)((data[4] << 8) | data[5]);
}

// Odczyt wartości z żyroskopu
void read_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
  uint8_t data[6];
  HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADRESS << 1, GYRO_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

  *gyro_x = (int16_t)((data[0] << 8) | data[1]);
  *gyro_y = (int16_t)((data[2] << 8) | data[3]);
  *gyro_z = (int16_t)((data[4] << 8) | data[5]);
}

// Odczyt temperatury
int16_t read_temp()
{
  uint8_t data[2];
  HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADRESS << 1, TEMP_OUT_H, 1, data, 2, HAL_MAX_DELAY);

  return (int16_t)((data[0] << 8) | data[1]);
}

// Odczyt wartości z magnetometru
void read_mag(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z)
{
  uint8_t data[6];
  HAL_I2C_Mem_Read(&hi2c1, DEVICE_ADRESS << 1, MAG_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

  *mag_x = (int16_t)((data[0] << 8) | data[1]);
  *mag_y = (int16_t)((data[2] << 8) | data[3]);
  *mag_z = (int16_t)((data[4] << 8) | data[5]);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Inicjalizacja czujnika
  sensor_init();

  // Zmienna do przechowywania danych
  int16_t accel_x, accel_y, accel_z;
  int16_t gyro_x, gyro_y, gyro_z;
  int16_t temp;
  int16_t mag_x, mag_y, mag_z;

  double accel_x_g, accel_y_g, accel_z_g;

  double gyro_x_dps, gyro_y_dps, gyro_z_dps;

  double temperature_c;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Odczyt danych z IMU
    read_accel(&accel_x, &accel_y, &accel_z);
    read_gyro(&gyro_x, &gyro_y, &gyro_z);
    temp = read_temp();
    read_mag(&mag_x, &mag_y, &mag_z);

    // Wysyłanie danych przez USART (na przykład w formacie ASCII)
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "acc_x:%d, acc_y:%d, acc_z:%d | gyr_x:%d, gyr_y:%d, gyr_z:%d | temp_c:%d\n",
             accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    // Opóźnienie między kolejnymi odczytami
    // HAL_Delay(500);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

#ifdef USE_FULL_ASSERT
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
