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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GY85.h"
#include "ads1115.h"
#include "uart_print.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_TIMEOUT 100

// Bufory do przechowywania danych
char uartBuffer[256];

// I2C_HandleTypeDef hi2c1;
// UART_HandleTypeDef huart2;
GY85_HandleTypeDef hgy85;
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
HAL_StatusTypeDef I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint8_t DevAddress);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**I2C1 GPIO Configuration
   PB6    ------> I2C1_SCL
   PB7    ------> I2C1_SDA
*/

// Sprawdzanie czy urządzenie I2C jest dostępne
HAL_StatusTypeDef I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint8_t DevAddress)
{
  return HAL_I2C_IsDeviceReady(hi2c, DevAddress << 1, 3, I2C_TIMEOUT);
}

/**
 * @brief Wyświetlenie danych z sensorów przez UART
 */
static void print_data(void)
{
  char buffer[256];

  snprintf(buffer, sizeof(buffer),
           "\r\n--- GY-85 Sensor Data ---\r\n"
           "Accelerometer (m/s^2): X=%.2f, Y=%.2f, Z=%.2f\r\n"
           "Gyroscope (rad/s): X=%.2f, Y=%.2f, Z=%.2f\r\n"
           "Magnetometer (Gauss): X=%.2f, Y=%.2f, Z=%.2f\r\n"
           "Temperature: %.2f °C\r\n"
           "--------------------------\r\n",
           hgy85.accel.x, hgy85.accel.y, hgy85.accel.z,
           hgy85.gyro.x, hgy85.gyro.y, hgy85.gyro.z,
           hgy85.mag.x, hgy85.mag.y, hgy85.mag.z,
           hgy85.temperature);

  uart_print(buffer);
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

  char buffer[100];
  HAL_StatusTypeDef status;

  // Sprawdzenie czy UART działa
  uart_print("\r\n\r\n=============================\r\n");
  uart_print("Test UART - System uruchomiony\r\n");
  uart_print("=============================\r\n");

  // Sprawdzenie adresów I2C
  uart_print("Skanowanie urządzeń I2C...\r\n");
  uint8_t devices_found = 0;

  for (uint8_t addr = 1; addr < 128; addr++)
  {
    status = I2C_IsDeviceReady(&hi2c1, addr);
    if (status == HAL_OK)
    {
      sprintf(buffer, "Znaleziono urządzenie I2C pod adresem: 0x%02X\r\n", addr);
      uart_print(buffer);
      devices_found++;
    }
  }

  if (devices_found == 0)
  {
    uart_print("Nie znaleziono żadnych urządzeń I2C!\r\n");
  }
  else
  {
    sprintf(buffer, "Znaleziono urządzeń I2C: %d\r\n", devices_found);
    uart_print(buffer);
  }

  // Sprawdzenie czy ADS1115 jest dostępny
  uart_print("Sprawdzanie ADS1115...\r\n");
  status = I2C_IsDeviceReady(&hi2c1, ADS1115_ADDRESS_GND);
  if (status != HAL_OK)
  {
    uart_print("BŁĄD: ADS1115 nie został znaleziony pod adresem 0x48!\r\n");
    uart_print("Sprawdź połączenia lub spróbuj inny adres.\r\n");
  }
  else
  {
    uart_print("ADS1115 znaleziony! Inicjalizacja...\r\n");

    // Inicjalizacja ADS1115
    ADS1115_Init();
    uart_print("ADS1115 zainicjalizowany.\r\n");

    // Krótkie opóźnienie po inicjalizacji
    HAL_Delay(100);
  }

  // Informacja o rozpoczęciu pomiarów
  uart_print("Rozpoczynam odczyty z ADS1115 A0...\r\n");

  // GY85_Init(&hgy85, &hi2c1);
  GY85_Init(&hgy85, &hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Status LED (jeśli jest dostępny)
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Zakładając, że LED jest na PA5 (typowo dla większości płytek STM32)

    // Próba odczytu z ADS1115
    uart_print("Odczyt z ADS1115...\r\n");

    // Odczyt konfiguracji
    uint16_t config_reg = 0;
    status = ADS1115_ReadRegister(ADS1115_REG_CONFIG, &config_reg);
    if (status != HAL_OK)
    {
      uart_print("BŁĄD: Nie można odczytać rejestru konfiguracji!\r\n");
    }
    else
    {
      sprintf(buffer, "Rejestr CONFIG: 0x%04X\r\n", config_reg);
      uart_print(buffer);
    }

    // Odczyt z kanału A0
    int16_t raw_value = 0;
    float voltage = 0.0f;

    raw_value = ADS1115_ReadADC_A0();
    voltage = ADS1115_ConvertToVoltage(raw_value, ADS1115_PGA_2_048V);

    sprintf(buffer, "ADC Raw: %d, Voltage: %.3f V\r\n", raw_value, voltage);
    uart_print(buffer);

    // Opóźnienie przed kolejnym odczytem (1 sekunda)
    uart_print("Czekam 1 sekundę...\r\n\r\n");
    HAL_Delay(1000);

    // /* Odczyt danych z sensorów */
    // if (GY85_ReadAllSensors(&hgy85) == HAL_OK)
    // {
    //   /* Wyświetlenie danych */
    //   print_data();

    //   /* Zapalenie diody LED po udanym odczycie */
    //   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    // }
    // else
    // {
    //   /* Zgaszenie diody LED przy błędzie */
    //   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    // }

    // /* Opóźnienie pomiędzy odczytami */
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