#ifndef ADS1115_H
#define ADS1115_H

#include <stdint.h>
#include "stm32l4xx_hal.h" // lub inny, zależnie od Twojego MCU
#include "usart.h"
#include "i2c.h"

// Domyślny adres I2C (ADDR do GND)
#define ADS1115_ADDRESS_GND 0x48
#define ADS1115_ADDRESS_VDD 0x49
#define ADS1115_ADDRESS_SDA 0x4A
#define ADS1115_ADDRESS_SCL 0x4B

// Rejestry
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01
#define ADS1115_REG_LO_THRESH 0x02
#define ADS1115_REG_HI_THRESH 0x03

// Bity w rejestrze konfiguracji
#define ADS1115_OS_SINGLE 0x8000 // Start single conversion
#define ADS1115_OS_BUSY 0x0000   // Conversion in progress
#define ADS1115_OS_NOTBUSY 0x8000 // Device is not performing a conversion

// MUX - wybór kanału/wejścia różnicowego
#define ADS1115_MUX_DIFF_0_1 0x0000
#define ADS1115_MUX_DIFF_0_3 0x1000
#define ADS1115_MUX_DIFF_1_3 0x2000
#define ADS1115_MUX_DIFF_2_3 0x3000
#define ADS1115_MUX_SINGLE_0 0x4000
#define ADS1115_MUX_SINGLE_1 0x5000
#define ADS1115_MUX_SINGLE_2 0x6000
#define ADS1115_MUX_SINGLE_3 0x7000

// PGA - wzmocnienie
#define ADS1115_PGA_6_144V 0x0000 // +/-6.144V
#define ADS1115_PGA_4_096V 0x0200 // +/-4.096V
#define ADS1115_PGA_2_048V 0x0400 // +/-2.048V (domyślne)
#define ADS1115_PGA_1_024V 0x0600 // +/-1.024V
#define ADS1115_PGA_0_512V 0x0800 // +/-0.512V
#define ADS1115_PGA_0_256V 0x0A00 // +/-0.256V

// Tryby pracy
#define ADS1115_MODE_CONTINUOUS 0x0000
#define ADS1115_MODE_SINGLESHOT 0x0100

// Szybkość próbkowania (samples per second)
#define ADS1115_DR_8SPS 0x0000
#define ADS1115_DR_16SPS 0x0020
#define ADS1115_DR_32SPS 0x0040
#define ADS1115_DR_64SPS 0x0060
#define ADS1115_DR_128SPS 0x0080 // domyślne
#define ADS1115_DR_250SPS 0x00A0
#define ADS1115_DR_475SPS 0x00C0
#define ADS1115_DR_860SPS 0x00E0

// Komparator (tryby pracy)
#define ADS1115_COMP_MODE_TRADITIONAL 0x0000 // Traditional comparator
#define ADS1115_COMP_MODE_WINDOW 0x0010      // Window comparator

// Komparator (polaryzacja)
#define ADS1115_COMP_POL_ACTIVE_LOW 0x0000  // Active low
#define ADS1115_COMP_POL_ACTIVE_HIGH 0x0008 // Active high

// Komparator (tryb zatrzasku)
#define ADS1115_COMP_LAT_NON_LATCHING 0x0000 // Non-latching
#define ADS1115_COMP_LAT_LATCHING 0x0004     // Latching

// Komparator (liczba konwersji)
#define ADS1115_COMP_QUE_AFTER_ONE 0x0000    // Assert after one conversion
#define ADS1115_COMP_QUE_AFTER_TWO 0x0001    // Assert after two conversions
#define ADS1115_COMP_QUE_AFTER_FOUR 0x0002   // Assert after four conversions
#define ADS1115_COMP_QUE_DISABLE 0x0003      // Disable comparator and set ALERT/RDY pin to high-impedance

// Domyślny adres I2C
#define ADS1115_I2C_ADDR ADS1115_ADDRESS_GND

// Funkcje inicjalizacji i ogólne
void ADS1115_Init(void);
int16_t ADS1115_Read(void);

// Funkcje odczytu z konkretnych kanałów
int16_t ADS1115_ReadADC_SingleEnded(uint8_t channel);
int16_t ADS1115_ReadADC_A0(void);
float ADS1115_ConvertToVoltage(int16_t raw_adc, uint16_t pga);

// Funkcje konfiguracji
HAL_StatusTypeDef ADS1115_StartConversion(uint16_t config);
HAL_StatusTypeDef ADS1115_WriteRegister(uint8_t reg, uint16_t value);
HAL_StatusTypeDef ADS1115_ReadRegister(uint8_t reg, uint16_t *value);
HAL_StatusTypeDef ADS1115_IsConversionReady(uint8_t *ready);

#endif // ADS1115_H