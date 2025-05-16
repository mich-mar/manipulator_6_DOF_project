#ifndef ADS1115_H
#define ADS1115_H

#include "stm32f4xx_hal.h" // lub inny, zależnie od Twojego MCU
#include "usbd_cdc_if.h"
#include "i2c.h"
#include <stdint.h>
#include <stdbool.h>

// Domyślny adres I2C (ADDR do GND)
#define ADS1115_ADDRESS_GND 0x48
#define ADS1115_ADDRESS_VDD 0x49
#define ADS1115_ADDRESS_SDA 0x4A
#define ADS1115_ADDRESS_SCL 0x4B

// Używane adressy
static const uint8_t ADS1115_ADDRESS_0_3 = ADS1115_ADDRESS_GND;
static const uint8_t ADS1115_ADDRESS_4_5 = ADS1115_ADDRESS_VDD;

// Rejestry
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01
#define ADS1115_REG_LO_THRESH 0x02
#define ADS1115_REG_HI_THRESH 0x03

// Bity w rejestrze konfiguracji
#define ADS1115_OS_SINGLE 0x8000  // Start single conversion
#define ADS1115_OS_BUSY 0x0000    // Conversion in progress
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
#define ADS1115_COMP_QUE_AFTER_ONE 0x0000  // Assert after one conversion
#define ADS1115_COMP_QUE_AFTER_TWO 0x0001  // Assert after two conversions
#define ADS1115_COMP_QUE_AFTER_FOUR 0x0002 // Assert after four conversions
#define ADS1115_COMP_QUE_DISABLE 0x0003    // Disable comparator and set ALERT/RDY pin to high-impedance

// Timeout dla konwersji [ms]
#define ADS1115_CONVERSION_TIMEOUT 200

// Status błędów ADS1115
typedef enum
{
    ADS1115_OK = 0,        // Brak błędów
    ADS1115_ERROR_I2C,     // Błąd komunikacji I2C
    ADS1115_ERROR_TIMEOUT, // Timeout konwersji
    ADS1115_ERROR_INIT,    // Błąd inicjalizacji
    ADS1115_ERROR_PARAM    // Błędny parametr
} ADS1115_Status;

// Struktura debugowania
typedef struct
{
    uint32_t total_reads;      // Całkowita liczba prób odczytu
    uint32_t successful_reads; // Udane odczyty
    uint32_t i2c_errors;       // Liczba błędów I2C
    uint32_t timeouts;         // Liczba timeoutów
    uint32_t last_error_code;  // Kod ostatniego błędu
    uint32_t last_config;      // Ostatnia konfiguracja
    int16_t last_raw_value;    // Ostatnia surowa wartość
    float last_voltage;        // Ostatnie obliczone napięcie
    bool device_detected;      // Czy urządzenie wykryto na magistrali
} ADS1115_Debug_t;

typedef struct
{
    int16_t A0_adc_raw;
    int16_t A1_adc_raw;
    int16_t A2_adc_raw;
    int16_t A3_adc_raw;
    int16_t A4_adc_raw;
    int16_t A5_adc_raw;

    float A0_adc;
    float A1_adc;
    float A2_adc;
    float A3_adc;
    float A4_adc;
    float A5_adc;
} ADS1115_Readings;

extern ADS1115_Debug_t ADS1115_Debug;

// Funkcje inicjalizacji i ogólne
ADS1115_Status ADS1115_Init(uint8_t *device_adress);
int16_t ADS1115_Read(uint8_t *device_adress, ADS1115_Status *status);
bool ADS1115_IsDeviceConnected(uint8_t *device_adress);

// Funkcje odczytu z konkretnych kanałów
int16_t ADS1115_ReadADC_SingleEnded(uint8_t *device_adress, uint8_t channel, ADS1115_Status *status);
int16_t ADS1115_ReadADC_A0(uint8_t *device_adress, ADS1115_Status *status);
int16_t ADS1115_ReadADC_A1(uint8_t *device_adress, ADS1115_Status *status);
int16_t ADS1115_ReadADC_A2(uint8_t *device_adress, ADS1115_Status *status);
void ADS1115_ReadAllValues(ADS1115_Readings *ADCdata);
float ADS1115_ConvertToVoltage(int16_t raw_adc, uint16_t pga);


// Funkcje konfiguracji
HAL_StatusTypeDef ADS1115_StartConversion(uint8_t *device_adress, uint16_t config_reg);
HAL_StatusTypeDef ADS1115_WriteRegister(uint8_t *device_adress, uint8_t reg, uint16_t value);
HAL_StatusTypeDef ADS1115_ReadRegister(uint8_t *device_adress, uint8_t reg, uint16_t *value);
HAL_StatusTypeDef ADS1115_IsConversionReady(uint8_t *device_adress, uint8_t *ready);

// Funkcje debugowania
void ADS1115_PrintDebugInfo(void);
const char *ADS1115_GetErrorString(ADS1115_Status status);
void ADS1115_Reset_Debug_Counters(void);

#endif // ADS1115_H