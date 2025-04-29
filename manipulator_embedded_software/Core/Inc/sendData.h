#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include "main.h" // lub bezpośrednie #include potrzebnych plików np. "i2c.h", "adc.h"
#include "ads1115.h"  // deklaracje ADS1115_ReadAllValues()
#include "GY85.h"     // deklaracje GY85_ReadAllSensors()
#include "string.h"   // dla memset
#include "stdio.h"    // dla sprintf
#include "sendUSB.h"
#include "stm32f4xx_hal.h" // albo Twoje HAL
#include "crc.h"

// Typ wyliczeniowy formatu wyjściowego
typedef enum {
    FORMAT_HUMAN_READABLE,
    FORMAT_CSV
} OutputFormat;

// Deklaracja funkcji
void ReadAllSensors(ADS1115_Readings *hadc, GY85_HandleTypeDef *hgy85, OutputFormat format);

#endif // SENSOR_READER_H
