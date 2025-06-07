#ifndef GY85_GRAVITY_H
#define GY85_GRAVITY_H

#include "gy85.h"
#include <math.h>

// Struktura przeniesiona do pliku nagłówkowego
typedef struct {
    float x;
    float y;
    float z;
    int initialized;
    uint32_t init_samples;
} GravityVector;

// Deklaracje funkcji
HAL_StatusTypeDef GY85_ProcessAndRemoveGravity(GY85_HandleTypeDef *hgy85);
HAL_StatusTypeDef GY85_ReadAllSensorsCompensated(GY85_HandleTypeDef *hgy85);

#endif // GY85_GRAVITY_H