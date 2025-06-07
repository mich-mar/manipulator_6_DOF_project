#include "gy85_gravity.h"

// Zmienna globalna przeniesiona z definicją struktury
static GravityVector gravity = {0.0f, 0.0f, 0.0f, 0, 0};

#define GRAVITY_FILTER_ALPHA 0.1f
#define INIT_SAMPLES_REQUIRED 50

static void normalizeVector(float *x, float *y, float *z) {
    float magnitude = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (magnitude > 0.0f) {
        *x /= magnitude;
        *y /= magnitude;
        *z /= magnitude;
    }
}

static void updateGravityVector(float ax, float ay, float az) {
    float magnitude = sqrtf(ax * ax + ay * ay + az * az);
    
    if (fabsf(magnitude - 9.81f) < 2.0f) {
        if (!gravity.initialized) {
            if (gravity.init_samples == 0) {
                gravity.x = ax;
                gravity.y = ay;
                gravity.z = az;
            } else {
                gravity.x = (gravity.x * gravity.init_samples + ax) / (gravity.init_samples + 1);
                gravity.y = (gravity.y * gravity.init_samples + ay) / (gravity.init_samples + 1);
                gravity.z = (gravity.z * gravity.init_samples + az) / (gravity.init_samples + 1);
            }
            
            gravity.init_samples++;
            
            if (gravity.init_samples >= INIT_SAMPLES_REQUIRED) {
                gravity.initialized = 1;
                normalizeVector(&gravity.x, &gravity.y, &gravity.z);
                gravity.x *= 9.81f;
                gravity.y *= 9.81f;
                gravity.z *= 9.81f;
            }
        } else {
            gravity.x = (1.0f - GRAVITY_FILTER_ALPHA) * gravity.x + GRAVITY_FILTER_ALPHA * ax;
            gravity.y = (1.0f - GRAVITY_FILTER_ALPHA) * gravity.y + GRAVITY_FILTER_ALPHA * ay;
            gravity.z = (1.0f - GRAVITY_FILTER_ALPHA) * gravity.z + GRAVITY_FILTER_ALPHA * az;
            
            normalizeVector(&gravity.x, &gravity.y, &gravity.z);
            gravity.x *= 9.81f;
            gravity.y *= 9.81f;
            gravity.z *= 9.81f;
        }
    }
}

HAL_StatusTypeDef GY85_ProcessAndRemoveGravity(GY85_HandleTypeDef *hgy85)
{
    HAL_StatusTypeDef status;
    
    status = GY85_ReadAccel(hgy85);
    if (status != HAL_OK)
        return status;
    
    updateGravityVector(hgy85->accel.x, hgy85->accel.y, hgy85->accel.z);
    
    if (gravity.initialized) {
        hgy85->accel.x -= gravity.x;
        hgy85->accel.y -= gravity.y;
        hgy85->accel.z -= gravity.z;
        
        const float noise_threshold = 0.1f;
        if (fabsf(hgy85->accel.x) < noise_threshold) hgy85->accel.x = 0;
        if (fabsf(hgy85->accel.y) < noise_threshold) hgy85->accel.y = 0;
        if (fabsf(hgy85->accel.z) < noise_threshold) hgy85->accel.z = 0;
        
        hgy85->accel_raw.x = (int16_t)(hgy85->accel.x / hgy85->accel_scale);
        hgy85->accel_raw.y = (int16_t)(hgy85->accel.y / hgy85->accel_scale);
        hgy85->accel_raw.z = (int16_t)(hgy85->accel.z / hgy85->accel_scale);
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef GY85_ReadAllSensorsCompensated(GY85_HandleTypeDef *hgy85)
{
    HAL_StatusTypeDef status;
    
    status = GY85_ProcessAndRemoveGravity(hgy85);
    if (status != HAL_OK)
        return status;
    
    status = GY85_ReadGyro(hgy85);
    if (status != HAL_OK)
        return status;
    
    status = GY85_ReadTemperature(hgy85);
    if (status != HAL_OK)
        return status;
    
    return HAL_OK;
}