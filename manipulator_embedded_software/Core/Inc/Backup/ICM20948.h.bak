#ifndef ICM20948_H
#define ICM20948_H

#include "stm32l4xx_hal.h" 
#include "imu_adresses.h"

typedef enum {
    ICM20948_OK = 0,
    ICM20948_ERROR
} ICM20948_StatusTypeDef;

typedef struct {
    float x, y, z;
} ICM20948_AxisData;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t addr;
    ICM20948_AxisData accel;
    ICM20948_AxisData gyro;
    float temperature;
} ICM20948_HandleTypeDef;

ICM20948_StatusTypeDef ICM20948_Init(ICM20948_HandleTypeDef* hicm, I2C_HandleTypeDef* hi2c);
ICM20948_StatusTypeDef ICM20948_ReadAccel(ICM20948_HandleTypeDef* hicm);
ICM20948_StatusTypeDef ICM20948_ReadGyro(ICM20948_HandleTypeDef* hicm);
ICM20948_StatusTypeDef ICM20948_ReadTemp(ICM20948_HandleTypeDef* hicm);
ICM20948_StatusTypeDef ICM20948_WriteReg(ICM20948_HandleTypeDef* hicm, uint8_t reg, uint8_t value);
ICM20948_StatusTypeDef ICM20948_ReadRegs(ICM20948_HandleTypeDef* hicm, uint8_t reg, uint8_t* data, uint16_t len);

#endif /* ICM20948_H */
