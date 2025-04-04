#include "ICM20948.h"
#include "stm32l4xx_hal.h" 
#include "imu_adresses.h"

// Konfiguracja Full Scale
enum {
    ACCEL_FS_2G  = 0,
    ACCEL_FS_4G  = 1,
    ACCEL_FS_8G  = 2,
    ACCEL_FS_16G = 3
};

enum {
    GYRO_FS_250DPS  = 0,
    GYRO_FS_500DPS  = 1,
    GYRO_FS_1000DPS = 2,
    GYRO_FS_2000DPS = 3
};

ICM20948_StatusTypeDef ICM20948_Init(ICM20948_HandleTypeDef* hicm, I2C_HandleTypeDef* hi2c) {
    hicm->hi2c = hi2c;
    hicm->addr = ICM20948_I2C_ADDR;

    // Reset urządzenia
    ICM20948_WriteReg(hicm, PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    
    // Włącz główny zegar
    ICM20948_WriteReg(hicm, PWR_MGMT_1, 0x01);
    HAL_Delay(10);
    
    // Sprawdź ID
    uint8_t whoami;
    ICM20948_ReadRegs(hicm, WHO_AM_I, &whoami, 1);
    if(whoami != 0xEA) return ICM20948_ERROR;
    
    // Konfiguracja domyślna
    ICM20948_SetGyroFullScale(hicm, GYRO_FS_2000DPS);
    ICM20948_SetAccelFullScale(hicm, ACCEL_FS_16G);
    
    return ICM20948_OK;
}

ICM20948_StatusTypeDef ICM20948_ReadAccel(ICM20948_HandleTypeDef* hicm) {
    uint8_t data[6];
    ICM20948_StatusTypeDef status = ICM20948_ReadRegs(hicm, ACCEL_XOUT_H, data, 6);
    
    if(status == ICM20948_OK) {
        int16_t raw_x = (data[0] << 8) | data[1];
        int16_t raw_y = (data[2] << 8) | data[3];
        int16_t raw_z = (data[4] << 8) | data[5];
        
        // TODO: Dodać skalowanie w oparciu o wybrany zakres
        hicm->accel.x = raw_x * 0.000488f;
        hicm->accel.y = raw_y * 0.000488f;
        hicm->accel.z = raw_z * 0.000488f;
    }
    
    return status;
}

ICM20948_StatusTypeDef ICM20948_ReadGyro(ICM20948_HandleTypeDef* hicm) {
    uint8_t data[6];
    ICM20948_StatusTypeDef status = ICM20948_ReadRegs(hicm, GYRO_XOUT_H, data, 6);
    
    if(status == ICM20948_OK) {
        int16_t raw_x = (data[0] << 8) | data[1];
        int16_t raw_y = (data[2] << 8) | data[3];
        int16_t raw_z = (data[4] << 8) | data[5];
        
        // TODO: Dodać skalowanie w oparciu o wybrany zakres
        hicm->gyro.x = raw_x * 0.007629f;
        hicm->gyro.y = raw_y * 0.007629f;
        hicm->gyro.z = raw_z * 0.007629f;
    }
    
    return status;
}

ICM20948_StatusTypeDef ICM20948_ReadTemp(ICM20948_HandleTypeDef* hicm) {
    uint8_t data[2];
    ICM20948_StatusTypeDef status = ICM20948_ReadRegs(hicm, TEMP_OUT_H, data, 2);
    
    if(status == ICM20948_OK) {
        int16_t raw_temp = (data[0] << 8) | data[1];
        hicm->temperature = (raw_temp / 333.87f) + 21.0f;
    }
    
    return status;
}

 ICM20948_StatusTypeDef ICM20948_WriteReg(ICM20948_HandleTypeDef* hicm, uint8_t reg, uint8_t value) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        hicm->hi2c,
        hicm->addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100);
        
    return (status == HAL_OK) ? ICM20948_OK : ICM20948_ERROR;
}

 ICM20948_StatusTypeDef ICM20948_ReadRegs(ICM20948_HandleTypeDef* hicm, uint8_t reg, uint8_t* data, uint16_t len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hicm->hi2c,
        hicm->addr,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        100);
        
    return (status == HAL_OK) ? ICM20948_OK : ICM20948_ERROR;
}