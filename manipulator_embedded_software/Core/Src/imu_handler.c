#include "imu_handler.h"

// Funkcja do odczytu rejestru z czujnika
uint8_t read_register(uint8_t reg) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, DEVICE_ID << 1, reg, 1, &data, 1, HAL_MAX_DELAY);
    return data;
}

// Funkcja do zapisu do rejestru w czujniku
void write_register(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, DEVICE_ID << 1, reg, 1, &value, 1, HAL_MAX_DELAY);
}

// Inicjalizacja czujnika
void sensor_init() {
    uint8_t who_am_i = read_register(WHO_AM_I);
    if (who_am_i != DEVICE_ID) {
        // Błąd, nie znaleziono urządzenia
        while (1);
    }
    
    // Inicjalizacja czujnika (np. ustawienie trybu pracy)
    write_register(PWR_MGMT_1, 0x01);  // Ustawienie trybu pracy (włączony czujnik)
    HAL_Delay(100);
}

// Odczyt wartości z akcelerometru
void read_accel(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, DEVICE_ID << 1, ACCEL_XOUT_H, 1, data, 6, HAL_MAX_DELAY);
    
    *accel_x = (int16_t)((data[0] << 8) | data[1]);
    *accel_y = (int16_t)((data[2] << 8) | data[3]);
    *accel_z = (int16_t)((data[4] << 8) | data[5]);
}

// Odczyt wartości z żyroskopu
void read_gyro(int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, DEVICE_ID << 1, GYRO_XOUT_H, 1, data, 6, HAL_MAX_DELAY);
    
    *gyro_x = (int16_t)((data[0] << 8) | data[1]);
    *gyro_y = (int16_t)((data[2] << 8) | data[3]);
    *gyro_z = (int16_t)((data[4] << 8) | data[5]);
}

// Odczyt temperatury
int16_t read_temp() {
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c1, DEVICE_ID << 1, TEMP_OUT_H, 1, data, 2, HAL_MAX_DELAY);
    
    return (int16_t)((data[0] << 8) | data[1]);
}

// Odczyt wartości z magnetometru
void read_mag(int16_t* mag_x, int16_t* mag_y, int16_t* mag_z) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, DEVICE_ID << 1, MAG_XOUT_H, 1, data, 6, HAL_MAX_DELAY);
    
    *mag_x = (int16_t)((data[0] << 8) | data[1]);
    *mag_y = (int16_t)((data[2] << 8) | data[3]);
    *mag_z = (int16_t)((data[4] << 8) | data[5]);
}

