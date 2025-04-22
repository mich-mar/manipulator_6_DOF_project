#ifndef SENSOR_H
#define SENSOR_H

#include "stm32l4xx_hal.h"
#include "imu_adresses.h"
#include <stdio.h>
#include <stdint.h>    
#include "usart.h"
#include "i2c.h"

// Deklaracja funkcji

// Funkcja do odczytu rejestru z czujnika
uint8_t IMU_read_register(uint8_t reg);

// Funkcja do zapisu do rejestru w czujniku
void IMU_write_register(uint8_t reg, uint8_t value);

// Inicjalizacja czujnika
void IMU_init();

// Odczyt wartości z akcelerometru
void IMU_read_accel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);

// Odczyt wartości z żyroskopu
void IMU_read_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

// Odczyt temperatury
int16_t IMU_read_temp();

// Odczyt wartości z magnetometru
void IMU_read_mag(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);

#endif // SENSOR_H
