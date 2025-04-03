#ifndef SENSOR_H
#define SENSOR_H

#include "stm32f4xx_hal.h"
#include "imu_adresses.h"

// Deklaracja funkcji
uint8_t read_register(uint8_t reg);
void write_register(uint8_t reg, uint8_t value);
void sensor_init(void);
void read_accel(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z);
void read_gyro(int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z);
int16_t read_temp(void);
void read_mag(int16_t* mag_x, int16_t* mag_y, int16_t* mag_z);

#endif // SENSOR_H
