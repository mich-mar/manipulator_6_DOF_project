/**
 * @file gy85.h
 * @brief Biblioteka do obsługi IMU GY-85 (ADXL345 + ITG3200 + HMC5883L) na STM32 z HAL
 * @author Claude
 * @date 25.04.2025
 */

 #ifndef GY85_H
 #define GY85_H
 
 #include "stm32f4xx_hal.h" // Dostosuj do swojej rodziny STM32
 #include "stdint.h"
 #include "stdio.h"
 #include "i2c.h"
 #include "sendUSB.h"

 #define PI 3.1415
 
 /* Adresy I2C urządzeń */
 #define ADXL345_ADDR        0x53 << 1 // Akcelerometr
 #define ITG3200_ADDR        0x68 << 1 // Żyroskop
 #define HMC5883L_ADDR       0x1E << 1 // Magnetometr
 
 /* Rejestry ADXL345 */
 #define ADXL345_DEVID           0x00
 #define ADXL345_POWER_CTL       0x2D
 #define ADXL345_DATA_FORMAT     0x31
 #define ADXL345_DATAX0          0x32
 #define ADXL345_FIFO_CTL        0x38
 #define ADXL345_BW_RATE         0x2C
 
 /* Rejestry ITG3200 */
 #define ITG3200_WHO_AM_I        0x00
 #define ITG3200_SMPLRT_DIV      0x15
 #define ITG3200_DLPF_FS         0x16
 #define ITG3200_INT_CFG         0x17
 #define ITG3200_TEMP_OUT_H      0x1B
 #define ITG3200_TEMP_OUT_L      0x1C
 #define ITG3200_GYRO_XOUT_H     0x1D
 #define ITG3200_PWR_MGM         0x3E
 
 /* Rejestry HMC5883L */
 #define HMC5883L_CONFIG_A       0x00
 #define HMC5883L_CONFIG_B       0x01
 #define HMC5883L_MODE           0x02
 #define HMC5883L_DATA_X_MSB     0x03
 #define HMC5883L_STATUS         0x09
 #define HMC5883L_ID_A           0x0A
 
 /* Struktury danych */
 typedef struct {
     int16_t x;
     int16_t y;
     int16_t z;
 } GY85_RawData;
 
 typedef struct {
     float x;
     float y;
     float z;
 } GY85_ScaledData;
 
 typedef struct {
     I2C_HandleTypeDef *hi2c;
     
     // Współczynniki skalowania
     float accel_scale;      // m/s^2 na jednostkę
     float gyro_scale;       // rad/s na jednostkę
     float mag_scale;        // gauss na jednostkę
     
     // Dane surowe
     GY85_RawData accel_raw;
     GY85_RawData gyro_raw;
     GY85_RawData mag_raw;
     
     // Dane po skalowaniu
     GY85_ScaledData accel;  // w m/s^2
     GY85_ScaledData gyro;   // w rad/s
     GY85_ScaledData mag;    // w gaussach
     
     // Temperatura z żyroskopu
     float temperature;      // w stopniach Celsjusza
     
 } GY85_HandleTypeDef;
 
 /* Prototypy funkcji */
 HAL_StatusTypeDef GY85_Init(GY85_HandleTypeDef *hgy85, I2C_HandleTypeDef *hi2c);
 HAL_StatusTypeDef GY85_ReadAllSensors(GY85_HandleTypeDef *hgy85);
 HAL_StatusTypeDef GY85_ReadAccel(GY85_HandleTypeDef *hgy85);
 HAL_StatusTypeDef GY85_ReadGyro(GY85_HandleTypeDef *hgy85);
 HAL_StatusTypeDef GY85_ReadMag(GY85_HandleTypeDef *hgy85);
 HAL_StatusTypeDef GY85_ReadTemperature(GY85_HandleTypeDef *hgy85);
 HAL_StatusTypeDef GY85_SelfTest(GY85_HandleTypeDef *hgy85);
 HAL_StatusTypeDef GY85_SendAllData(GY85_HandleTypeDef *hgy85);
 HAL_StatusTypeDef GY85_Begin(GY85_HandleTypeDef *hgy85, I2C_HandleTypeDef *hi2c);
 
 #endif /* GY85_H */