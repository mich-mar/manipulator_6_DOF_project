#include "GY85.h"
#include <math.h>

/* Funkcje pomocnicze prywatne */
static HAL_StatusTypeDef I2C_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2] = {reg_addr, data};
    return HAL_I2C_Master_Transmit(hi2c, device_addr, buf, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef I2C_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(hi2c, device_addr, &reg_addr, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
        return status;

    return HAL_I2C_Master_Receive(hi2c, device_addr, data, size, HAL_MAX_DELAY);
}

/* Inicjalizacja poszczególnych sensorów */
static HAL_StatusTypeDef ADXL345_Init(GY85_HandleTypeDef *hgy85)
{
    HAL_StatusTypeDef status;
    uint8_t device_id;

    // Sprawdzenie ID urządzenia
    status = I2C_ReadRegister(hgy85->hi2c, ADXL345_ADDR, ADXL345_DEVID, &device_id, 1);
    if (status != HAL_OK || device_id != 0xE5) // 0xE5 to oczekiwane ID dla ADXL345
        return HAL_ERROR;

    // Ustawienie zakresu pomiarowego na ±16g i full resolution
    status = I2C_WriteRegister(hgy85->hi2c, ADXL345_ADDR, ADXL345_DATA_FORMAT, 0x0B);
    if (status != HAL_OK)
        return status;

    // Ustawienie częstotliwości próbkowania 100Hz
    status = I2C_WriteRegister(hgy85->hi2c, ADXL345_ADDR, ADXL345_BW_RATE, 0x0A);
    if (status != HAL_OK)
        return status;

    // Włączenie pomiaru (wyjście z trybu standby)
    status = I2C_WriteRegister(hgy85->hi2c, ADXL345_ADDR, ADXL345_POWER_CTL, 0x08);
    if (status != HAL_OK)
        return status;

    // Ustawienie współczynnika skalowania dla trybu full resolution
    // W trybie full resolution mamy 3.9 mg/LSB
    hgy85->accel_scale = 9.81 * 0.0039; // Konwersja z g do m/s^2

    return HAL_OK;
}

static HAL_StatusTypeDef ITG3200_Init(GY85_HandleTypeDef *hgy85)
{
    HAL_StatusTypeDef status;
    uint8_t device_id;

    // Sprawdzenie ID urządzenia
    status = I2C_ReadRegister(hgy85->hi2c, ITG3200_ADDR, ITG3200_WHO_AM_I, &device_id, 1);
    if (status != HAL_OK || (device_id & 0x7E) != 0x68) // Bity 1-6 powinny być równe 0x68
        return HAL_ERROR;

    // Reset urządzenia
    status = I2C_WriteRegister(hgy85->hi2c, ITG3200_ADDR, ITG3200_PWR_MGM, 0x80);
    if (status != HAL_OK)
        return status;

    HAL_Delay(100); // Dajemy czas na reset

    // Ustawienie źródła zegara na wewnętrzny oscylator
    status = I2C_WriteRegister(hgy85->hi2c, ITG3200_ADDR, ITG3200_PWR_MGM, 0x00);
    if (status != HAL_OK)
        return status;

    // Częstotliwość próbkowania 100Hz (8kHz/(1+divider) = 100Hz -> divider = 79)
    status = I2C_WriteRegister(hgy85->hi2c, ITG3200_ADDR, ITG3200_SMPLRT_DIV, 79);
    if (status != HAL_OK)
        return status;

    // Ustawienie zakresu ±2000°/s i DLPF na 42Hz
    status = I2C_WriteRegister(hgy85->hi2c, ITG3200_ADDR, ITG3200_DLPF_FS, 0x1B);
    if (status != HAL_OK)
        return status;

    // Współczynnik skalowania dla ±2000°/s: 14.375 LSB/(°/s)
    hgy85->gyro_scale = (PI / 180.0) / 14.375; // Konwersja z °/s do rad/s

    return HAL_OK;
}

// static HAL_StatusTypeDef HMC5883L_Init(GY85_HandleTypeDef *hgy85)
// {
//     HAL_StatusTypeDef status;
//     uint8_t device_id[3];

//     // Sprawdzenie ID urządzenia (3 kolejne rejestry ID)
//     status = I2C_ReadRegister(hgy85->hi2c, HMC5883L_ADDR, HMC5883L_ID_A, device_id, 3);
//     if (status != HAL_OK || device_id[0] != 'H' || device_id[1] != '4' || device_id[2] != '3')
//         return HAL_ERROR;

//     // Konfiguracja trybu pracy
//     // - 8 próbek uśrednianych
//     // - Częstotliwość próbkowania 15Hz
//     // - Normalny tryb pomiarowy
//     status = I2C_WriteRegister(hgy85->hi2c, HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x70);
//     if (status != HAL_OK)
//         return status;

//     // Ustawienie zakresu ±1.3Ga
//     status = I2C_WriteRegister(hgy85->hi2c, HMC5883L_ADDR, HMC5883L_CONFIG_B, 0x20);
//     if (status != HAL_OK)
//         return status;

//     // Ustawienie trybu ciągłego pomiaru
//     status = I2C_WriteRegister(hgy85->hi2c, HMC5883L_ADDR, HMC5883L_MODE, 0x00);
//     if (status != HAL_OK)
//         return status;

//     // Współczynnik skalowania dla ±1.3Ga: 1090 LSB/Gauss
//     hgy85->mag_scale = 1.0 / 1090.0;

//     return HAL_OK;
// }


// implementacja na podstawie gotowego programu z dokumentacji - nie działa
static HAL_StatusTypeDef HMC5883L_Init(GY85_HandleTypeDef *hgy85)
{
    HAL_StatusTypeDef status;
    char debug_buffer[128];

    sendUSBmsg("\r\nHMC5883L: Starting initialization...\r\n");
    
    // 1. Sprawdź stan linii
    I2C_CheckPins();
    
    // 2. Jeśli linie nie są w stanie wysokim, zresetuj magistralę
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET ||
       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
        sendUSBmsg("I2C lines stuck low, attempting reset...\r\n");
        I2C_ResetBus(hgy85);
        I2C_CheckPins();
    }
    
    // 3. Skanuj magistralę
    I2C_ScanBus(hgy85->hi2c);
    
    // 4. Spróbuj różne adresy HMC5883L
    const uint8_t possible_addresses[] = {0x1E, 0x3C, 0x3D};
    for(int i = 0; i < sizeof(possible_addresses); i++) {
        status = HAL_I2C_IsDeviceReady(hgy85->hi2c, possible_addresses[i] << 1, 1, 100);
        sprintf(debug_buffer, "Testing address 0x%02X: %s\r\n", 
                possible_addresses[i], 
                status == HAL_OK ? "FOUND!" : "not found");
        sendUSBmsg(debug_buffer);
    }
    
    // 5. Stan I2C
    sprintf(debug_buffer, "I2C State: %lu, Error: %lu\r\n", 
            HAL_I2C_GetState(hgy85->hi2c), 
            HAL_I2C_GetError(hgy85->hi2c));
    sendUSBmsg(debug_buffer);

    return HAL_OK;  // Zwróć OK nawet jeśli nie znaleziono urządzenia
}

/* Implementacja funkcji z pliku nagłówkowego */
HAL_StatusTypeDef GY85_Init(GY85_HandleTypeDef *hgy85, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    // Inicjalizacja struktury
    hgy85->hi2c = hi2c;

    // Inicjalizacja akcelerometru
    status = ADXL345_Init(hgy85);
    if (status != HAL_OK)
        return status;

    // Inicjalizacja żyroskopu
    status = ITG3200_Init(hgy85);
    if (status != HAL_OK)
        return status;

    // Inicjalizacja magnetometru
     status = HMC5883L_Init(hgy85);
     if (status != HAL_OK)
         return status;

    return HAL_OK;
}

HAL_StatusTypeDef GY85_ReadAccel(GY85_HandleTypeDef *hgy85)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = I2C_ReadRegister(hgy85->hi2c, ADXL345_ADDR, ADXL345_DATAX0, data, 6);
    if (status != HAL_OK)
        return status;

    // Dane są w formacie little-endian
    hgy85->accel_raw.x = (int16_t)((data[1] << 8) | data[0]);
    hgy85->accel_raw.y = (int16_t)((data[3] << 8) | data[2]);
    hgy85->accel_raw.z = (int16_t)((data[5] << 8) | data[4]);

    // Skalowanie do jednostek fizycznych (m/s^2)
    hgy85->accel.x = hgy85->accel_raw.x * hgy85->accel_scale;
    hgy85->accel.y = hgy85->accel_raw.y * hgy85->accel_scale;
    hgy85->accel.z = hgy85->accel_raw.z * hgy85->accel_scale;

    return HAL_OK;
}

HAL_StatusTypeDef GY85_ReadGyro(GY85_HandleTypeDef *hgy85)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = I2C_ReadRegister(hgy85->hi2c, ITG3200_ADDR, ITG3200_GYRO_XOUT_H, data, 6);
    if (status != HAL_OK)
        return status;

    // Dane są w formacie big-endian
    hgy85->gyro_raw.x = (int16_t)((data[0] << 8) | data[1]);
    hgy85->gyro_raw.y = (int16_t)((data[2] << 8) | data[3]);
    hgy85->gyro_raw.z = (int16_t)((data[4] << 8) | data[5]);

    // Skalowanie do jednostek fizycznych (rad/s)
    hgy85->gyro.x = hgy85->gyro_raw.x * hgy85->gyro_scale;
    hgy85->gyro.y = hgy85->gyro_raw.y * hgy85->gyro_scale;
    hgy85->gyro.z = hgy85->gyro_raw.z * hgy85->gyro_scale;

    return HAL_OK;
}

HAL_StatusTypeDef GY85_ReadMag(GY85_HandleTypeDef *hgy85)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    status = I2C_ReadRegister(hgy85->hi2c, HMC5883L_ADDR, HMC5883L_DATA_X_MSB, data, 6);
    if (status != HAL_OK)
        return status;

    // Dane są w formacie big-endian
    // Kolejność w HMC5883L: X, Z, Y
    hgy85->mag_raw.x = (int16_t)((data[0] << 8) | data[1]);
    hgy85->mag_raw.z = (int16_t)((data[2] << 8) | data[3]);
    hgy85->mag_raw.y = (int16_t)((data[4] << 8) | data[5]);

    // Skalowanie do jednostek fizycznych (Gauss)
    hgy85->mag.x = hgy85->mag_raw.x * hgy85->mag_scale;
    hgy85->mag.y = hgy85->mag_raw.y * hgy85->mag_scale;
    hgy85->mag.z = hgy85->mag_raw.z * hgy85->mag_scale;

    return HAL_OK;
}

HAL_StatusTypeDef GY85_ReadTemperature(GY85_HandleTypeDef *hgy85)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;
    int16_t raw_temp;

    status = I2C_ReadRegister(hgy85->hi2c, ITG3200_ADDR, ITG3200_TEMP_OUT_H, data, 2);
    if (status != HAL_OK)
        return status;

    // Temperatura w formacie big-endian
    raw_temp = (int16_t)((data[0] << 8) | data[1]);

    // Konwersja do stopni Celsjusza (zgodnie z dokumentacją ITG3200)
    // 35°C = 0 offset
    // Czułość to 280 LSB/°C
    hgy85->temperature = 35.0 + ((float)raw_temp + 13200.0) / 280.0;

    return HAL_OK;
}

HAL_StatusTypeDef GY85_ReadAllSensors(GY85_HandleTypeDef *hgy85)
{
    HAL_StatusTypeDef status;

    status = GY85_ReadAccel(hgy85);
    if (status != HAL_OK)
        return status;

    status = GY85_ReadGyro(hgy85);
    if (status != HAL_OK)
        return status;

    //  status = GY85_ReadMag(hgy85);
    //  if (status != HAL_OK)
    //      return status;

    status = GY85_ReadTemperature(hgy85);
    if (status != HAL_OK)
        return status;

    return HAL_OK;
}

HAL_StatusTypeDef GY85_SelfTest(GY85_HandleTypeDef *hgy85)
{
    uint8_t device_id;
    HAL_StatusTypeDef status;

    // Sprawdzenie akcelerometru
    status = I2C_ReadRegister(hgy85->hi2c, ADXL345_ADDR, ADXL345_DEVID, &device_id, 1);
    if (status != HAL_OK || device_id != 0xE5)
        return HAL_ERROR;

    // Sprawdzenie żyroskopu
    status = I2C_ReadRegister(hgy85->hi2c, ITG3200_ADDR, ITG3200_WHO_AM_I, &device_id, 1);
    if (status != HAL_OK || (device_id & 0x7E) != 0x68)
        return HAL_ERROR;

    // Sprawdzenie magnetometru
    uint8_t id_bytes[3];
    status = I2C_ReadRegister(hgy85->hi2c, HMC5883L_ADDR, HMC5883L_ID_A, id_bytes, 3);
    if (status != HAL_OK || id_bytes[0] != 'H' || id_bytes[1] != '4' || id_bytes[2] != '3')
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef GY85_SendAllData(GY85_HandleTypeDef *hgy85)
{
    HAL_StatusTypeDef status;
    char buffer[512];
    char tempBuffer[32]; // na wszelki wypadek trochę większy
    int length = 0;

    status = GY85_ReadAllSensors(hgy85);
    if (status != HAL_OK)
        return status;

    // Formatowanie danych
    floatToString(hgy85->accel.x, tempBuffer);
    length += sprintf(buffer + length, "ACCEL: X=%s ", tempBuffer);

    floatToString(hgy85->accel.y, tempBuffer);
    length += sprintf(buffer + length, "Y=%s ", tempBuffer);

    floatToString(hgy85->accel.z, tempBuffer);
    length += sprintf(buffer + length, "Z=%s m/s^2\r\n", tempBuffer);

    floatToString(hgy85->gyro.x, tempBuffer);
    length += sprintf(buffer + length, "GYRO: X=%s ", tempBuffer);

    floatToString(hgy85->gyro.y, tempBuffer);
    length += sprintf(buffer + length, "Y=%s ", tempBuffer);

    floatToString(hgy85->gyro.z, tempBuffer);
    length += sprintf(buffer + length, "Z=%s rad/s\r\n", tempBuffer);

    floatToString(hgy85->mag.x, tempBuffer);
    length += sprintf(buffer + length, "MAG: X=%s ", tempBuffer);

    floatToString(hgy85->mag.y, tempBuffer);
    length += sprintf(buffer + length, "Y=%s ", tempBuffer);

    floatToString(hgy85->mag.z, tempBuffer);
    length += sprintf(buffer + length, "Z=%s Gauss\r\n", tempBuffer);

    floatToString(hgy85->temperature, tempBuffer);
    length += sprintf(buffer + length, "TEMP: %s C\r\n\r\n", tempBuffer);

    // Wysłanie przez UART
    sendUSBmsg(buffer);

    return HAL_OK;
}


/* Funkcja inicjalizująca IMU */
HAL_StatusTypeDef GY85_Begin(GY85_HandleTypeDef *hgy85, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    status = GY85_Init(hgy85, hi2c);
    if (status != HAL_OK)
        return status;

    // Opcjonalnie można dodać self-test
    // status = GY85_SelfTest(hgy85);
    // if (status != HAL_OK)
    //     return status;

    return HAL_OK;
}

static void I2C_ScanBus(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    char debug_buffer[128];
    int devices_found = 0;
    
    sendUSBmsg("\r\nScanning I2C bus...\r\n");

    // Skanuj wszystkie możliwe 7-bitowe adresy
    for(uint8_t addr = 1; addr < 128; addr++) {
        // Przesuń 7-bitowy adres o 1 w lewo (format 8-bitowy)
        status = HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 10);
        
        if(status == HAL_OK) {
            sprintf(debug_buffer, "Found device at address 0x%02X (7-bit) / 0x%02X (8-bit)\r\n", 
                    addr, addr << 1);
            sendUSBmsg(debug_buffer);
            devices_found++;
        }
    }
    
    if(devices_found == 0) {
        sendUSBmsg("No I2C devices found!\r\n");
    } else {
        sprintf(debug_buffer, "Found %d device(s)\r\n", devices_found);
        sendUSBmsg(debug_buffer);
    }
}

// I sprawdźmy stan linii I2C
static void I2C_CheckPins(void)
{
    char debug_buffer[128];
    GPIO_PinState scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);  // SCL
    GPIO_PinState sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);  // SDA
    
    sprintf(debug_buffer, "I2C pins state - SCL: %d, SDA: %d (should both be 1)\r\n", 
            (int)scl, (int)sda);
    sendUSBmsg(debug_buffer);
}

// Dodajmy też reset magistrali I2C
static HAL_StatusTypeDef I2C_ResetBus(GY85_HandleTypeDef *hgy85)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    char debug_buffer[128];
    
    sendUSBmsg("Resetting I2C bus...\r\n");
    
    // 1. Wyłącz I2C
    __HAL_RCC_I2C1_CLK_DISABLE();
    
    // 2. Skonfiguruj piny jako wyjścia
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;  // SCL i SDA
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    // Open-drain
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 3. Wymuś stan wysoki
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);
    
    // 4. Wygeneruj 9 impulsów na SCL
    for(int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }
    
    // 5. Generuj STOP (SDA low->high while SCL is high)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);
    
    // 6. Przywróć piny do trybu I2C
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 7. Włącz z powrotem I2C
    __HAL_RCC_I2C1_CLK_ENABLE();
    
    // 8. Reinicjalizuj I2C
    if(HAL_I2C_Init(hgy85->hi2c) != HAL_OK) {
        sendUSBmsg("Failed to reinitialize I2C\r\n");
        return HAL_ERROR;
    }
    
    sendUSBmsg("I2C bus reset complete\r\n");
    return HAL_OK;
}