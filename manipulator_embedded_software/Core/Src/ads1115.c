#include "ads1115.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1; // Dostosuj do swojej konfiguracji I2C

// Struktura do przechowywania informacji debugowania
ADS1115_Debug_t ADS1115_Debug = {0};

/**
 * @brief Sprawdza, czy urządzenie ADS1115 jest podłączone do magistrali I2C
 * @return true jeśli wykryto urządzenie, false w przeciwnym wypadku
 */
bool ADS1115_IsDeviceConnected(void) {
    uint8_t reg = ADS1115_REG_CONFIG;
    uint8_t dummy_data;
    HAL_StatusTypeDef status;
    
    // Próba komunikacji z urządzeniem
    status = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_I2C_ADDR << 1, &reg, 1, 10);
    if (status != HAL_OK) return false;
    
    // Próba odczytu danych
    status = HAL_I2C_Master_Receive(&hi2c1, ADS1115_I2C_ADDR << 1, &dummy_data, 1, 10);
    
    ADS1115_Debug.device_detected = (status == HAL_OK);
    return (status == HAL_OK);
}

/**
 * @brief Inicjalizacja ADS1115
 * @return Status inicjalizacji
 */
ADS1115_Status ADS1115_Init(void) {
    // Sprawdź, czy urządzenie jest podłączone
    if (!ADS1115_IsDeviceConnected()) {
        ADS1115_Debug.last_error_code = ADS1115_ERROR_INIT;
        return ADS1115_ERROR_INIT;
    }
    
    // Ustawienie domyślnej konfiguracji
    uint16_t config = ADS1115_OS_SINGLE |     // Start pojedynczej konwersji
                     ADS1115_MUX_SINGLE_0 |  // Kanał A0
                     ADS1115_PGA_2_048V |    // Zakres ±2.048V
                     ADS1115_MODE_SINGLESHOT | // Tryb pojedynczego odczytu
                     ADS1115_DR_860SPS |     // 128 próbek na sekundę
                     ADS1115_COMP_QUE_DISABLE; // Wyłączenie komparatora
    
    // Zapisz konfigurację do rejestru CONFIG
    HAL_StatusTypeDef status = ADS1115_WriteRegister(ADS1115_REG_CONFIG, config);
    
    if (status != HAL_OK) {
        ADS1115_Debug.last_error_code = ADS1115_ERROR_I2C;
        ADS1115_Debug.i2c_errors++;
        return ADS1115_ERROR_I2C;
    }
    
    ADS1115_Debug.last_config = config;
    return ADS1115_OK;
}

/**
 * @brief Zapis wartości do rejestru ADS1115
 * @param reg Adres rejestru
 * @param value Wartość do zapisania
 * @return Status HAL
 */
HAL_StatusTypeDef ADS1115_WriteRegister(uint8_t reg, uint16_t value) {
    uint8_t data[3];
    data[0] = reg;               // Adres rejestru
    data[1] = (value >> 8) & 0xFF; // Starszy bajt
    data[2] = value & 0xFF;        // Młodszy bajt
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_I2C_ADDR << 1, data, 3, 100);
    
    if (status != HAL_OK) {
        ADS1115_Debug.i2c_errors++;
    }
    
    return status;
}

/**
 * @brief Odczyt wartości z rejestru ADS1115
 * @param reg Adres rejestru
 * @param value Wskaźnik do zmiennej, gdzie zostanie zapisana odczytana wartość
 * @return Status HAL
 */
HAL_StatusTypeDef ADS1115_ReadRegister(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    HAL_StatusTypeDef status;
    
    // Wybór rejestru do odczytu
    status = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_I2C_ADDR << 1, &reg, 1, 100);
    if(status != HAL_OK) {
        ADS1115_Debug.i2c_errors++;
        return status;
    }
    
    // Odczyt wartości
    status = HAL_I2C_Master_Receive(&hi2c1, ADS1115_I2C_ADDR << 1, data, 2, 100);
    if(status != HAL_OK) {
        ADS1115_Debug.i2c_errors++;
        return status;
    }
    
    *value = ((uint16_t)data[0] << 8) | data[1];
    return HAL_OK;
}

/**
 * @brief Rozpoczęcie konwersji z określoną konfiguracją
 * @param config Wartość rejestru konfiguracyjnego
 * @return Status HAL
 */
HAL_StatusTypeDef ADS1115_StartConversion(uint16_t config) {
    // Ustaw bit OS na 1 aby rozpocząć konwersję
    config |= ADS1115_OS_SINGLE;
    HAL_StatusTypeDef status = ADS1115_WriteRegister(ADS1115_REG_CONFIG, config);
    
    if (status == HAL_OK) {
        ADS1115_Debug.last_config = config;
    }
    
    return status;
}

/**
 * @brief Sprawdzenie czy konwersja jest zakończona
 * @param ready Wskaźnik do zmiennej, gdzie zostanie zapisany status (1 = gotowe, 0 = w trakcie)
 * @return Status HAL
 */
HAL_StatusTypeDef ADS1115_IsConversionReady(uint8_t *ready) {
    uint16_t config;
    HAL_StatusTypeDef status = ADS1115_ReadRegister(ADS1115_REG_CONFIG, &config);
    if(status != HAL_OK) return status;
    
    // Bit OS: 1 = gotowe, 0 = w trakcie
    *ready = (config & ADS1115_OS_NOTBUSY) ? 1 : 0;
    return HAL_OK;
}

/**
 * @brief Odczyt ostatniego wyniku konwersji
 * @param status Wskaźnik na zmienną do zapisania statusu operacji
 * @return Wartość odczytana z rejestru CONVERSION
 */
int16_t ADS1115_Read(ADS1115_Status* status) {
    uint16_t value;
    HAL_StatusTypeDef hal_status = ADS1115_ReadRegister(ADS1115_REG_CONVERSION, &value);
    
    if (hal_status != HAL_OK) {
        if (status) *status = ADS1115_ERROR_I2C;
        ADS1115_Debug.i2c_errors++;
        ADS1115_Debug.last_error_code = ADS1115_ERROR_I2C;
        return 0;
    }
    
    ADS1115_Debug.last_raw_value = (int16_t)value;
    if (status) *status = ADS1115_OK;
    ADS1115_Debug.successful_reads++;
    
    return (int16_t)value;
}

/**
 * @brief Odczyt wartości z wybranego kanału w trybie single-ended
 * @param channel Numer kanału (0-3)
 * @param status Wskaźnik na zmienną do zapisania statusu operacji
 * @return Odczytana wartość
 */
int16_t ADS1115_ReadADC_SingleEnded(uint8_t channel, ADS1115_Status* status) {
    ADS1115_Debug.total_reads++;
    
    // Sprawdź parametr
    if (channel > 3) {
        if (status) *status = ADS1115_ERROR_PARAM;
        ADS1115_Debug.last_error_code = ADS1115_ERROR_PARAM;
        return 0;
    }
    
    // Sprawdź, czy urządzenie jest podłączone
    if (!ADS1115_Debug.device_detected && !ADS1115_IsDeviceConnected()) {
        if (status) *status = ADS1115_ERROR_I2C;
        ADS1115_Debug.last_error_code = ADS1115_ERROR_I2C;
        return 0;
    }
    
    uint16_t mux;
    
    // Wybór kanału
    switch(channel) {
        case 0:
            mux = ADS1115_MUX_SINGLE_0;
            break;
        case 1:
            mux = ADS1115_MUX_SINGLE_1;
            break;
        case 2:
            mux = ADS1115_MUX_SINGLE_2;
            break;
        case 3:
            mux = ADS1115_MUX_SINGLE_3;
            break;
        default:
            mux = ADS1115_MUX_SINGLE_0;
            break;
    }
    
    // Konfiguracja ADC
    uint16_t config = ADS1115_OS_SINGLE |     // Start konwersji
                     mux |                    // Wybrany kanał
                     ADS1115_PGA_6_144V |     // Zakres ±2.048V
                     ADS1115_MODE_SINGLESHOT | // Tryb pojedynczego odczytu
                     ADS1115_DR_128SPS |      // 128 próbek na sekundę
                     ADS1115_COMP_QUE_DISABLE; // Wyłączenie komparatora
    
    // Rozpocznij konwersję
    HAL_StatusTypeDef hal_status = ADS1115_WriteRegister(ADS1115_REG_CONFIG, config);
    
    if (hal_status != HAL_OK) {
        if (status) *status = ADS1115_ERROR_I2C;
        ADS1115_Debug.i2c_errors++;
        ADS1115_Debug.last_error_code = ADS1115_ERROR_I2C;
        return 0;
    }
    
    ADS1115_Debug.last_config = config;
    
    // Poczekaj na zakończenie konwersji z timeoutem
    uint8_t ready = 0;
    uint32_t startTime = HAL_GetTick();
    uint32_t currentTime;
    
    do {
        HAL_Delay(1); // Krótkie opóźnienie
        
        // Sprawdź, czy konwersja jest zakończona
        hal_status = ADS1115_IsConversionReady(&ready);
        if (hal_status != HAL_OK) {
            if (status) *status = ADS1115_ERROR_I2C;
            ADS1115_Debug.i2c_errors++;
            ADS1115_Debug.last_error_code = ADS1115_ERROR_I2C;
            return 0;
        }
        
        // Sprawdź timeout
        currentTime = HAL_GetTick();
        if (currentTime - startTime > ADS1115_CONVERSION_TIMEOUT) {
            if (status) *status = ADS1115_ERROR_TIMEOUT;
            ADS1115_Debug.timeouts++;
            ADS1115_Debug.last_error_code = ADS1115_ERROR_TIMEOUT;
            return 0;
        }
    } while(!ready);
    
    // Odczytaj wynik
    int16_t result = ADS1115_Read(status);
    
    // Oblicz napięcie dla celów debugowania
    ADS1115_Debug.last_voltage = ADS1115_ConvertToVoltage(result, ADS1115_PGA_2_048V);
    
    return result;
}

/**
 * @brief Odczyt wartości z kanału A0 (wejście analogowe 0)
 * @param status Wskaźnik na zmienną do zapisania statusu operacji
 * @return Odczytana wartość
 */
int16_t ADS1115_ReadADC_A0(ADS1115_Status* status) {
    return ADS1115_ReadADC_SingleEnded(0, status);
}

/**
 * @brief Odczyt wartości z kanału A1 (wejście analogowe 1)
 * @param status Wskaźnik na zmienną do zapisania statusu operacji
 * @return Odczytana wartość
 */
int16_t ADS1115_ReadADC_A1(ADS1115_Status* status) {
    return ADS1115_ReadADC_SingleEnded(1, status);
}


/**
 * @brief Odczyt wartości z kanału A2 (wejście analogowe 2)
 * @param status Wskaźnik na zmienną do zapisania statusu operacji
 * @return Odczytana wartość
 */
int16_t ADS1115_ReadADC_A2(ADS1115_Status* status) {
    return ADS1115_ReadADC_SingleEnded(2, status);
}


/**
 * @brief Konwersja wartości surowej ADC na napięcie
 * @param raw_adc Surowa wartość z ADC
 * @param pga Wartość wzmocnienia (np. ADS1115_PGA_2_048V)
 * @return Napięcie w woltach
 */
float ADS1115_ConvertToVoltage(int16_t raw_adc, uint16_t pga) {
    float voltage = 0.0f;
    
    // Współczynnik konwersji zależy od wybranego wzmocnienia (PGA)
    switch(pga) {
        case ADS1115_PGA_6_144V:
            voltage = (float)raw_adc * 6.144f / 32768.0f;
            break;
        case ADS1115_PGA_4_096V:
            voltage = (float)raw_adc * 4.096f / 32768.0f;
            break;
        case ADS1115_PGA_2_048V:
            voltage = (float)raw_adc * 2.048f / 32768.0f;
            break;
        case ADS1115_PGA_1_024V:
            voltage = (float)raw_adc * 1.024f / 32768.0f;
            break;
        case ADS1115_PGA_0_512V:
            voltage = (float)raw_adc * 0.512f / 32768.0f;
            break;
        case ADS1115_PGA_0_256V:
            voltage = (float)raw_adc * 0.256f / 32768.0f;
            break;
        default:
            voltage = (float)raw_adc * 2.048f / 32768.0f; // domyślne 2.048V
            break;
    }
    
    return voltage;
}

/**
 * @brief Zwraca opis słowny kodu błędu
 * @param status Kod błędu ADS1115
 * @return Opis błędu
 */
const char* ADS1115_GetErrorString(ADS1115_Status status) {
    switch (status) {
        case ADS1115_OK:
            return "OK";
        case ADS1115_ERROR_I2C:
            return "I2C communication error";
        case ADS1115_ERROR_TIMEOUT:
            return "Conversion timeout";
        case ADS1115_ERROR_INIT:
            return "Initialization error - device not found";
        case ADS1115_ERROR_PARAM:
            return "Invalid parameter";
        default:
            return "Unknown error";
    }
}

/**
 * @brief Wypisuje informacje debugowania przez USB CDC
 */
void ADS1115_PrintDebugInfo(void) {
    char buffer[256];
    
    snprintf(buffer, sizeof(buffer), 
        "=== ADS1115 Debug Info ===\r\n"
        "Device detected: %s\r\n"
        "Total reads: %lu\r\n"
        "Successful reads: %lu\r\n"
        "I2C errors: %lu\r\n"
        "Timeouts: %lu\r\n"
        "Last error: %s (code: %lu)\r\n"
        "Last config: 0x%04lX\r\n"
        "Last raw value: %d\r\n"
        "Last voltage: %.4f V\r\n"
        "=======================\r\n",
        ADS1115_Debug.device_detected ? "YES" : "NO",
        ADS1115_Debug.total_reads,
        ADS1115_Debug.successful_reads,
        ADS1115_Debug.i2c_errors,
        ADS1115_Debug.timeouts,
        ADS1115_GetErrorString((ADS1115_Status)ADS1115_Debug.last_error_code),
        ADS1115_Debug.last_error_code,
        ADS1115_Debug.last_config,
        ADS1115_Debug.last_raw_value,
        ADS1115_Debug.last_voltage
    );
    
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}

/**
 * @brief Resetuje liczniki debugowania
 */
void ADS1115_Reset_Debug_Counters(void) {
    ADS1115_Debug.total_reads = 0;
    ADS1115_Debug.successful_reads = 0; 
    ADS1115_Debug.i2c_errors = 0;
    ADS1115_Debug.timeouts = 0;
}