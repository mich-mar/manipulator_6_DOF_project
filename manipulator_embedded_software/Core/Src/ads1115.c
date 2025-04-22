#include "ads1115.h"

extern I2C_HandleTypeDef hi2c1; // Dostosuj do swojej konfiguracji I2C

/**
 * @brief Inicjalizacja ADS1115
 */
void ADS1115_Init(void) {
    // Ustawienie domyślnej konfiguracji
    uint16_t config = ADS1115_OS_SINGLE |     // Start pojedynczej konwersji
                     ADS1115_MUX_SINGLE_0 |  // Kanał A0
                     ADS1115_PGA_2_048V |    // Zakres ±2.048V
                     ADS1115_MODE_SINGLESHOT | // Tryb pojedynczego odczytu
                     ADS1115_DR_128SPS |     // 128 próbek na sekundę
                     ADS1115_COMP_QUE_DISABLE; // Wyłączenie komparatora
    
    // Zapisz konfigurację do rejestru CONFIG
    ADS1115_WriteRegister(ADS1115_REG_CONFIG, config);
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
    
    return HAL_I2C_Master_Transmit(&hi2c1, ADS1115_I2C_ADDR << 1, data, 3, HAL_MAX_DELAY);
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
    status = HAL_I2C_Master_Transmit(&hi2c1, ADS1115_I2C_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
    if(status != HAL_OK) return status;
    
    // Odczyt wartości
    status = HAL_I2C_Master_Receive(&hi2c1, ADS1115_I2C_ADDR << 1, data, 2, HAL_MAX_DELAY);
    if(status != HAL_OK) return status;
    
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
    return ADS1115_WriteRegister(ADS1115_REG_CONFIG, config);
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
 * @return Wartość odczytana z rejestru CONVERSION
 */
int16_t ADS1115_Read(void) {
    uint16_t value;
    ADS1115_ReadRegister(ADS1115_REG_CONVERSION, &value);
    return (int16_t)value;
}

/**
 * @brief Odczyt wartości z wybranego kanału w trybie single-ended
 * @param channel Numer kanału (0-3)
 * @return Odczytana wartość
 */
int16_t ADS1115_ReadADC_SingleEnded(uint8_t channel) {
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
                     ADS1115_PGA_2_048V |     // Zakres ±2.048V
                     ADS1115_MODE_SINGLESHOT | // Tryb pojedynczego odczytu
                     ADS1115_DR_128SPS |      // 128 próbek na sekundę
                     ADS1115_COMP_QUE_DISABLE; // Wyłączenie komparatora
    
    // Rozpocznij konwersję
    ADS1115_WriteRegister(ADS1115_REG_CONFIG, config);
    
    // Poczekaj na zakończenie konwersji
    uint8_t ready = 0;
    do {
        HAL_Delay(1); // Krótkie opóźnienie
        ADS1115_IsConversionReady(&ready);
    } while(!ready);
    
    // Odczytaj wynik
    return ADS1115_Read();
}

/**
 * @brief Odczyt wartości z kanału A0 (wejście analogowe 0)
 * @return Odczytana wartość
 */
int16_t ADS1115_ReadADC_A0(void) {
    return ADS1115_ReadADC_SingleEnded(0);
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