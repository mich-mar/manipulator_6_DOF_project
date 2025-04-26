#include "sendData.h"

void ReadAllSensors(ADS1115_Readings *hadc, GY85_HandleTypeDef *hgy85, OutputFormat format) {
    static char buffer[512]; // Stały bufor
    memset(buffer, 0, sizeof(buffer));

    // Odczyt ADS1115
    ADS1115_ReadAllValues(hadc);
    HAL_Delay(100);

    // Odczyt GY85
    GY85_ReadAllSensors(hgy85);
    HAL_Delay(100);

    if (format == FORMAT_HUMAN_READABLE) {
        sprintf(buffer,
            "===============================================\r\n"
            "ADC_A0: %.2f [V]\n"
            "ADC_A1: %.2f [V]\r\n"
            "ADC_A2: %.2f [V]\r\n"
            "ADC_A3: %.2f [V]\r\n"
            "ADC_A4: %.2f [V]\r\n"
            "ADC_A5: %.2f [V]\r\n"
            "ACC: X=%.2f, Y=%.2f, Z=%.2f [m/s^2]\r\n"
            "GYRO: X=%.2f, Y=%.2f, Z=%.2f [rad/s]\r\n"
            "MAG: X=%.2f, Y=%.2f, Z=%.2f [gauss]\r\n"
            "Temp: %.2f [C]\r\n"
            "===============================================\r\n",
            hadc->A0_adc,
            hadc->A1_adc,
            hadc->A2_adc,
            hadc->A3_adc,
            hadc->A4_adc,
            hadc->A5_adc,
            hgy85->accel.x, hgy85->accel.y, hgy85->accel.z,
            hgy85->gyro.x,  hgy85->gyro.y,  hgy85->gyro.z,
            hgy85->mag.x,   hgy85->mag.y,   hgy85->mag.z,
            hgy85->temperature
        );
    } else if (format == FORMAT_CSV) {
        sprintf(buffer,
            "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\r\n",
            hadc->A0_adc,
            hadc->A1_adc,
            hadc->A2_adc,
            hadc->A3_adc,
            hadc->A4_adc,
            hadc->A5_adc,
            hgy85->accel.x, hgy85->accel.y, hgy85->accel.z,
            hgy85->gyro.x,  hgy85->gyro.y,  hgy85->gyro.z,
            hgy85->mag.x,   hgy85->mag.y,   hgy85->mag.z,
            hgy85->temperature
        );
    }

    sendUSBmsg(buffer);
}
