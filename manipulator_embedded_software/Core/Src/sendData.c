#include "sendData.h"

void printAllSensors(ADS1115_Readings *hadc, GY85_HandleTypeDef *hgy85, OutputFormat format) {
    static char buffer[512]; // Stały bufor
    memset(buffer, 0, sizeof(buffer));
    int length = 0;
    
    // Odczyt ADS1115
    ADS1115_ReadAllValues(hadc);
    HAL_Delay(100);
    
    // Odczyt GY85
    GY85_ReadAllSensors(hgy85);
    HAL_Delay(100);
    
    if (format == FORMAT_HUMAN_READABLE) {
        sprintf(buffer,
            "===============================================\r\n"
            "JOINT_0: %.2f [°]\n"
            "JOINT_1: %.2f [°]\r\n"
            "JOINT_2: %.2f [°]\r\n"
            "JOINT_3: %.2f [°]\r\n"
            "JOINT_4: %.2f [°]\r\n"
            "JOINT_5: %.2f [°]\r\n"
            "ACC: X=%.2f, Y=%.2f, Z=%.2f [m/s^2]\r\n"
            "GYRO: X=%.2f, Y=%.2f, Z=%.2f [rad/s]\r\n"
            "MAG: X=%.2f, Y=%.2f, Z=%.2f [gauss]\r\n"
            "Temp: %.2f [C]\r\n"
            "===============================================\r\n",
            hadc->joint_angle_0,
            hadc->joint_angle_1,
            hadc->joint_angle_2,
            hadc->joint_angle_3,
            hadc->joint_angle_4,
            hadc->joint_angle_5,
            hgy85->accel.x, hgy85->accel.y, hgy85->accel.z,
            hgy85->gyro.x, hgy85->gyro.y, hgy85->gyro.z,
            hgy85->mag.x, hgy85->mag.y, hgy85->mag.z,
            hgy85->temperature
        );
        sendUSBmsg(buffer);
    } 
    else if (format == FORMAT_CSV) {
        // Formatowanie danych CSV
        length = sprintf(buffer,
            "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f",
            hadc->joint_angle_0,
            hadc->joint_angle_1,
            hadc->joint_angle_2,
            hadc->joint_angle_3,
            hadc->joint_angle_4,
            hadc->joint_angle_5,
            hgy85->accel.x, hgy85->accel.y, hgy85->accel.z,
            hgy85->gyro.x, hgy85->gyro.y, hgy85->gyro.z,
            hgy85->mag.x, hgy85->mag.y, hgy85->mag.z,
            hgy85->temperature
        );
        
        // Obliczenie CRC8 dla sformatowanych danych
        uint8_t crc = computeCRC8((uint8_t*)buffer, length, CRC8_POLYNOMIAL, CRC8_INIT);
        
        // Dodanie CRC8 na końcu wiadomości, przed znakami końca linii
        length += sprintf(buffer + length, "*%02X\r\n", crc);
        
        sendUSBmsg(buffer);
    }
}