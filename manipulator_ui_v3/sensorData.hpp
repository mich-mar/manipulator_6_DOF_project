#ifndef SENSORDATA_HPP
#define SENSORDATA_HPP

#include <QString>
#include <QDateTime>

/**
 * @brief Struktura przechowująca dane z przetwornika analogowo-cyfrowego
 *
 * Zawiera tablicę 6 wartości odpowiadających kanałom ADC (A0-A5),
 * wykorzystywanych do odczytu pozycji kątowych manipulatora.
 */
struct ADCData {
     double adc[6];  ///< Tablica wartości napięć z kanałów A0 do A5 [V]
};


/**
 * @brief Struktura przechowująca dane z czujnika IMU
 *
 * Zawiera dane z akcelerometru, żyroskopu i magnetometru,
 * oraz wartość temperatury czujnika.
 */
struct IMUData {
    /**
     * @brief Dane z akcelerometru
     * Przyspieszenia liniowe w trzech osiach [m/s²]
     */
    struct {
        double x;  ///< Przyspieszenie w osi X
        double y;  ///< Przyspieszenie w osi Y
        double z;  ///< Przyspieszenie w osi Z
    } accel;

    /**
     * @brief Dane z żyroskopu
     * Prędkości kątowe w trzech osiach [rad/s]
     */
    struct {
        double x;  ///< Prędkość kątowa wokół osi X
        double y;  ///< Prędkość kątowa wokół osi Y
        double z;  ///< Prędkość kątowa wokół osi Z
    } gyro;

    /**
     * @brief Dane z magnetometru
     * Składowe pola magnetycznego w trzech osiach [gauss]
     */
    struct {
        double x;  ///< Składowa X pola magnetycznego
        double y;  ///< Składowa Y pola magnetycznego
        double z;  ///< Składowa Z pola magnetycznego
    } mag;

    double temperature;  ///< Temperatura czujnika IMU [°C]
};

/**
 * @brief Główna struktura przechowująca komplet danych z czujników
 *
 * Zawiera znacznik czasowy pomiaru, identyfikator użytkownika,
 * oraz dane z przetwornika ADC i czujnika IMU.
 */
struct SensorDataPoint {
    QDateTime timestamp;    ///< Znacznik czasowy pomiaru
    QString userLogin;      ///< Login użytkownika wykonującego pomiar
    ADCData adcData;       ///< Dane z przetwornika ADC
    IMUData imuData;       ///< Dane z czujnika IMU
};

#endif // SENSORDATA_HPP
