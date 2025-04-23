//
// Created by michal on 23.04.25.
//

#ifndef MANIPULATOR6DOF_DEFINITIONS_H
#define MANIPULATOR6DOF_DEFINITIONS_H

#include <QApplication>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QLabel>
#include <QChart>
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <QFont>
#include <QFrame>
#include <QDial>
#include <QPixmap>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QPainter>
#include <QLineEdit>
#include <QDebug>
#include <QDir>
#include <QQueue>

/**
 * @brief Stałe konfiguracyjne dla aplikacji
 */
// Ogólne ustawienia aplikacji
#define APP_TITLE "Manipulator 6 DOF"  ///< Tytuł aplikacji
#define LANG_CODE "EN"                 ///< Domyślny kod języka

// Konfiguracja layoutu
#define LEFT_LAYOUT_WEIGHT 7           ///< Waga lewego panelu w układzie
#define RIGHT_LAYOUT_WEIGHT 3          ///< Waga prawego panelu w układzie

// Ustawienia wykresów
#define NUM_ANGLE_CHARTS 6             ///< Liczba wykresów kątów (potencjometrów)
#define NUM_IMU_CHARTS 3               ///< Liczba wykresów IMU
#define CHART_TIME_RANGE 30            ///< Zakres czasu dla osi X (w sekundach)
#define ANGLE_MAX_VALUE 180            ///< Maksymalna wartość kąta (w stopniach)
#define IMU_VALUE_RANGE 20             ///< Zakres wartości IMU (od -IMU_VALUE_RANGE do +IMU_VALUE_RANGE)

// Ustawienia czcionek
#define TITLE_FONT_SIZE 24             ///< Rozmiar czcionki tytułu
#define LABEL_FONT_SIZE 12             ///< Rozmiar czcionki etykiet
#define LANG_FONT_SIZE 12              ///< Rozmiar czcionki dla oznaczenia języka

// Ustawienia manipulatora
#define MANIPULATOR_WIDTH 350          ///< Szerokość placeholdera manipulatora
#define MANIPULATOR_HEIGHT 350         ///< Wysokość placeholdera manipulatora

/**
 * @brief Tablice z kolorami dla wykresów
 */
const QColor ANGLE_COLORS[] = {
        QColor(255, 0, 0),     ///< Kolor dla wykresu kąta 1 (czerwony)
        QColor(0, 0, 255),     ///< Kolor dla wykresu kąta 2 (niebieski)
        QColor(0, 200, 0),     ///< Kolor dla wykresu kąta 3 (zielony)
        QColor(255, 200, 0),   ///< Kolor dla wykresu kąta 4 (żółty)
        QColor(200, 0, 200),   ///< Kolor dla wykresu kąta 5 (fioletowy)
        QColor(150, 75, 75)    ///< Kolor dla wykresu kąta 6 (brązowy)
};

const QColor IMU_COLORS[] = {
        QColor(255, 0, 0),     ///< Kolor dla Accel (czerwony)
        QColor(0, 200, 0),     ///< Kolor dla Gyro (zielony)
        QColor(0, 0, 255),     ///< Kolor dla Mag (niebieski)
        QColor(0, 0, 0)        ///< Kolor dla Position (czarny)
};

const QString IMU_LABELS[] = {
        "Accel",             ///< Etykieta dla przyspieszenia X
        "Gyro",              ///< Etykieta dla żyroskopu X
        "Mag",               ///< Etykieta dla magnetometru X
        "Position"           ///< Etykieta dla pozycji X
};

/**
 * @brief Etykiety dla osi IMU
 */
const QString IMU_AXIS_LABELS[] = {
        "X axis",              ///< Etykieta dla osi X
        "Y axis",              ///< Etykieta dla osi Y
        "Z axis"               ///< Etykieta dla osi Z
};

/**
 * @brief Struktura przechowująca wartości kątów.
 *
 * Struktura ta zawiera sześć zmiennych typu float, które reprezentują różne kąty.
 * Każdy z tych kątów może być wykorzystywany w różnych obliczeniach związanych
 * z geometrą lub ruchem.
 *
 * @note Kąty są przechowywane w jednostkach miary stopni.
 */
typedef struct {
    float ang_1; /**< Kąt 1 */
    float ang_2; /**< Kąt 2 */
    float ang_3; /**< Kąt 3 */
    float ang_4; /**< Kąt 4 */
    float ang_5; /**< Kąt 5 */
    float ang_6; /**< Kąt 6 */
} angles;

/**
 * @brief Dane z akcelerometru.
 *
 * Zawiera wartości przyspieszenia dla osi X, Y, Z. Jednostką miary jest m/s².
 */
struct accel {
    float x; /**< Przyspieszenie w osi X */
    float y; /**< Przyspieszenie w osi Y */
    float z; /**< Przyspieszenie w osi Z */
};

/**
 * @brief Dane z żyroskopu.
 *
 * Zawiera wartości prędkości kątowej dla osi X, Y, Z. Jednostką miary jest stopień na sekundę (°/s).
 */
struct gyro {
    float x; /**< Prędkość kątowa w osi X */
    float y; /**< Prędkość kątowa w osi Y */
    float z; /**< Prędkość kątowa w osi Z */
};

/**
 * @brief Dane z magnetometru.
 *
 * Zawiera wartości natężenia pola magnetycznego dla osi X, Y, Z. Jednostką miary jest uT (mikrotesla).
 */
struct mag {
    float x; /**< Natężenie pola magnetycznego w osi X */
    float y; /**< Natężenie pola magnetycznego w osi Y */
    float z; /**< Natężenie pola magnetycznego w osi Z */
};

/**
 * @brief Struktura przechowująca dane wejściowe dla manipulatora 6 DOF.
 *
 * Struktura zawiera dane z różnych czujników: kąty, akcelerometr, żyroskop, magnetometr.
 * Dane te są przechowywane w kolejce typu QQueue, umożliwiając organizację i przetwarzanie
 * danych wejściowych w czasie rzeczywistym.
 *
 * @note Kolejka pozwala na przechowywanie historycznych danych i ich późniejsze przetwarzanie.
 */
struct inputData {
    angles angleData;  ///< Dane kątów manipulatora
    accel accelData;   ///< Dane z akcelerometru
    gyro gyroData;     ///< Dane z żyroskopu
    mag magData;       ///< Dane z magnetometru
};



#endif //MANIPULATOR6DOF_DEFINITIONS_H
