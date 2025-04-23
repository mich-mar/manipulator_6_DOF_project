#ifndef QTFUNCTIONS_H
#define QTFUNCTIONS_H

#include "definitions.h"

QT_CHARTS_USE_NAMESPACE

/**
 * @brief Funkcja tworząca obrócony tekst w etykiecie
 * @param text Tekst do obrócenia
 * @param font Czcionka dla tekstu
 * @param width Szerokość wynikowego pixmapy
 * @param height Wysokość wynikowej pixmapy
 * @return QPixmap z obróconym tekstem
 */
QPixmap createRotatedText(const QString &text, const QFont &font, int width, int height);

/**
 * @brief Funkcja tworząca wykres kąta
 * @param colorIndex Indeks koloru z tablicy ANGLE_COLORS
 * @return QChartView z przygotowanym wykresem
 */
QChartView* createAngleChart(int colorIndex);

/**
 * @brief Funkcja tworząca wykres IMU
 * @param axisIndex Indeks osi (0-X, 1-Y, 2-Z)
 * @return QChartView z przygotowanym wykresem IMU
 */
QChartView* createIMUChart(int axisIndex);

/**
 * @brief Funkcja tworząca placeholder flagi
 * @param text Tekst na placeholderze flagi
 * @param color Kolor tła flagi
 * @return QPixmap z flagą
 */
QPixmap createFlagPlaceholder(const QString &text, const QColor &color);

/**
 * @brief Funkcja tworząca placeholder manipulatora jeśli brak obrazu
 * @return QPixmap z placeholderem
 */
QPixmap createManipulatorPlaceholder();

#endif // QTFUNCTIONS_H
