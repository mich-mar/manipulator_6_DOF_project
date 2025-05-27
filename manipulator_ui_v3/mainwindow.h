#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QDateTime>
#include <QVector>
#include <QTimer>
#include <QRandomGenerator>
#include <QTranslator>
#include <QSettings>
#include <QMessageBox>
#include <QLibraryInfo>
#include "sensorData.hpp"
#include "dataQueue.hpp"
#include "serialReader.hpp"

/**
 * @file mainwindow.h
 * @brief Główne okno aplikacji do wizualizacji danych z manipulatora
 */
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

/**
 * @brief Klasa głównego okna aplikacji
 *
 * Implementuje interfejs użytkownika oraz logikę wizualizacji danych
 * z czujników manipulatora. Obsługuje wykresy IMU, kątów oraz
 * komunikację szeregową.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Konstruktor głównego okna
     * @param parent Wskaźnik na widget rodzica (domyślnie nullptr)
     */
    MainWindow(QWidget *parent = nullptr);

    /**
     * @brief Destruktor głównego okna
     */
    ~MainWindow();

private slots:
    /**
     * @name Sloty obsługi przycisków nawigacji
     * @{
     */
    void on_modelButton_clicked();    ///< Przejście do widoku modelu 3D
    void on_imuButton_clicked();      ///< Przejście do widoku IMU
    void on_anglesButton_clicked();   ///< Przejście do widoku kątów
    void on_valuesButton_clicked();   ///< Przejście do widoku wartości
    void on_compareButton_clicked();  ///< Przejście do widoku porównania
    void on_plButton_clicked();       ///< Zmiana języka na polski
    void on_engButton_clicked();      ///< Zmiana języka na angielski
    /** @} */

    /**
     * @brief Slot odbierający nowe dane z portu szeregowego
     * @param data Struktura zawierająca odebrane dane z czujników
     */
    void onNewSerialData(const SensorDataPoint& data);

    /**
     * @brief Przetwarza nowe dane z czujników i aktualizuje wykresy
     * @param data Struktura zawierająca dane do przetworzenia
     */
    void processNewSensorData(const SensorDataPoint& data);

private:
    /** @brief Wskaźnik na interfejs użytkownika */
    Ui::MainWindow *ui;

    /** @brief Obiekt obsługujący komunikację szeregową */
    SerialReader *serialReader;

    /** @brief Kolejka przechowująca dane z czujników */
    DataQueue dataQueue;

    /**
     * @brief Zmienia język interfejsu użytkownika
     * @param language Kod języka ("pl" lub "en")
     */
    void switchLanguage(const QString& language);

    /** @brief Obiekt tłumacza */
    QTranslator translator;

    /**
     * @name Funkcje konfiguracji wykresów
     * @{
     */
    void setupCharts();      ///< Konfiguracja wszystkich wykresów
    void setupIMUCharts();   ///< Konfiguracja wykresów IMU
    void setupAnglesCharts(); ///< Konfiguracja wykresów kątów
    /** @} */

    /**
     * @name Wykresy IMU
     * @{
     */
    QChart *imuChartX;  ///< Wykres IMU dla osi X
    QChart *imuChartY;  ///< Wykres IMU dla osi Y
    QChart *imuChartZ;  ///< Wykres IMU dla osi Z
    /** @} */

    /**
     * @name Serie danych IMU - oś X
     * @{
     */
    QLineSeries *accelSeriesX;  ///< Seria danych akcelerometru X
    QLineSeries *gyroSeriesX;   ///< Seria danych żyroskopu X
    QLineSeries *magSeriesX;    ///< Seria danych magnetometru X
    QLineSeries *fusSeriesX;    ///< Seria danych fuzji X
    /** @} */

    /**
     * @name Serie danych IMU - oś Y
     * @{
     */
    QLineSeries *accelSeriesY;  ///< Seria danych akcelerometru Y
    QLineSeries *gyroSeriesY;   ///< Seria danych żyroskopu Y
    QLineSeries *magSeriesY;    ///< Seria danych magnetometru Y
    QLineSeries *fusSeriesY;    ///< Seria danych fuzji Y
    /** @} */

    /**
     * @name Serie danych IMU - oś Z
     * @{
     */
    QLineSeries *accelSeriesZ;  ///< Seria danych akcelerometru Z
    QLineSeries *gyroSeriesZ;   ///< Seria danych żyroskopu Z
    QLineSeries *magSeriesZ;    ///< Seria danych magnetometru Z
    QLineSeries *fusSeriesZ;    ///< Seria danych fuzji Z
    /** @} */

    /**
     * @name Wykresy i serie danych kątów
     * @{
     */
    QVector<QChart*> angleCharts;      ///< Wektor wykresów kątów
    QVector<QLineSeries*> angleSeries; ///< Wektor serii danych kątów
    /** @} */

    /**
     * @name Timery i liczniki
     * @{
     */
    QTimer *updateTimer;      ///< Timer do aktualizacji danych
    QTimer *chartUpdateTimer; ///< Timer do odświeżania wykresów
    int dataCounter;         ///< Licznik punktów na wykresie
    /** @} */

    /** @name Stałe konfiguracyjne
     * @{
     */
    static const int MAX_POINTS = 100;              ///< Maksymalna liczba punktów na wykresie
    static const int CHART_UPDATE_INTERVAL = 50;    ///< Interwał odświeżania wykresów [ms]
    /** @} */
};

#endif // MAINWINDOW_H
