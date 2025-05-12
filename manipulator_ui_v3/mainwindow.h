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

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // Sloty do obsługi przycisków nawigacji
    void on_modelButton_clicked();
    void on_imuButton_clicked();
    void on_anglesButton_clicked();
    void on_valuesButton_clicked();

private:
    Ui::MainWindow *ui;

    void setupCharts();
    void setupIMUCharts();
    void setupAnglesCharts();

    // Wykresy dla IMU (po jednym dla każdej osi)
    QChart *imuChartX;
    QChart *imuChartY;
    QChart *imuChartZ;

    // Serie danych dla IMU - oś X
    QLineSeries *accelSeriesX;
    QLineSeries *gyroSeriesX;
    QLineSeries *magSeriesX;
    QLineSeries *fusSeriesX;

    // Serie danych dla IMU - oś Y
    QLineSeries *accelSeriesY;
    QLineSeries *gyroSeriesY;
    QLineSeries *magSeriesY;
    QLineSeries *fusSeriesY;

    // Serie danych dla IMU - oś Z
    QLineSeries *accelSeriesZ;
    QLineSeries *gyroSeriesZ;
    QLineSeries *magSeriesZ;
    QLineSeries *fusSeriesZ;

    // Wykresy dla kątów
    QVector<QChart*> angleCharts;
    QVector<QLineSeries*> angleSeries;

    // Timer do aktualizacji
    QTimer *updateTimer;

    // Licznik punktów na wykresie
    int dataCounter;
    static const int MAX_POINTS = 50; // maksymalna ilość punktów na wy
};
#endif // MAINWINDOW_H
