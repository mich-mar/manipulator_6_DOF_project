#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->setWindowTitle("Manipulator 6 DOF");

    // Inicjalizacja licznika punktów
    dataCounter = 0;

    // Konfiguracja wykresów - dodaj to!
    setupCharts();

    // Ustawienie początkowej strony
    ui->stackedWidget->setCurrentWidget(ui->modelPage);

    // Połączenie przycisków z odpowiednimi stronami
    connect(ui->modelButton, &QPushButton::clicked, this, &MainWindow::on_modelButton_clicked);
    connect(ui->imuButton, &QPushButton::clicked, this, &MainWindow::on_imuButton_clicked);
    connect(ui->anglesButton, &QPushButton::clicked, this, &MainWindow::on_anglesButton_clicked);
    connect(ui->valuesButton, &QPushButton::clicked, this, &MainWindow::on_valuesButton_clicked);
    connect(ui->compareButton, &QPushButton::clicked, this, &MainWindow::on_compareButton_clicked);
}

// Implementacja slotów do przełączania stron
void MainWindow::on_modelButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->modelPage);
}

void MainWindow::on_imuButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->imuPage);
}

void MainWindow::on_anglesButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->anglesPage);
}

void MainWindow::on_valuesButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->valuesPage);
}

void MainWindow::on_compareButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->comparePage);
}

void MainWindow::setupCharts()
{
    setupIMUCharts();
    setupAnglesCharts();

    // Timer do aktualizacji wykresów
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, [this]() {
        // Dodawanie nowych punktów
        float time = dataCounter++;

        // Przykładowe dane (zastąp własnymi)
        float accelX = QRandomGenerator::global()->bounded(100);
        float gyroX = QRandomGenerator::global()->bounded(100);
        float magX = QRandomGenerator::global()->bounded(100);
        float fusX = QRandomGenerator::global()->bounded(100);

        // Dodawanie punktów do serii
        accelSeriesX->append(time, accelX);
        gyroSeriesX->append(time, gyroX);
        magSeriesX->append(time, magX);
        fusSeriesX->append(time, fusX);

        // Usuwanie starych punktów jeśli przekroczono limit
        if (accelSeriesX->count() > MAX_POINTS) {
            accelSeriesX->remove(0);
            gyroSeriesX->remove(0);
            magSeriesX->remove(0);
            fusSeriesX->remove(0);
        }

        // Aktualizacja zakresu osi X
        for (QChart* chart : {imuChartX, imuChartY, imuChartZ}) {
            QValueAxis *axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
            if (dataCounter > MAX_POINTS) {
                axisX->setRange(dataCounter - MAX_POINTS, dataCounter);
            }
        }

        // Podobnie dla pozostałych osi...

        // Aktualizacja wartości w labelach
        ui->accValX->setText(QString::number(accelX));
        ui->gyroValX->setText(QString::number(gyroX));
        ui->magValX->setText(QString::number(magX));
        ui->fusValX->setText(QString::number(fusX));
        // ... podobnie dla pozostałych wartości
    });

    updateTimer->start(1000); // aktualizacja co sekundę
}

void MainWindow::setupIMUCharts()
{
    // Tworzenie wykresu dla osi X
    imuChartX = new QChart();
    accelSeriesX = new QLineSeries();
    gyroSeriesX = new QLineSeries();
    magSeriesX = new QLineSeries();
    fusSeriesX = new QLineSeries();

    accelSeriesX->setName("Accelerometer");
    gyroSeriesX->setName("Gyroscope");
    magSeriesX->setName("Magnetometer");
    fusSeriesX->setName("Fusion");

    imuChartX->addSeries(accelSeriesX);
    imuChartX->addSeries(gyroSeriesX);
    imuChartX->addSeries(magSeriesX);
    imuChartX->addSeries(fusSeriesX);

    // Osie dla X
    QValueAxis *axisX_X = new QValueAxis();
    QValueAxis *axisY_X = new QValueAxis();
    axisX_X->setRange(0, MAX_POINTS);
    axisY_X->setRange(-100, 100);

    imuChartX->addAxis(axisX_X, Qt::AlignBottom);
    imuChartX->addAxis(axisY_X, Qt::AlignLeft);

    accelSeriesX->attachAxis(axisX_X);
    accelSeriesX->attachAxis(axisY_X);
    gyroSeriesX->attachAxis(axisX_X);
    gyroSeriesX->attachAxis(axisY_X);
    magSeriesX->attachAxis(axisX_X);
    magSeriesX->attachAxis(axisY_X);
    fusSeriesX->attachAxis(axisX_X);
    fusSeriesX->attachAxis(axisY_X);

    // Style dla X
    imuChartX->setTheme(QChart::ChartThemeBlueCerulean);
    imuChartX->legend()->setAlignment(Qt::AlignRight);

    // Utworzenie widoku wykresu X
    QChartView *chartViewX = new QChartView(imuChartX);
    chartViewX->setRenderHint(QPainter::Antialiasing);
    chartViewX->setMinimumSize(300, 200);

    // Dodanie do layoutu X
    if (ui->imuChartX->layout()) {
        delete ui->imuChartX->layout();
    }
    QVBoxLayout *layoutX = new QVBoxLayout(ui->imuChartX);
    layoutX->setContentsMargins(0, 0, 0, 0);
    layoutX->addWidget(chartViewX);

    // Tworzenie wykresu dla osi Y
    imuChartY = new QChart();
    accelSeriesY = new QLineSeries();
    gyroSeriesY = new QLineSeries();
    magSeriesY = new QLineSeries();
    fusSeriesY = new QLineSeries();

    accelSeriesY->setName("Accelerometer");
    gyroSeriesY->setName("Gyroscope");
    magSeriesY->setName("Magnetometer");
    fusSeriesY->setName("Fusion");

    imuChartY->addSeries(accelSeriesY);
    imuChartY->addSeries(gyroSeriesY);
    imuChartY->addSeries(magSeriesY);
    imuChartY->addSeries(fusSeriesY);

    // Osie dla Y
    QValueAxis *axisX_Y = new QValueAxis();
    QValueAxis *axisY_Y = new QValueAxis();
    axisX_Y->setRange(0, MAX_POINTS);
    axisY_Y->setRange(-100, 100);

    imuChartY->addAxis(axisX_Y, Qt::AlignBottom);
    imuChartY->addAxis(axisY_Y, Qt::AlignLeft);

    accelSeriesY->attachAxis(axisX_Y);
    accelSeriesY->attachAxis(axisY_Y);
    gyroSeriesY->attachAxis(axisX_Y);
    gyroSeriesY->attachAxis(axisY_Y);
    magSeriesY->attachAxis(axisX_Y);
    magSeriesY->attachAxis(axisY_Y);
    fusSeriesY->attachAxis(axisX_Y);
    fusSeriesY->attachAxis(axisY_Y);

    // Style dla Y
    imuChartY->setTheme(QChart::ChartThemeBlueCerulean);
    imuChartY->legend()->setAlignment(Qt::AlignRight);

    // Utworzenie widoku wykresu Y
    QChartView *chartViewY = new QChartView(imuChartY);
    chartViewY->setRenderHint(QPainter::Antialiasing);
    chartViewY->setMinimumSize(300, 200);

    // Dodanie do layoutu Y
    if (ui->imuChartY->layout()) {
        delete ui->imuChartY->layout();
    }
    QVBoxLayout *layoutY = new QVBoxLayout(ui->imuChartY);
    layoutY->setContentsMargins(0, 0, 0, 0);
    layoutY->addWidget(chartViewY);

    // Tworzenie wykresu dla osi Z
    imuChartZ = new QChart();
    accelSeriesZ = new QLineSeries();
    gyroSeriesZ = new QLineSeries();
    magSeriesZ = new QLineSeries();
    fusSeriesZ = new QLineSeries();

    accelSeriesZ->setName("Accelerometer");
    gyroSeriesZ->setName("Gyroscope");
    magSeriesZ->setName("Magnetometer");
    fusSeriesZ->setName("Fusion");

    imuChartZ->addSeries(accelSeriesZ);
    imuChartZ->addSeries(gyroSeriesZ);
    imuChartZ->addSeries(magSeriesZ);
    imuChartZ->addSeries(fusSeriesZ);

    // Osie dla Z
    QValueAxis *axisX_Z = new QValueAxis();
    QValueAxis *axisY_Z = new QValueAxis();
    axisX_Z->setRange(0, MAX_POINTS);
    axisY_Z->setRange(-100, 100);

    imuChartZ->addAxis(axisX_Z, Qt::AlignBottom);
    imuChartZ->addAxis(axisY_Z, Qt::AlignLeft);

    accelSeriesZ->attachAxis(axisX_Z);
    accelSeriesZ->attachAxis(axisY_Z);
    gyroSeriesZ->attachAxis(axisX_Z);
    gyroSeriesZ->attachAxis(axisY_Z);
    magSeriesZ->attachAxis(axisX_Z);
    magSeriesZ->attachAxis(axisY_Z);
    fusSeriesZ->attachAxis(axisX_Z);
    fusSeriesZ->attachAxis(axisY_Z);

    // Style dla Z
    imuChartZ->setTheme(QChart::ChartThemeBlueCerulean);
    imuChartZ->legend()->setAlignment(Qt::AlignRight);

    // Utworzenie widoku wykresu Z
    QChartView *chartViewZ = new QChartView(imuChartZ);
    chartViewZ->setRenderHint(QPainter::Antialiasing);
    chartViewZ->setMinimumSize(300, 200);

    // Dodanie do layoutu Z
    if (ui->imuChartZ->layout()) {
        delete ui->imuChartZ->layout();
    }
    QVBoxLayout *layoutZ = new QVBoxLayout(ui->imuChartZ);
    layoutZ->setContentsMargins(0, 0, 0, 0);
    layoutZ->addWidget(chartViewZ);
}

void MainWindow::setupAnglesCharts()
{
    for (int i = 0; i < 6; ++i) {
        QChart *chart = new QChart();
        QLineSeries *series = new QLineSeries();

        series->setName(QString("Angle %1").arg(i+1));
        chart->addSeries(series);

        QValueAxis *axisX = new QValueAxis();
        QValueAxis *axisY = new QValueAxis();
        axisX->setRange(0, MAX_POINTS);
        axisY->setRange(-180, 180);

        chart->addAxis(axisX, Qt::AlignBottom);
        chart->addAxis(axisY, Qt::AlignLeft);

        series->attachAxis(axisX);
        series->attachAxis(axisY);

        chart->setTheme(QChart::ChartThemeBlueCerulean);
        chart->legend()->setVisible(false);

        QChartView *chartView = new QChartView(chart);
        chartView->setRenderHint(QPainter::Antialiasing);

        // Dodanie do odpowiedniego widgetu
        QLabel *chartLabel = findChild<QLabel*>(QString("angChart%1").arg(i+1));
        if (chartLabel) {
            chartLabel->setLayout(new QVBoxLayout);
            chartLabel->layout()->addWidget(chartView);
        }

        angleCharts.append(chart);
        angleSeries.append(series);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
