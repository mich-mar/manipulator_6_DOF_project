#include "mainwindow.h"
#include "ui_mainwindow.h"

/**
 * @brief Konstruktor klasy MainWindow
 *
 * Inicjalizuje interfejs użytkownika, konfiguruje wykresy i komunikację szeregową.
 * Ustawia domyślny język aplikacji na angielski i łączy sygnały z odpowiednimi slotami.
 *
 * @param parent Wskaźnik na widget rodzica
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    QTranslator* qtTranslator = new QTranslator(this);
    if (qtTranslator->load("qt_" + QLocale::system().name(),
                           QLibraryInfo::path(QLibraryInfo::TranslationsPath))) {
        qApp->installTranslator(qtTranslator);
    }

    ui->setupUi(this);

    // Ustaw domyślnie język angielski
    switchLanguage("en");

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

    // Połączenie przycisków ze zmianą języka
    connect(ui->plButton, &QPushButton::clicked, this, &MainWindow::on_plButton_clicked);
    connect(ui->engButton, &QPushButton::clicked, this, &MainWindow::on_engButton_clicked);

    // Inicjalizacja SerialReader
    serialReader = new SerialReader(this);
    connect(serialReader, &SerialReader::newDataAvailable,
            this, &MainWindow::onNewSerialData);

    // Próba otwarcia portu
    if (!serialReader->findAndOpenPort()) {
        QMessageBox::warning(this, "Błąd", "Nie można znaleźć portu szeregowego!");
    }
}

/**
 * @brief Slot obsługujący przejście do strony modelu 3D
 */
void MainWindow::on_modelButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->modelPage);
}

/**
 * @brief Slot obsługujący przejście do strony IMU
 */
void MainWindow::on_imuButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->imuPage);
}

/**
 * @brief Slot obsługujący przejście do strony kątów
 */
void MainWindow::on_anglesButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->anglesPage);
}

/**
 * @brief Slot obsługujący przejście do strony wartości
 */
void MainWindow::on_valuesButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->valuesPage);
}

/**
 * @brief Slot obsługujący przejście do strony porównującej kąty
 */
void MainWindow::on_compareButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->comparePage);
}

/**
 * @brief Zmienia język interfejsu użytkownika
 *
 * Funkcja usuwa poprzednie tłumaczenie, ładuje nowe i aktualizuje
 * wszystkie elementy interfejsu użytkownika, w tym:
 * - Przyciski nawigacyjne
 * - Etykiety wykresów
 * - Opisy osi
 * - Etykiety wartości
 *
 * @param language Kod języka ("pl" lub "en")
 */
// @todo: napraw ta funkcje
void MainWindow::switchLanguage(const QString& language)
{
    // Usuń poprzednie tłumaczenie
    qApp->removeTranslator(&translator);

    // Próba załadowania nowego tłumaczenia
    bool loaded = translator.load(":/translations/translations/manipulator_" + language + ".qm");

    if (loaded) {
        bool installed = qApp->installTranslator(&translator);
    } else {
        return;
    }

    // Aktualizuj interfejs użytkownika
    ui->retranslateUi(this);

    // Przyciski nawigacyjne (używaj dokładnie takich samych tekstów jak w pliku .ts)
    ui->modelButton->setText(tr("3D Model"));
    ui->imuButton->setText(tr("IMU"));
    ui->anglesButton->setText(tr("Angles"));
    ui->valuesButton->setText(tr("Values"));
    ui->compareButton->setText(tr("Compare"));

    // Tytuł strony modelu 3D
    ui->label->setText(tr("MANIPULATOR 3D MODEL"));

    // Tytuł strony IMU
    ui->imuPageLabel->setText(tr("IMU VALUES"));

    // Legendy wykresów IMU
    if (accelSeriesX) {
        // Wykresy IMU powinny używać tr()
        QString accelerometer = tr("Accelerometer");
        QString gyroscope = tr("Gyroscope");
        QString magnetometer = tr("Magnetometer");
        QString fusion = tr("Fusion");

        // Oś X
        accelSeriesX->setName(accelerometer);
        gyroSeriesX->setName(gyroscope);
        magSeriesX->setName(magnetometer);
        fusSeriesX->setName(fusion);

        // Oś Y
        accelSeriesY->setName(accelerometer);
        gyroSeriesY->setName(gyroscope);
        magSeriesY->setName(magnetometer);
        fusSeriesY->setName(fusion);

        // Oś Z
        accelSeriesZ->setName(accelerometer);
        gyroSeriesZ->setName(gyroscope);
        magSeriesZ->setName(magnetometer);
        fusSeriesZ->setName(fusion);
    }

    // Nazwy osi
    ui->imuChartX->setText(tr("X axis"));
    ui->imuChartY->setText(tr("Y axis"));
    ui->imuChartZ->setText(tr("Z axis"));

    // Etykiety kątów
    ui->angLabel1->setText(tr("ANG_1"));
    ui->angLabel2->setText(tr("ANG_2"));
    ui->angLabel3->setText(tr("ANG_3"));
    ui->angLabel4->setText(tr("ANG_4"));
    ui->angLabel5->setText(tr("ANG_5"));
    ui->angLabel6->setText(tr("ANG_6"));

    // Etykiety IMU
    ui->accelLabelX->setText(tr("ACC_X"));
    ui->accelLabelY->setText(tr("ACC_Y"));
    ui->accelLabelZ->setText(tr("ACC_Z"));

    ui->gyroLabelX->setText(tr("GYRO_X"));
    ui->gyroLabelY->setText(tr("GYRO_Y"));
    ui->gyroLabelZ->setText(tr("GYRO_Z"));

    ui->magLabelX->setText(tr("MAG_X"));
    ui->magLabelY->setText(tr("MAG_Y"));
    ui->magLabelZ->setText(tr("MAG_Z"));

    ui->fusLabelX->setText(tr("FUS_X"));
    ui->fusLabelY->setText(tr("FUS_Y"));
    ui->fusLabelZ->setText(tr("FUS_Z"));

    // Etykiety w zakładce Compare
    ui->imuAnglesLabel->setText(tr("IMU ANGLES"));
    ui->potAnglesLabel->setText(tr("POTENTIOMETRS ANGLES"));

    // Etykiety przegubów
    for (int i = 1; i <= 6; ++i) {
        // IMU joints
        QLabel* imuJoint = findChild<QLabel*>(QString("imuJointLabel%1").arg(i));
        if (imuJoint) {
            // Użyj bezpośrednio sformatowanego stringa dla klucza tłumaczenia
            QString key = QString("JOINT_%1").arg(i);
            QString translatedText = tr(key.toLatin1().constData());
            imuJoint->setText(translatedText);
        }

        // Potentiometer joints
        QLabel* potJoint = findChild<QLabel*>(QString("potJointLabel%1").arg(i));
        if (potJoint) {
            QString key = QString("JOINT_%1").arg(i);
            QString translatedText = tr(key.toLatin1().constData());
            potJoint->setText(translatedText);
        }
    }

    // Odśwież wykresy
    if (imuChartX) {
        imuChartX->update();
        imuChartY->update();
        imuChartZ->update();
    }

    for (QChart* chart : angleCharts) {
        if (chart) {
            chart->update();
        }
    }
}

/**
 * @brief Slot obsługujący zmianę języka na polski
 */
void MainWindow::on_plButton_clicked()
{
    switchLanguage("pl");
}

/**
 * @brief Slot obsługujący zmianę języka na angielski
 */
void MainWindow::on_engButton_clicked()
{
    switchLanguage("en");
}

/**
 * @brief Konfiguruje wszystkie wykresy w aplikacji
 *
 * Inicjalizuje wykresy IMU i kątów poprzez wywołanie
 * odpowiednich funkcji konfiguracyjnych.
 */
void MainWindow::setupCharts()
{
    setupIMUCharts();
    setupAnglesCharts();
}

/**
 * @brief Konfiguruje wykresy IMU
 *
 * Tworzy i konfiguruje wykresy dla wszystkich osi (X, Y, Z) IMU.
 * Dla każdej osi tworzone są cztery serie danych:
 * - Akcelerometr
 * - Żyroskop
 * - Magnetometr
 * - Fuzja danych
 */
void MainWindow::setupIMUCharts()
{
    // Common settings for all IMU charts
    auto setupChart = [](QChart* chart) {
        chart->setBackgroundBrush(Qt::white);
        chart->setMargins(QMargins(0, 0, 0, 0));
        chart->legend()->setAlignment(Qt::AlignRight);
        chart->setTheme(QChart::ChartThemeLight);
    };

    auto setupAxis = [](QValueAxis* axis) {
        axis->setGridLineVisible(false);
        axis->setMinorGridLineVisible(false);
        axis->setLabelsColor(Qt::black);
    };

    auto setupChartView = [](QChartView* chartView) {
        chartView->setRenderHint(QPainter::Antialiasing);
        chartView->setMinimumSize(300, 200);
        chartView->setBackgroundBrush(QColor("#2F3542"));
        // Add rounded corners to the chart view
        chartView->setRenderHint(QPainter::Antialiasing);
        chartView->setStyleSheet("background-color: #2F3542; border-radius: 10px;");
    };

    // Setup for X axis chart
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

    QValueAxis *axisX_X = new QValueAxis();
    QValueAxis *axisY_X = new QValueAxis();
    axisX_X->setRange(0, MAX_POINTS);
    axisY_X->setRange(-100, 100);

    setupAxis(axisX_X);
    setupAxis(axisY_X);

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

    setupChart(imuChartX);

    QChartView *chartViewX = new QChartView(imuChartX);
    setupChartView(chartViewX);

    if (ui->imuChartX->layout()) {
        delete ui->imuChartX->layout();
    }
    QVBoxLayout *layoutX = new QVBoxLayout(ui->imuChartX);
    layoutX->setContentsMargins(0, 0, 0, 0);
    layoutX->addWidget(chartViewX);

    // Setup for Y axis chart
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

    QValueAxis *axisX_Y = new QValueAxis();
    QValueAxis *axisY_Y = new QValueAxis();
    axisX_Y->setRange(0, MAX_POINTS);
    axisY_Y->setRange(-100, 100);

    setupAxis(axisX_Y);
    setupAxis(axisY_Y);

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

    setupChart(imuChartY);

    QChartView *chartViewY = new QChartView(imuChartY);
    setupChartView(chartViewY);

    if (ui->imuChartY->layout()) {
        delete ui->imuChartY->layout();
    }
    QVBoxLayout *layoutY = new QVBoxLayout(ui->imuChartY);
    layoutY->setContentsMargins(0, 0, 0, 0);
    layoutY->addWidget(chartViewY);

    // Setup for Z axis chart
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

    QValueAxis *axisX_Z = new QValueAxis();
    QValueAxis *axisY_Z = new QValueAxis();
    axisX_Z->setRange(0, MAX_POINTS);
    axisY_Z->setRange(-100, 100);

    setupAxis(axisX_Z);
    setupAxis(axisY_Z);

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

    setupChart(imuChartZ);

    QChartView *chartViewZ = new QChartView(imuChartZ);
    setupChartView(chartViewZ);

    if (ui->imuChartZ->layout()) {
        delete ui->imuChartZ->layout();
    }
    QVBoxLayout *layoutZ = new QVBoxLayout(ui->imuChartZ);
    layoutZ->setContentsMargins(0, 0, 0, 0);
    layoutZ->addWidget(chartViewZ);
}


/**
 * @brief Konfiguruje wykresy kątów
 *
 * Tworzy i konfiguruje wykresy dla wszystkich sześciu kątów manipulatora.
 * Każdy wykres zawiera jedną serię danych reprezentującą wartość kąta
 * w zakresie od -180° do 180°.
 */
void MainWindow::setupAnglesCharts()
{
    auto setupChart = [](QChart* chart) {
        chart->setBackgroundBrush(Qt::white);
        chart->setMargins(QMargins(0, 0, 0, 0));
        chart->legend()->setVisible(false);
        chart->setTheme(QChart::ChartThemeLight);
    };

    auto setupAxis = [](QValueAxis* axis) {
        axis->setGridLineVisible(false);
        axis->setMinorGridLineVisible(false);
        axis->setLabelsColor(Qt::black);
    };

    auto setupChartView = [](QChartView* chartView) {
        chartView->setRenderHint(QPainter::Antialiasing);
        chartView->setMinimumSize(100, 100);
        chartView->setBackgroundBrush(QColor("#2F3542"));
        // Add rounded corners to the chart view
        chartView->setRenderHint(QPainter::Antialiasing);
        chartView->setStyleSheet("background-color: #2F3542; border-radius: 10px;");
    };

    for (int i = 0; i < 6; ++i) {
        QChart *chart = new QChart();
        QLineSeries *series = new QLineSeries();

        series->setName(QString("Angle %1").arg(i+1));
        chart->addSeries(series);

        QValueAxis *axisX = new QValueAxis();
        QValueAxis *axisY = new QValueAxis();
        axisX->setRange(0, MAX_POINTS);
        axisY->setRange(-150, 150);

        setupAxis(axisX);
        setupAxis(axisY);

        chart->addAxis(axisX, Qt::AlignBottom);
        chart->addAxis(axisY, Qt::AlignLeft);

        series->attachAxis(axisX);
        series->attachAxis(axisY);

        setupChart(chart);

        QChartView *chartView = new QChartView(chart);
        setupChartView(chartView);

        QLabel *chartLabel = findChild<QLabel*>(QString("angChart%1").arg(i+1));
        if (chartLabel) {
            if (chartLabel->layout()) {
                delete chartLabel->layout();
            }
            QVBoxLayout *layout = new QVBoxLayout(chartLabel);
            layout->setContentsMargins(0, 0, 0, 0);
            layout->addWidget(chartView);
        }

        angleCharts.append(chart);
        angleSeries.append(series);
    }
}

/**
 * @brief Slot odbierający nowe dane z portu szeregowego
 *
 * @param data Struktura zawierająca odebrane dane z czujników
 */
void MainWindow::onNewSerialData(const SensorDataPoint& data)
{
    qDebug() << "Otrzymano nowe dane w MainWindow";
    processNewSensorData(data);
}

/**
 * @brief Przetwarza nowe dane z czujników
 *
 * Funkcja aktualizuje:
 * - Wykresy IMU (akcelerometr, żyroskop, magnetometr)
 * - Wykresy kątów
 * - Etykiety z wartościami
 * - Wizualizacje potencjometrów
 *
 * Dodatkowo zarządza buforowaniem danych i usuwa stare punkty
 * z wykresów gdy przekroczony zostanie limit.
 *
 * @param data Struktura zawierająca dane z czujników do przetworzenia
 */
void MainWindow::processNewSensorData(const SensorDataPoint& data)
{
    // Dodaj dane do kolejki
    dataQueue.enqueue(data);
    float time = dataCounter++;

    // Aktualizuj wykresy IMU
    // Akcelerometr
    accelSeriesX->append(time, data.imuData.accel.x);
    accelSeriesY->append(time, data.imuData.accel.y);
    accelSeriesZ->append(time, data.imuData.accel.z);

    // Żyroskop
    gyroSeriesX->append(time, data.imuData.gyro.x);
    gyroSeriesY->append(time, data.imuData.gyro.y);
    gyroSeriesZ->append(time, data.imuData.gyro.z);

    // Magnetometr
    magSeriesX->append(time, data.imuData.mag.x);
    magSeriesY->append(time, data.imuData.mag.y);
    magSeriesZ->append(time, data.imuData.mag.z);

    // Aktualizuj wykresy kątów (ADC)
    for (int i = 0; i < 6; ++i) {
        if (i < angleSeries.size()) {
            // Konwersja napięcia na kąt (przykładowe mapowanie, dostosuj według potrzeb)
            // Załóżmy, że 0V = -180°, 5V = 180°
            double angle = (data.adcData.adc[i]);
            angleSeries[i]->append(time, angle);

            // Usuń stare punkty jeśli przekroczono limit
            if (angleSeries[i]->count() > MAX_POINTS) {
                angleSeries[i]->remove(0);
            }
        }
    }

    // Usuń stare punkty jeśli przekroczono limit dla wykresów IMU
    if (accelSeriesX->count() > MAX_POINTS) {
        // IMU series X
        accelSeriesX->remove(0);
        gyroSeriesX->remove(0);
        magSeriesX->remove(0);

        // IMU series Y
        accelSeriesY->remove(0);
        gyroSeriesY->remove(0);
        magSeriesY->remove(0);

        // IMU series Z
        accelSeriesZ->remove(0);
        gyroSeriesZ->remove(0);
        magSeriesZ->remove(0);
    }

    // Aktualizuj zakresy osi X
    if (dataCounter > MAX_POINTS) {
        float minX = dataCounter - MAX_POINTS;
        float maxX = dataCounter;

        // IMU Charts
        for (QChart* chart : {imuChartX, imuChartY, imuChartZ}) {
            QValueAxis* axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
            if (axisX) {
                axisX->setRange(minX, maxX);
            }
        }

        // Angles Charts
        for (QChart* chart : angleCharts) {
            QValueAxis* axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
            if (axisX) {
                axisX->setRange(minX, maxX);
            }
        }
    }

    // Aktualizuj wartości ADC w labelach
    for (int i = 0; i < 6; i++) {
        // ADC value labels
        QLabel* adcLabel = findChild<QLabel*>(QString("adcVal%1").arg(i));
        if (adcLabel) {
            adcLabel->setText(QString::number(data.adcData.adc[i], 'f', 2) + " V");
        }

        // Angle value labels (converted from ADC)
        QLabel* angleLabel = findChild<QLabel*>(QString("angVal%1").arg(i + 1));
        if (angleLabel) {
            double angle = data.adcData.adc[i];
            angleLabel->setText(QString::number(angle, 'f', 2) + "°");
        }
    }

    // IMU Values
    ui->accValX->setText(QString::number(data.imuData.accel.x, 'f', 2) + " m/s²");
    ui->accValY->setText(QString::number(data.imuData.accel.y, 'f', 2) + " m/s²");
    ui->accValZ->setText(QString::number(data.imuData.accel.z, 'f', 2) + " m/s²");

    ui->gyroValX->setText(QString::number(data.imuData.gyro.x, 'f', 2) + " rad/s");
    ui->gyroValY->setText(QString::number(data.imuData.gyro.y, 'f', 2) + " rad/s");
    ui->gyroValZ->setText(QString::number(data.imuData.gyro.z, 'f', 2) + " rad/s");

    ui->magValX->setText(QString::number(data.imuData.mag.x, 'f', 2) + " gauss");
    ui->magValY->setText(QString::number(data.imuData.mag.y, 'f', 2) + " gauss");
    ui->magValZ->setText(QString::number(data.imuData.mag.z, 'f', 2) + " gauss");

    // Odśwież wykresy
    imuChartX->update();
    imuChartY->update();
    imuChartZ->update();
    for (QChart* chart : angleCharts) {
        chart->update();
    }

    // Aktualizacja potencjometrów
    for (int i = 0; i < 6; i++) {
        // Znajdź potencjometr przez nazwę
        QDial* potentiometer = findChild<QDial*>(QString("dialAngle%1").arg(i + 1));
        if (potentiometer) {
            int angle = static_cast<int>(data.adcData.adc[i]);

            // Ustaw wartość potencjometru
            potentiometer->setValue(angle);

            // Opcjonalnie: dodaj tooltip z dokładną wartością
            potentiometer->setToolTip(QString("Voltage: %1 V\nAngle: %2°")
                                          .arg(data.adcData.adc[i], 0, 'f', 2)
                                          .arg(angle));
        }
    }

    // Aktualizuj wartości kątów w labelach w porywnaniu z kinematyką
    for (int i = 0; i < 6; i++) {
        // Oblicz kąt
        double angle = data.adcData.adc[i];

        // Dodaj aktualizację dla imuJointValue
        QLabel* imuJointLabel = findChild<QLabel*>(QString("imuJointValue%1").arg(i + 1));
        if (imuJointLabel) {
            imuJointLabel->setText(QString::number(angle, 'f', 2) + "°");
        }

        // Dodaj aktualizację dla potJointValue
        QLabel* potJointLabel = findChild<QLabel*>(QString("potJointValue%1").arg(i + 1));
        if (potJointLabel) {
            potJointLabel->setText(QString::number(angle, 'f', 2) + "°");
        }
    }

}

/**
 * @brief Destruktor klasy MainWindow
 *
 * Zwalnia zaalokowane zasoby, w tym interfejs użytkownika.
 */
MainWindow::~MainWindow()
{
    delete ui;
}
