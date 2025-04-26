/**
 * @file main.cpp
 * @author Michał Markuzel
 * @date 2025-04-15
 * @brief Aplikacja do wizualizacji i modelowania manipulatorem 6 DOF
 *
 * Program umożliwia monitorowanie kątów przegubów manipulatora oraz danych z IMU.
 * Zawiera wykresy dla 6 potencjometrów oraz 3 zestawy wykresów dla danych z IMU.
 * Uwzględniono również miejsce na wizualizację 3D manipulatora.
 */

#include "definitions.h"
#include "qtFunctions.h"

// Inicjalizacja kolejki do przechowywania danych wejściowych typu inputData
QQueue<inputData> dataQueue;

/**
 * @brief Główna funkcja programu.
 * @param argc Liczba argumentów wiersza poleceń.
 * @param argv Tablica argumentów wiersza poleceń.
 * @return Kod zakończenia programu.
 */
int main(int argc, char *argv[]) {
    // Inicjalizacja aplikacji Qt
    QApplication app(argc, argv);

    // Tworzenie głównego okna aplikacji
    QMainWindow window;
    window.setWindowTitle(APP_TITLE);

    // Ustawienie głównego widżetu jako centralnego
    QWidget *centralWidget = new QWidget(&window);
    window.setCentralWidget(centralWidget);

    // Utworzenie głównego układu poziomego (podział na lewą i prawą część)
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);

    // Utworzenie layoutu dla lewej części (wykresy i kontrolki)
    QVBoxLayout *leftLayout = new QVBoxLayout();

    // Siatka układu dla wykresów i kontrolek
    QGridLayout *chartsGridLayout = new QGridLayout();

    // Konfiguracja czcionki dla etykiet
    QFont labelFont;
    labelFont.setBold(true);
    labelFont.setPointSize(LABEL_FONT_SIZE);

    // Dodanie pionowej etykiety "ANGLES"
    QLabel *anglesLabel = new QLabel("ANGLES");
    anglesLabel->setFixedWidth(40);
    anglesLabel->setFont(labelFont);
    anglesLabel->setPixmap(createRotatedText("ANGLES", labelFont, 50, 200));
    chartsGridLayout->addWidget(anglesLabel, 0, 0, NUM_ANGLE_CHARTS, 1);

    // Dodanie wykresów i potencjometrów dla każdej osi (kąta)
    for (int i = 0; i < NUM_ANGLE_CHARTS; i++) {
        QLabel *angleLabel = new QLabel("angle");
        angleLabel->setFixedWidth(40);
        QString label = QString("angle_%1").arg(i + 1);
        angleLabel->setPixmap(createRotatedText(label, labelFont, 40, 100));

        QChartView *chartView = createAngleChart(i); // Utworzenie wykresu kąta
        QDial *dial = new QDial();                   // Potencjometr (kontrolka)
        dial->setFixedSize(60, 60);

        chartsGridLayout->addWidget(angleLabel, i, 1);
        chartsGridLayout->addWidget(chartView, i, 2);
        chartsGridLayout->addWidget(dial, i, 3);
    }

    // Dodanie pionowej etykiety "IMU"
    QLabel *imuLabel = new QLabel("IMU");
    imuLabel->setFixedWidth(40);
    imuLabel->setPixmap(createRotatedText("IMU", labelFont, 50, 150));
    chartsGridLayout->addWidget(imuLabel, NUM_ANGLE_CHARTS, 0, NUM_IMU_CHARTS, 1);

    // Dodanie wykresów dla danych z IMU (X, Y, Z)
    for (int i = 0; i < NUM_IMU_CHARTS; i++) {
        QLabel *axisLabel = new QLabel(IMU_AXIS_LABELS[i]);
        axisLabel->setFixedWidth(40);
        axisLabel->setPixmap(createRotatedText(IMU_AXIS_LABELS[i], labelFont, 40, 80));

        QChartView *chartView = createIMUChart(i); // Utworzenie wykresu IMU

        // Utworzenie legendy dla wykresów IMU
        QHBoxLayout *legendLayout = new QHBoxLayout();
        for (int j = 0; j < 4; j++) {
            QLabel *legendLabel = new QLabel("■ " + IMU_LABELS[j]);
            QString styleSheet = QString("color: %1; font-size: 8pt;").arg(IMU_COLORS[j].name());
            legendLabel->setStyleSheet(styleSheet);
            legendLabel->setAlignment(Qt::AlignCenter);
            legendLayout->addWidget(legendLabel);
        }

        QVBoxLayout *imuChartLayout = new QVBoxLayout();
        imuChartLayout->addWidget(chartView);
        imuChartLayout->addLayout(legendLayout);

        chartsGridLayout->addWidget(axisLabel, i + NUM_ANGLE_CHARTS, 1);
        chartsGridLayout->addLayout(imuChartLayout, i + NUM_ANGLE_CHARTS, 2);
    }

    // Ustawienie proporcji siatki (jednakowe rozciąganie w pionie)
    for (int i = 0; i < NUM_ANGLE_CHARTS + NUM_IMU_CHARTS; i++) {
        chartsGridLayout->setRowStretch(i, 1);
    }

    // Dodanie siatki wykresów do lewego layoutu
    leftLayout->addLayout(chartsGridLayout);

    // Utworzenie layoutu dla prawej części (wizualizacja + tytuł)
    QVBoxLayout *rightLayout = new QVBoxLayout();

    // Dodanie tytułu aplikacji
    QLabel *titleLabel = new QLabel(APP_TITLE);
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleFont.setPointSize(TITLE_FONT_SIZE);
    titleLabel->setFont(titleFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    rightLayout->addWidget(titleLabel);

    // Utworzenie widoku dla wizualizacji manipulatora
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsView *manipulatorView = new QGraphicsView(scene);
    manipulatorView->setMinimumSize(500, 500);

    // Załadowanie i dodanie obrazu manipulatora
    QPixmap manipulatorPixmap(":/icons/icons/manipulator_6dof.png");
    QGraphicsPixmapItem *manipulatorItem = scene->addPixmap(manipulatorPixmap);
    manipulatorItem->setPos(0, 0);

    rightLayout->addWidget(manipulatorView);

    // Sekcja językowa – dodanie ikon flag
    QHBoxLayout *flagsLayout = new QHBoxLayout();

    QPixmap ukFlagPixmap(":/icons/icons/uk.png");
    QPixmap plFlagPixmap(":/icons/icons/pl.png");

    QLabel *ukFlagLabel = new QLabel();
    ukFlagLabel->setPixmap(ukFlagPixmap.scaled(50, 50, Qt::KeepAspectRatio));
    ukFlagLabel->setToolTip("English");
    ukFlagLabel->setCursor(Qt::PointingHandCursor);

    QLabel *plFlagLabel = new QLabel();
    plFlagLabel->setPixmap(plFlagPixmap.scaled(50, 50, Qt::KeepAspectRatio));
    plFlagLabel->setToolTip("Polski");
    plFlagLabel->setCursor(Qt::PointingHandCursor);

    flagsLayout->addStretch();
    flagsLayout->addWidget(ukFlagLabel);
    flagsLayout->addSpacing(10);
    flagsLayout->addWidget(plFlagLabel);
    flagsLayout->addSpacing(10);

    rightLayout->addLayout(flagsLayout);
    rightLayout->addStretch();

    // Dodanie obu layoutów (lewego i prawego) do głównego layoutu okna
    mainLayout->addLayout(leftLayout, LEFT_LAYOUT_WEIGHT);
    mainLayout->addLayout(rightLayout, RIGHT_LAYOUT_WEIGHT);

    // Wyświetlenie głównego okna w trybie maksymalizacji
    window.showMaximized();

    // Uruchomienie głównej pętli aplikacji Qt
    return app.exec();
}
