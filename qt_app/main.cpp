/**
 * @file main.cpp
 * @author Michał Markuzel
 * @date 2025-04-22
 * @brief Aplikacja do wizualizacji i modelowania manipulatorem 6 DOF
 *
 * Program umożliwia monitorowanie kątów przegubów manipulatora oraz danych z IMU.
 * Aplikacja zawiera wykresy dla 6 potencjometrów oraz 3 zestawy wykresów dla danych z IMU.
 * Dodatkowo przygotowane jest miejsce na wizualizację 3D manipulatora.
 */

#include <QApplication>
#include <QMainWindow>
#include <QGridLayout>
#include <QVBoxLayout>
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
#include <QResource>
#include <QDebug>
#include <QDir>
#include <QDirIterator>

QT_CHARTS_USE_NAMESPACE

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
 * @brief Struktura przechowująca dane z czujników IMU.
 *
 * Struktura zawiera dane z trzech czujników: akcelerometru, żyroskopu i magnetometru.
 * Każdy z tych czujników dostarcza wartości dla trzech osi (X, Y, Z), które są przechowywane
 * w postaci zmiennych typu float.
 */
typedef struct {
    /**
     * @brief Dane z akcelerometru.
     *
     * Zawiera wartości przyspieszenia dla osi X, Y, Z. Jednostką miary jest m/s².
     */
    struct {
        float x; /**< Przyspieszenie w osi X */
        float y; /**< Przyspieszenie w osi Y */
        float z; /**< Przyspieszenie w osi Z */
    } accel;

    /**
     * @brief Dane z żyroskopu.
     *
     * Zawiera wartości prędkości kątowej dla osi X, Y, Z. Jednostką miary jest stopień na sekundę (°/s).
     */
    struct {
        float x; /**< Prędkość kątowa w osi X */
        float y; /**< Prędkość kątowa w osi Y */
        float z; /**< Prędkość kątowa w osi Z */
    } gyro;

    /**
     * @brief Dane z magnetometru.
     *
     * Zawiera wartości natężenia pola magnetycznego dla osi X, Y, Z. Jednostką miary jest uT (mikrotesla).
     */
    struct {
        float x; /**< Natężenie pola magnetycznego w osi X */
        float y; /**< Natężenie pola magnetycznego w osi Y */
        float z; /**< Natężenie pola magnetycznego w osi Z */
    } mag;
} imu_data;

/**
 * @brief Zmienna globalna przechowująca wartości kątów.
 *
 * Zawiera sześć zmiennych typu float, które reprezentują różne kąty, przechowywane w jednostkach stopni.
 */
angles ANGLES_DATA;

/**
 * @brief Zmienna globalna przechowująca dane z czujników IMU.
 *
 * Zawiera dane z trzech czujników: akcelerometru, żyroskopu i magnetometru. Każdy czujnik
 * dostarcza wartości dla trzech osi (X, Y, Z) przechowywanych w zmiennych typu float.
 */
imu_data IMU_DATA;



/**
 * @brief Funkcja tworząca obrócony tekst w etykiecie
 * @param text Tekst do obrócenia
 * @param font Czcionka dla tekstu
 * @param width Szerokość wynikowego pixmapy
 * @param height Wysokość wynikowej pixmapy
 * @return QPixmap z obróconym tekstem
 */
QPixmap createRotatedText(const QString &text, const QFont &font, int width, int height) {
    QTransform transform;
    transform.rotate(-90);

    QPixmap pixmap(width, height);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setTransform(transform);
    painter.setFont(font);
    painter.drawText(QRect(-height, 0, height, width), Qt::AlignCenter, text);
    painter.end();

    return pixmap;
}

/**
 * @brief Funkcja tworząca wykres kąta
 * @param colorIndex Indeks koloru z tablicy ANGLE_COLORS
 * @return QChartView z przygotowanym wykresem
 */
QChartView *createAngleChart(int colorIndex) {
    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->setMargins(QMargins(5, 5, 5, 5));
    chart->setBackgroundBrush(QBrush(Qt::white));
    chart->setTitle("");

    QLineSeries *series = new QLineSeries();

    QPen pen(ANGLE_COLORS[colorIndex]);
    pen.setWidth(2);
    series->setPen(pen);

    chart->addSeries(series);

    QValueAxis *axisX = new QValueAxis();
    axisX->setRange(0, CHART_TIME_RANGE);
    axisX->setGridLineVisible(true);
    axisX->setTickCount(7);
    axisX->setTitleText("");
    axisX->setLabelsVisible(false);

    QValueAxis *axisY = new QValueAxis();
    axisY->setRange(0, ANGLE_MAX_VALUE);
    axisY->setGridLineVisible(true);
    axisY->setTickCount(5);
    axisY->setTitleText("");
    axisY->setLabelsVisible(false);

    chart->setAxisX(axisX, series);
    chart->setAxisY(axisY, series);

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setMinimumHeight(80);

    return chartView;
}

/**
 * @brief Funkcja tworząca wykres IMU
 * @param axisIndex Indeks osi (0-X, 1-Y, 2-Z)
 * @return QChartView z przygotowanym wykresem IMU
 */
QChartView *createIMUChart(int axisIndex) {
    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->setMargins(QMargins(5, 5, 5, 5));
    chart->setBackgroundBrush(QBrush(Qt::white));
    chart->setTitle("");

    // Dodajemy serię danych - najpierw tylko jedną serię dla każdego wykresu
    QLineSeries *series = new QLineSeries();
    QPen pen(IMU_COLORS[axisIndex]);
    pen.setWidth(2);
    series->setPen(pen);
    chart->addSeries(series);

    // Osie
    QValueAxis *axisX = new QValueAxis();
    axisX->setRange(0, CHART_TIME_RANGE);
    axisX->setGridLineVisible(true);
    axisX->setTickCount(7);
    axisX->setTitleText("");
    axisX->setLabelsVisible(false);

    QValueAxis *axisY = new QValueAxis();
    axisY->setRange(-IMU_VALUE_RANGE, IMU_VALUE_RANGE);
    axisY->setGridLineVisible(true);
    axisY->setTickCount(5);
    axisY->setTitleText("");
    axisY->setLabelsVisible(false);

    chart->setAxisX(axisX, series);
    chart->setAxisY(axisY, series);

    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setMinimumHeight(80);

    return chartView;
}

/**
 * @brief Funkcja tworząca placeholder flagi
 * @param text Tekst na placeholderze flagi
 * @param color Kolor tła flagi
 * @return QPixmap z flagą
 */
QPixmap createFlagPlaceholder(const QString &text, const QColor &color) {
    QPixmap flagPixmap(30, 20);
    flagPixmap.fill(color);

    QPainter painter(&flagPixmap);
    painter.setPen(Qt::black);
    painter.drawText(flagPixmap.rect(), Qt::AlignCenter, text);
    painter.drawRect(0, 0, flagPixmap.width() - 1, flagPixmap.height() - 1);
    painter.end();

    return flagPixmap;
}

/**
 * @brief Funkcja tworząca placeholder manipulatora jeśli brak obrazu
 * @return QPixmap z placeholderem
 */
QPixmap createManipulatorPlaceholder() {
    QPixmap placeholder(MANIPULATOR_WIDTH, MANIPULATOR_HEIGHT);
    placeholder.fill(Qt::white);

    QPainter painter(&placeholder);
    painter.setPen(QPen(Qt::black, 2));

    // Rysujemy prosty szkic manipulatora
    int centerX = MANIPULATOR_WIDTH / 2;
    int baseY = MANIPULATOR_HEIGHT - 50;

    // Podstawa
    painter.drawRect(centerX - 50, baseY, 100, 30);

    // Ramię 1
    painter.drawLine(centerX, baseY, centerX, baseY - 100);

    // Ramię 2
    painter.drawLine(centerX, baseY - 100, centerX + 120, baseY - 150);

    // Ramię 3
    painter.drawLine(centerX + 120, baseY - 150, centerX + 150, baseY - 250);

    // Chwytak
    painter.drawEllipse(centerX + 140, baseY - 270, 20, 20);

    // Dodajemy tekst informacyjny
    painter.drawText(10, 20, "Manipulator 6 DOF (placeholder)");

    painter.end();
    return placeholder;
}

/**
 * @brief Główna funkcja programu
 * @param argc Liczba argumentów wiersza poleceń
 * @param argv Tablica argumentów wiersza poleceń
 * @return Kod zakończenia programu
 */
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Tworzenie głównego okna
    QMainWindow window;
    window.setWindowTitle(APP_TITLE);

    // Główny widżet
    QWidget *centralWidget = new QWidget(&window);
    window.setCentralWidget(centralWidget);

    // Główny układ
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);

    // Część lewa - wykresy i potencjometry
    QVBoxLayout *leftLayout = new QVBoxLayout();

    // Tworzymy siatkę dla lewej części
    QGridLayout *chartsGridLayout = new QGridLayout();

    // Przygotowanie czcionki dla etykiet
    QFont labelFont;
    labelFont.setBold(true);
    labelFont.setPointSize(LABEL_FONT_SIZE);

    // Dodajemy pionowy napis ANGLES
    QLabel *anglesLabel = new QLabel("ANGLES");
    anglesLabel->setFixedWidth(40);
    anglesLabel->setFont(labelFont);
    anglesLabel->setPixmap(createRotatedText("ANGLES", labelFont, 50, 200));

    chartsGridLayout->addWidget(anglesLabel, 0, 0, NUM_ANGLE_CHARTS, 1);

    // Tworzenie wykresów dla potencjometrów
    for (int i = 0; i < NUM_ANGLE_CHARTS; i++) {
        // Etykieta dla angle
        QLabel *angleLabel = new QLabel("angle");
        angleLabel->setFixedWidth(40);
        QString label = QString("angle_%1").arg(i + 1);
        angleLabel->setPixmap(createRotatedText(label, labelFont, 40, 100));

        // Tworzenie wykresu
        QChartView *chartView = createAngleChart(i);

        // Potencjometr
        QDial *dial = new QDial();
        dial->setFixedSize(60, 60);

        // Dodajemy wykres i potencjometr do siatki
        chartsGridLayout->addWidget(angleLabel, i, 1);
        chartsGridLayout->addWidget(chartView, i, 2);
        chartsGridLayout->addWidget(dial, i, 3);
    }

    // Sekcja IMU
    QLabel *imuLabel = new QLabel("IMU");
    imuLabel->setFixedWidth(40);
    imuLabel->setPixmap(createRotatedText("IMU", labelFont, 50, 150));

    chartsGridLayout->addWidget(imuLabel, NUM_ANGLE_CHARTS, 0, NUM_IMU_CHARTS, 1);

    // Tworzymy wykresy dla IMU - z tym samym wyglądem jak dla angles
    for (int i = 0; i < NUM_IMU_CHARTS; i++) {
        // Etykieta dla osi IMU (X, Y, Z)
        QLabel *axisLabel = new QLabel(IMU_AXIS_LABELS[i]);
        axisLabel->setFixedWidth(40);
        axisLabel->setPixmap(createRotatedText(IMU_AXIS_LABELS[i], labelFont, 40, 80));

        // Tworzenie wykresu IMU z tym samym stylem co wykresy angle
        QChartView *chartView = createIMUChart(i);

        // Legenda pod wykresem
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

    // Ustawiamy rozciąganie wierszy
    for (int i = 0; i < NUM_ANGLE_CHARTS + NUM_IMU_CHARTS; i++) {
        chartsGridLayout->setRowStretch(i, 1);
    }

    // Dodajemy siatkę wykresów do lewego layoutu
    leftLayout->addLayout(chartsGridLayout);

    // Część prawa - tytuł i wizualizacja manipulatora
    QVBoxLayout *rightLayout = new QVBoxLayout();

    // Tytuł
    QLabel *titleLabel = new QLabel(APP_TITLE);
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleFont.setPointSize(TITLE_FONT_SIZE);
    titleLabel->setFont(titleFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    rightLayout->addWidget(titleLabel);

    // Miejsce na wizualizację manipulatora
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsView *manipulatorView = new QGraphicsView(scene);
    manipulatorView->setMinimumSize(400, 400);

    // Ladowanie obrazu manipulatora
    QPixmap manipulatorPixmap;
    manipulatorPixmap = QPixmap(":/icons/icons/manipulator_6dof.png");

    QGraphicsPixmapItem *manipulatorItem = scene->addPixmap(manipulatorPixmap);
    manipulatorItem->setPos(0, 0);

    rightLayout->addWidget(manipulatorView);

    // Ikony flag
    QHBoxLayout *flagsLayout = new QHBoxLayout();

    // Ladowanie ikon flag
    QPixmap ukFlagPixmap;
    QPixmap plFlagPixmap;

    ukFlagPixmap = QPixmap(":/icons/icons/uk.png");
    plFlagPixmap = QPixmap(":/icons/icons/pl.png");


    // Tworzenie etykiet z ikonami flag
    QLabel *ukFlagLabel = new QLabel();
    ukFlagLabel->setPixmap(ukFlagPixmap.scaled(30, 20, Qt::KeepAspectRatio));
    ukFlagLabel->setToolTip("English");
    ukFlagLabel->setCursor(Qt::PointingHandCursor);

    QLabel *plFlagLabel = new QLabel();
    plFlagLabel->setPixmap(plFlagPixmap.scaled(30, 20, Qt::KeepAspectRatio));
    plFlagLabel->setToolTip("Polski");
    plFlagLabel->setCursor(Qt::PointingHandCursor);

    flagsLayout->addStretch();
    flagsLayout->addWidget(ukFlagLabel);
    flagsLayout->addSpacing(10);
    flagsLayout->addWidget(plFlagLabel);
    flagsLayout->addSpacing(10);

    rightLayout->addLayout(flagsLayout);
    rightLayout->addStretch();

    // Dodanie lewy i prawego layout'u do głównego layoutu
    mainLayout->addLayout(leftLayout, LEFT_LAYOUT_WEIGHT);
    mainLayout->addLayout(rightLayout, RIGHT_LAYOUT_WEIGHT);

    // Ustawiamy okno jako zmaksymalizowane
    window.showMaximized();

    return app.exec();
}