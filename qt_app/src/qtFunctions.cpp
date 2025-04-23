#include "qtFunctions.h"

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

QChartView* createAngleChart(int colorIndex) {
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

QChartView* createIMUChart(int axisIndex) {
    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->setMargins(QMargins(5, 5, 5, 5));
    chart->setBackgroundBrush(QBrush(Qt::white));
    chart->setTitle("");

    QLineSeries *series = new QLineSeries();
    QPen pen(IMU_COLORS[axisIndex]);
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

QPixmap createManipulatorPlaceholder() {
    QPixmap placeholder(MANIPULATOR_WIDTH, MANIPULATOR_HEIGHT);
    placeholder.fill(Qt::white);

    QPainter painter(&placeholder);
    painter.setPen(QPen(Qt::black, 2));

    int centerX = MANIPULATOR_WIDTH / 2;
    int baseY = MANIPULATOR_HEIGHT - 50;

    painter.drawRect(centerX - 50, baseY, 100, 30);
    painter.drawLine(centerX, baseY, centerX, baseY - 100);
    painter.drawLine(centerX, baseY - 100, centerX + 120, baseY - 150);
    painter.drawLine(centerX + 120, baseY - 150, centerX + 150, baseY - 250);
    painter.drawEllipse(centerX + 140, baseY - 270, 20, 20);

    painter.drawText(10, 20, "Manipulator 6 DOF (placeholder)");

    painter.end();
    return placeholder;
}
