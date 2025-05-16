#include "serialReader.hpp"
#include <QDebug>

SerialReader::SerialReader(QObject *parent) : QObject(parent)
{
    // Konfiguracja timera do odczytu danych
    readTimer = new QTimer(this);
    connect(readTimer, &QTimer::timeout, this, &SerialReader::readData);

    // Konfiguracja portu szeregowego
    serialPort.setBaudRate(QSerialPort::Baud115200);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);

    // Dodaj te linie
    qDebug() << "Dostępne porty:";
    foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts()) {
        qDebug() << "Port:" << portInfo.portName()
        << "Description:" << portInfo.description()
        << "Manufacturer:" << portInfo.manufacturer();
    }
}

SerialReader::~SerialReader()
{
    closePort();
}

bool SerialReader::findAndOpenPort()
{
    closePort(); // Zamknij port jeśli był otwarty

    // Szukaj dostępnych portów
    foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts()) {
        qDebug() << "Próba otwarcia portu:" << portInfo.portName();

        serialPort.setPort(portInfo);
        if (serialPort.open(QIODevice::ReadOnly)) {
            qDebug() << "Pomyślnie otwarto port:" << portInfo.portName();
            readTimer->start(10); // Rozpocznij odczyt co 10ms
            return true;
        }
    }

    qDebug() << "Nie znaleziono dostępnego portu";
    return false;
}

void SerialReader::closePort()
{
    readTimer->stop();
    if (serialPort.isOpen()) {
        serialPort.close();
    }
}

void SerialReader::readData()
{
    if (!serialPort.isOpen()) {
        qDebug() << "Port nie jest otwarty";
        return;
    }

    QByteArray data = serialPort.readAll();
    if (data.isEmpty()) {
        qDebug() << "Brak danych z portu";
        return;
    }

    qDebug() << "Odebrane dane:" << data;

    SensorDataPoint sensorData;
    if (processReceivedData(data, sensorData)) {
        qDebug() << "Przetworzone dane:";
        qDebug() << "Accel:" << sensorData.imuData.accel.x
                 << sensorData.imuData.accel.y
                 << sensorData.imuData.accel.z;
        qDebug() << "Gyro:" << sensorData.imuData.gyro.x
                 << sensorData.imuData.gyro.y
                 << sensorData.imuData.gyro.z;
        qDebug() << "Mag:" << sensorData.imuData.mag.x
                 << sensorData.imuData.mag.y
                 << sensorData.imuData.mag.z;
        qDebug() << "Temp:" << sensorData.imuData.temperature;
        emit newDataAvailable(sensorData);
    } else {
        qDebug() << "Nie udało się przetworzyć danych";
    }
}

bool SerialReader::processReceivedData(const QByteArray& data, SensorDataPoint& outData)
{
    // Usuń \r\n z końca
    QByteArray cleanData = data.trimmed();

    // Rozdziel dane i sumę kontrolną
    QList<QByteArray> mainParts = cleanData.split('*');
    if (mainParts.size() != 2) {
        qDebug() << "Nieprawidłowy format ramki - brak sumy kontrolnej";
        return false;
    }

    // Podziel główną część danych
    QList<QByteArray> values = mainParts[0].split(';');
    if (values.size() < 16) {
        qDebug() << "Za mało wartości w ramce:" << values.size();
        return false;
    }

    // Ustaw timestamp i login
    outData.timestamp = QDateTime::currentDateTimeUtc();
    outData.userLogin = "mich-mar";

    bool ok;
    try {
        // Parsuj dane ADC (A0-A5)
        for (int i = 0; i < 6; i++) {
            outData.adcData.adc[i] = values[i].toDouble(&ok);
            if (!ok) throw QString("adc[%1]").arg(i);
        }

        // Parsuj dane akcelerometru
        outData.imuData.accel.x = values[6].toDouble(&ok);
        if (!ok) throw QString("accel.x");
        outData.imuData.accel.y = values[7].toDouble(&ok);
        if (!ok) throw QString("accel.y");
        outData.imuData.accel.z = values[8].toDouble(&ok);
        if (!ok) throw QString("accel.z");

        // Parsuj dane żyroskopu
        outData.imuData.gyro.x = values[9].toDouble(&ok);
        if (!ok) throw QString("gyro.x");
        outData.imuData.gyro.y = values[10].toDouble(&ok);
        if (!ok) throw QString("gyro.y");
        outData.imuData.gyro.z = values[11].toDouble(&ok);
        if (!ok) throw QString("gyro.z");

        // Parsuj dane magnetometru
        outData.imuData.mag.x = values[12].toDouble(&ok);
        if (!ok) throw QString("mag.x");
        outData.imuData.mag.y = values[13].toDouble(&ok);
        if (!ok) throw QString("mag.y");
        outData.imuData.mag.z = values[14].toDouble(&ok);
        if (!ok) throw QString("mag.z");

        // Parsuj temperaturę
        outData.imuData.temperature = values[15].toDouble(&ok);
        if (!ok) throw QString("temperature");

        return true;

    } catch (const QString& error) {
        qDebug() << "Błąd konwersji" << error << "wartość:" << values[values.indexOf(error)];
        return false;
    }
}
