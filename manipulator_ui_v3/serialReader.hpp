#ifndef SERIALREADER_HPP
#define SERIALREADER_HPP

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include "sensorData.hpp"

/**
 * @brief Klasa odpowiedzialna za komunikację szeregową z manipulatorem
 *
 * SerialReader obsługuje odczyt danych z portu szeregowego, do którego
 * podłączony jest manipulator. Przetwarza otrzymane dane i emituje
 * sygnały zawierające odczytane wartości z czujników.
 */
class SerialReader : public QObject {
    Q_OBJECT

public:
    /**
     * @brief Konstruktor klasy SerialReader
     * @param parent Wskaźnik na obiekt rodzica (domyślnie nullptr)
     */
    explicit SerialReader(QObject *parent = nullptr);


    /**
     * @brief Destruktor klasy SerialReader
     */
    ~SerialReader();

    /**
     * @brief Wyszukuje i otwiera port szeregowy
     * @return true jeśli port został pomyślnie otwarty, false w przeciwnym razie
     */
    bool findAndOpenPort();

    /**
     * @brief Zamyka połączenie z portem szeregowym
     */
    void closePort();

    /**
     * @brief Sprawdza czy port szeregowy jest otwarty
     * @return true jeśli port jest otwarty, false w przeciwnym razie
     */
    bool isPortConnected() const { return serialPort.isOpen(); }

signals:
    /**
     * @brief Sygnał emitowany gdy dostępne są nowe dane z czujników
     * @param data Struktura zawierająca odczytane dane z czujników
     */
    void newDataAvailable(const SensorDataPoint& data);

private slots:
    /**
     * @brief Slot wywoływany gdy dostępne są nowe dane do odczytu
     */
    void readData();

private:
    QSerialPort serialPort;    ///< Obiekt reprezentujący port szeregowy
    QTimer *readTimer;         ///< Timer kontrolujący częstotliwość odczytu
    static const int BUFFER_SIZE = 1024;  ///< Rozmiar bufora na dane
    QByteArray buffer;         ///< Bufor na otrzymywane dane

    /**
     * @brief Przetwarza otrzymane dane z portu szeregowego
     * @param data Surowe dane otrzymane z portu
     * @param outData Referencja do struktury, w której zostaną zapisane przetworzone dane
     * @return true jeśli dane zostały poprawnie przetworzone, false w przeciwnym razie
     */
    bool processReceivedData(const QByteArray& data, SensorDataPoint& outData);
};

#endif // SERIALREADER_HPP
