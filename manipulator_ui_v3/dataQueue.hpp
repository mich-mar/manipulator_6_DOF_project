#ifndef DATAQUEUE_HPP
#define DATAQUEUE_HPP

#include <QQueue>
#include <QMutex>
#include "sensorData.hpp"

/**
 * @brief Klasa implementująca bezpieczną wątkowo kolejkę danych z czujników
 *
 * Klasa DataQueue zapewnia thread-safe mechanizm przechowywania i dostępu
 * do danych z czujników. Implementuje mechanizm kolejki o ograniczonym rozmiarze
 * z automatycznym usuwaniem najstarszych elementów.
 */
class DataQueue {
public:
    /** @brief Maksymalny rozmiar kolejki */
    static const int MAX_QUEUE_SIZE = 1000;

    /**
     * @brief Dodaje nowy punkt pomiarowy do kolejki
     *
     * Metoda automatycznie usuwa najstarsze elementy, jeśli przekroczony
     * zostanie maksymalny rozmiar kolejki (MAX_QUEUE_SIZE).
     *
     * @param data Struktura zawierająca dane z czujników do dodania
     */
    void enqueue(const SensorDataPoint& data);

    /**
     * @brief Pobiera określoną liczbę ostatnich punktów pomiarowych
     *
     * @param count Liczba punktów do pobrania
     * @return QQueue<SensorDataPoint> Kolejka zawierająca żądaną liczbę ostatnich punktów
     */
    QQueue<SensorDataPoint> getLastPoints(int count);

    /**
     * @brief Czyści zawartość kolejki
     */
    void clear();

private:
    QQueue<SensorDataPoint> dataQueue;  ///< Kolejka przechowująca dane z czujników
    QMutex mutex;                       ///< Mutex zapewniający bezpieczeństwo wielowątkowe
};

#endif // DATAQUEUE_HPP
