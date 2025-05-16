#include "dataQueue.hpp"
#include <QMutexLocker>

void DataQueue::enqueue(const SensorDataPoint& data) {
    QMutexLocker locker(&mutex);  // Automatyczna blokada mutexa
    dataQueue.enqueue(data);

    // Usuń najstarsze elementy, jeśli przekroczono maksymalny rozmiar
    while (dataQueue.size() > MAX_QUEUE_SIZE) {
        dataQueue.dequeue();
    }
}

QQueue<SensorDataPoint> DataQueue::getLastPoints(int count) {
    QMutexLocker locker(&mutex);  // Automatyczna blokada mutexa
    QQueue<SensorDataPoint> result;

    // Oblicz indeks początkowy
    int startIndex = qMax(0, dataQueue.size() - count);

    // Skopiuj żądaną liczbę ostatnich elementów
    for (int i = startIndex; i < dataQueue.size(); ++i) {
        result.enqueue(dataQueue[i]);
    }
    return result;
}

void DataQueue::clear() {
    QMutexLocker locker(&mutex);  // Automatyczna blokada mutexa
    dataQueue.clear();
}
