#include "mainwindow.h"

#include <QApplication>


/**
 * @brief Główna funkcja aplikacji
 *
 * Inicjalizuje aplikację Qt, tworzy i wyświetla główne okno aplikacji.
 * Następnie uruchamia główną pętlę zdarzeń Qt.
 *
 * @param argc Liczba argumentów wiersza poleceń
 * @param argv Tablica wskaźników do argumentów wiersza poleceń
 * @return int Kod zakończenia aplikacji (0 dla poprawnego zakończenia)
 */
int main(int argc, char *argv[])
{
    // Włącz wyświetlanie komunikatów debug
    qSetMessagePattern("%{message}");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
