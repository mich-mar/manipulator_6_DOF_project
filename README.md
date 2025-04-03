# Manipulator o sześciu stopniach swobody - Komunikacja i sterowanie

Projekt ma na celu opracowanie systemu sterowania manipulatorem o sześciu stopniach swobody. Skupia się na implementacji magistrali komunikacyjnych, doborze mikrokontrolerów, czujników oraz opracowaniu algorytmów przetwarzania danych z tych czujników.

## Opis projektu

Projekt obejmuje zaprojektowanie i implementację systemu sterowania, w którym wykorzystuje się:

- **Mikrokontroler STM32** do akwizycji danych z czujników i sterowania serwomechanizmami.
- **Magistrale komunikacyjne (I2C, SPI)** do komunikacji z czujnikami (potencjometry, moduł IMU).
- **Moduł ESP32** do bezprzewodowej transmisji danych (Wi-Fi/Bluetooth).

Dane z czujników IMU oraz potencjometrów będą przetwarzane, a na podstawie tych danych sterowane będą serwomechanizmy manipulujące położeniem manipulatora.

## Technologie

- **STM32** - mikrokontroler odpowiedzialny za akwizycję danych i sterowanie.
- **I2C/SPI** - magistrale komunikacyjne do wymiany danych z czujnikami.
- **PWM/UART** - protokoły używane do sterowania serwomechanizmami.
- **ESP32** - moduł umożliwiający bezprzewodową transmisję danych.
- **C** - język programowania wykorzystywany do implementacji oprogramowania mikrokontrolera.
- **Markdown** - do dokumentacji.
