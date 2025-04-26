#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>

void floatToString(float value, char *buffer)
{
    // Obsługa liczb ujemnych
    if (value < 0) {
        *buffer++ = '-';
        value = -value;
    }

    // Część całkowita
    int intPart = (int)value;
    float fraction = value - (float)intPart;

    // Zamiana części całkowitej na string
    int tempInt = intPart;
    char temp[20];
    int i = 0;
    if (tempInt == 0) {
        temp[i++] = '0';
    } else {
        while (tempInt > 0) {
            temp[i++] = (tempInt % 10) + '0';
            tempInt /= 10;
        }
    }

    // Odwrócenie liczby całkowitej
    for (int j = i - 1; j >= 0; j--) {
        *buffer++ = temp[j];
    }

    // Kropka
    *buffer++ = '.';

    // Część ułamkowa (6 miejsc po przecinku)
    for (int j = 0; j < 6; j++) {
        fraction *= 10;
        int digit = (int)fraction;
        *buffer++ = digit + '0';
        fraction -= digit;
    }

    // Na końcu null-terminator
    *buffer = '\0';
}



#endif // UTILS_H
