#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

typedef struct
{
    // Stan (x, y, z, vx, vy, vz)
    float state[6];

    // Macierz kowariancji (6x6)
    float P[6][6];

    // Macierz szumu procesu
    float Q[6][6];

    // Macierz szumu pomiaru
    float R[3][3];

    // Czas ostatniej aktualizacji
    uint32_t last_update;

    // Flaga inicjalizacji
    uint8_t is_initialized;
} KalmanFilter;

void kalman_filter_init(KalmanFilter *kf);
void kalman_filter_predict(KalmanFilter *kf, float dt);
void kalman_filter_update(KalmanFilter *kf, const float *measurements);

// Pomocnicze funkcje do operacji na macierzach
void matrix_multiply(float *A, float *B, float *C, int m, int n, int p);
void matrix_add(float *A, float *B, float *C, int m, int n);
void matrix_transpose(float *A, float *AT, int m, int n);
void matrix_inverse_3x3(float *A, float *Ainv);

#endif // KALMAN_FILTER_H