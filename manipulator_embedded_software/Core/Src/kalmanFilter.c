#include "kalmanFilter.h"
#include <string.h>
#include <math.h>

// Mnożenie macierzy: C = A * B
// A: macierz m x n
// B: macierz n x p
// C: macierz m x p
void matrix_multiply(float *A, float *B, float *C, int m, int n, int p) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            C[i * p + j] = 0;
            for (int k = 0; k < n; k++) {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

// Dodawanie macierzy: C = A + B
// A, B, C: macierze m x n
void matrix_add(float *A, float *B, float *C, int m, int n) {
    int size = m * n;
    for (int i = 0; i < size; i++) {
        C[i] = A[i] + B[i];
    }
}

// Transponowanie macierzy: AT = A^T
// A: macierz m x n
// AT: macierz n x m
void matrix_transpose(float *A, float *AT, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            AT[j * m + i] = A[i * n + j];
        }
    }
}

// Odwracanie macierzy 3x3: Ainv = A^(-1)
// A: macierz 3x3
// Ainv: odwrócona macierz 3x3
void matrix_inverse_3x3(float *A, float *Ainv) {
    // Obliczanie wyznacznika
    float det = A[0] * (A[4] * A[8] - A[5] * A[7]) -
                A[1] * (A[3] * A[8] - A[5] * A[6]) +
                A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    // Sprawdzenie czy macierz jest odwracalna
    if (fabsf(det) < 1e-10f) {
        // Jeśli wyznacznik jest bliski zeru, zwróć macierz jednostkową
        for (int i = 0; i < 9; i++) {
            Ainv[i] = (i % 4) == 0 ? 1.0f : 0.0f;
        }
        return;
    }
    
    float inv_det = 1.0f / det;
    
    // Obliczanie dopełnień algebraicznych
    Ainv[0] = (A[4] * A[8] - A[5] * A[7]) * inv_det;
    Ainv[1] = (A[2] * A[7] - A[1] * A[8]) * inv_det;
    Ainv[2] = (A[1] * A[5] - A[2] * A[4]) * inv_det;
    
    Ainv[3] = (A[5] * A[6] - A[3] * A[8]) * inv_det;
    Ainv[4] = (A[0] * A[8] - A[2] * A[6]) * inv_det;
    Ainv[5] = (A[2] * A[3] - A[0] * A[5]) * inv_det;
    
    Ainv[6] = (A[3] * A[7] - A[4] * A[6]) * inv_det;
    Ainv[7] = (A[1] * A[6] - A[0] * A[7]) * inv_det;
    Ainv[8] = (A[0] * A[4] - A[1] * A[3]) * inv_det;
}

// Pomocnicza funkcja do debugowania - wyświetlanie macierzy
void matrix_print(float *A, int m, int n, const char *name) {
    char buffer[256];
    int len = 0;
    
    len += snprintf(buffer + len, sizeof(buffer) - len, "Matrix %s (%dx%d):\r\n", name, m, n);
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            len += snprintf(buffer + len, sizeof(buffer) - len, "%.4f ", A[i * n + j]);
        }
        len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");
    }
    
    sendUSBmsg(buffer);
}

void kalman_filter_init(KalmanFilter *kf) {
    memset(kf, 0, sizeof(KalmanFilter));
    
    // Inicjalizacja macierzy kowariancji P
    for(int i = 0; i < 6; i++) {
        kf->P[i][i] = 1.0f;
    }
    
    // Inicjalizacja macierzy szumu procesu Q
    // Większe wartości dla prędkości
    for(int i = 0; i < 3; i++) {
        kf->Q[i][i] = 0.01f;        // dla pozycji
        kf->Q[i+3][i+3] = 0.1f;     // dla prędkości
    }
    
    // Inicjalizacja macierzy szumu pomiaru R
    for(int i = 0; i < 3; i++) {
        kf->R[i][i] = 0.1f;         // dla pomiarów przyspieszenia
    }
    
    kf->is_initialized = 1;
}

void kalman_filter_predict(KalmanFilter *kf, float dt) {
    // F = [I       dt*I]
    //     [0       I   ]
    float F[6][6] = {0};
    for(int i = 0; i < 6; i++) {
        F[i][i] = 1.0f;
    }
    for(int i = 0; i < 3; i++) {
        F[i][i+3] = dt;
    }
    
    // Predykcja stanu: x = F*x
    float new_state[6] = {0};
    for(int i = 0; i < 3; i++) {
        new_state[i] = kf->state[i] + kf->state[i+3] * dt;
        new_state[i+3] = kf->state[i+3];
    }
    memcpy(kf->state, new_state, sizeof(new_state));
    
    // Predykcja kowariancji: P = F*P*F' + Q
    float temp[6][6] = {0};
    float FP[6][6] = {0};
    
    // F*P
    matrix_multiply((float*)F, (float*)kf->P, (float*)FP, 6, 6, 6);
    
    // (F*P)*F' + Q
    float FT[6][6];
    matrix_transpose((float*)F, (float*)FT, 6, 6);
    matrix_multiply((float*)FP, (float*)FT, (float*)temp, 6, 6, 6);
    matrix_add((float*)temp, (float*)kf->Q, (float*)kf->P, 6, 6);
}

void kalman_filter_update(KalmanFilter *kf, const float *measurements) {
    // H = [0 0 0 I]
    float H[3][6] = {0};
    for(int i = 0; i < 3; i++) {
        H[i][i+3] = 1.0f;
    }
    
    // y = z - H*x
    float y[3];
    for(int i = 0; i < 3; i++) {
        y[i] = measurements[i] - kf->state[i+3];
    }
    
    // S = H*P*H' + R
    float HP[3][6];
    float S[3][3];
    float HT[6][3];
    
    matrix_multiply((float*)H, (float*)kf->P, (float*)HP, 3, 6, 6);
    matrix_transpose((float*)H, (float*)HT, 3, 6);
    matrix_multiply((float*)HP, (float*)HT, (float*)S, 3, 6, 3);
    matrix_add((float*)S, (float*)kf->R, (float*)S, 3, 3);
    
    // K = P*H'*S^-1
    float K[6][3];
    float Sinv[3][3];
    float PHT[6][3];
    
    matrix_inverse_3x3((float*)S, (float*)Sinv);
    matrix_multiply((float*)kf->P, (float*)HT, (float*)PHT, 6, 6, 3);
    matrix_multiply((float*)PHT, (float*)Sinv, (float*)K, 6, 3, 3);
    
    // x = x + K*y
    for(int i = 0; i < 6; i++) {
        float correction = 0;
        for(int j = 0; j < 3; j++) {
            correction += K[i][j] * y[j];
        }
        kf->state[i] += correction;
    }
    
    // P = (I - K*H)*P
    float KH[6][6] = {0};
    float I_KH[6][6];
    float temp[6][6];
    
    matrix_multiply((float*)K, (float*)H, (float*)KH, 6, 3, 6);
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    matrix_multiply((float*)I_KH, (float*)kf->P, (float*)temp, 6, 6, 6);
    memcpy(kf->P, temp, sizeof(temp));
}