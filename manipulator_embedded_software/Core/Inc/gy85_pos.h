#ifndef GY85_POSITION_H
#define GY85_POSITION_H

#include "gy85.h"
#include "gy85_gravity.h"

// Stałe konfiguracyjne
#define GRAVITY 9.81f
#define COMPLEMENTARY_FILTER_ALPHA 0.96f
#define VELOCITY_DECAY 0.98f        // Współczynnik tłumienia prędkości
#define MIN_ACCEL_THRESHOLD 0.1f    // Próg minimalnego przyspieszenia (m/s^2)
#define MAX_ACCEL_THRESHOLD 20.0f   // Próg maksymalnego przyspieszenia (m/s^2)

typedef struct {
    // Pozycja w przestrzeni
    struct {
        float x;
        float y;
        float z;
    } position;
    
    // Prędkość
    struct {
        float x;
        float y;
        float z;
    } velocity;
    
    // Kąty orientacji (w radianach)
    struct {
        float roll;    // obrót wokół osi X
        float pitch;   // obrót wokół osi Y
        float yaw;     // obrót wokół osi Z
    } orientation;
    
    // Czas ostatniego pomiaru
    uint32_t last_update;
    
    // Stan inicjalizacji
    int initialized;
} IMU_Position;

// Funkcje do obsługi pozycji
HAL_StatusTypeDef IMU_InitPosition(IMU_Position *position);
HAL_StatusTypeDef IMU_UpdatePosition(GY85_HandleTypeDef *hgy85, IMU_Position *position);
void IMU_ResetPosition(IMU_Position *position);

#endif // GY85_POSITION_H