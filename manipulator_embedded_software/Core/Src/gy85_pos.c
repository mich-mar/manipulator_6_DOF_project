#include "gy85_pos.h"

// Funkcja pomocnicza do ograniczania wartości
static float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

HAL_StatusTypeDef IMU_InitPosition(IMU_Position *position) {
    // Zerowanie wszystkich wartości
    position->position.x = 0.0f;
    position->position.y = 0.0f;
    position->position.z = 0.0f;
    
    position->velocity.x = 0.0f;
    position->velocity.y = 0.0f;
    position->velocity.z = 0.0f;
    
    position->orientation.roll = 0.0f;
    position->orientation.pitch = 0.0f;
    position->orientation.yaw = 0.0f;
    
    position->last_update = HAL_GetTick();
    position->initialized = 1;
    
    return HAL_OK;
}

void IMU_ResetPosition(IMU_Position *position) {
    IMU_InitPosition(position);
}

static void updateOrientation(GY85_HandleTypeDef *hgy85, IMU_Position *position, float dt) {
    // Obliczanie kątów z akcelerometru
    float accel_roll = atan2f(hgy85->accel.y, hgy85->accel.z);
    float accel_pitch = atan2f(-hgy85->accel.x, 
                              sqrtf(hgy85->accel.y * hgy85->accel.y + 
                                   hgy85->accel.z * hgy85->accel.z));
    
    // Integracja danych z żyroskopu
    float gyro_roll = position->orientation.roll + hgy85->gyro.x * dt;
    float gyro_pitch = position->orientation.pitch + hgy85->gyro.y * dt;
    float gyro_yaw = position->orientation.yaw + hgy85->gyro.z * dt;
    
    // Fuzja danych - filtr komplementarny
    position->orientation.roll = COMPLEMENTARY_FILTER_ALPHA * gyro_roll + 
                               (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;
    position->orientation.pitch = COMPLEMENTARY_FILTER_ALPHA * gyro_pitch + 
                                (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch;
    position->orientation.yaw = gyro_yaw;
    
    // Normalizacja kąta yaw do zakresu -π do π
    while (position->orientation.yaw > M_PI) position->orientation.yaw -= 2.0f * M_PI;
    while (position->orientation.yaw < -M_PI) position->orientation.yaw += 2.0f * M_PI;
}

static void transformAcceleration(GY85_HandleTypeDef *hgy85, IMU_Position *position, 
                                float *ax, float *ay, float *az) {
    // Macierz rotacji z układu IMU do układu świata
    float cr = cosf(position->orientation.roll);
    float sr = sinf(position->orientation.roll);
    float cp = cosf(position->orientation.pitch);
    float sp = sinf(position->orientation.pitch);
    float cy = cosf(position->orientation.yaw);
    float sy = sinf(position->orientation.yaw);
    
    // Przekształcenie przyspieszenia z układu IMU do układu świata
    float ax_world = (cy * cp) * hgy85->accel.x + 
                    (cy * sp * sr - sy * cr) * hgy85->accel.y + 
                    (cy * sp * cr + sy * sr) * hgy85->accel.z;
                    
    float ay_world = (sy * cp) * hgy85->accel.x + 
                    (sy * sp * sr + cy * cr) * hgy85->accel.y + 
                    (sy * sp * cr - cy * sr) * hgy85->accel.z;
                    
    float az_world = (-sp) * hgy85->accel.x + 
                    (cp * sr) * hgy85->accel.y + 
                    (cp * cr) * hgy85->accel.z;
    
    // Filtrowanie małych przyspieszeń (redukcja szumu)
    *ax = fabsf(ax_world) < MIN_ACCEL_THRESHOLD ? 0.0f : 
          constrain(ax_world, -MAX_ACCEL_THRESHOLD, MAX_ACCEL_THRESHOLD);
    *ay = fabsf(ay_world) < MIN_ACCEL_THRESHOLD ? 0.0f : 
          constrain(ay_world, -MAX_ACCEL_THRESHOLD, MAX_ACCEL_THRESHOLD);
    *az = fabsf(az_world) < MIN_ACCEL_THRESHOLD ? 0.0f : 
          constrain(az_world, -MAX_ACCEL_THRESHOLD, MAX_ACCEL_THRESHOLD);
}

HAL_StatusTypeDef IMU_UpdatePosition(GY85_HandleTypeDef *hgy85, IMU_Position *position) {
    if (!position->initialized) {
        return IMU_InitPosition(position);
    }
    
    // Obliczenie czasu od ostatniej aktualizacji
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - position->last_update) / 1000.0f; // konwersja na sekundy
    position->last_update = current_time;
    
    if (dt <= 0.0f || dt > 1.0f) { // Za duża przerwa lub błąd czasu
        return HAL_ERROR;
    }
    
    // Aktualizacja orientacji
    updateOrientation(hgy85, position, dt);
    
    // Transformacja przyspieszenia do układu świata
    float ax_world, ay_world, az_world;
    transformAcceleration(hgy85, position, &ax_world, &ay_world, &az_world);
    
    // Aktualizacja prędkości z uwzględnieniem tłumienia
    position->velocity.x = position->velocity.x * VELOCITY_DECAY + ax_world * dt;
    position->velocity.y = position->velocity.y * VELOCITY_DECAY + ay_world * dt;
    position->velocity.z = position->velocity.z * VELOCITY_DECAY + az_world * dt;
    
    // Aktualizacja pozycji
    position->position.x += position->velocity.x * dt;
    position->position.y += position->velocity.y * dt;
    position->position.z += position->velocity.z * dt;
    
    return HAL_OK;
}