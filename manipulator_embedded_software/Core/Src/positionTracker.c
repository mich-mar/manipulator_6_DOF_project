#include "positionTracker.h"

// Stałe konfiguracyjne
#define CALIBRATION_SAMPLES 100
#define STATIC_ACCEL_THRESHOLD 0.05f
#define STATIC_SAMPLES_THRESHOLD 20

void position_tracker_init(PositionTracker *tracker) {
    // Inicjalizacja filtru Kalmana
    kalman_filter_init(&tracker->kf);
    
    // Zerowanie offsetów
    memset(&tracker->accel_offset, 0, sizeof(GY85_ScaledData));
    memset(&tracker->gyro_offset, 0, sizeof(GY85_ScaledData));
    
    // Inicjalizacja stanu
    tracker->calibration_samples = 0;
    tracker->is_calibrated = 0;
    tracker->static_samples = 0;
    tracker->is_static = 0;
}

void position_tracker_reset(PositionTracker *tracker) {
    // Reset filtru Kalmana
    kalman_filter_init(&tracker->kf);
    
    // Reset stanu spoczynku
    tracker->static_samples = 0;
    tracker->is_static = 0;
}

static void calibrate_sensors(PositionTracker *tracker, const GY85_HandleTypeDef *gy85_data) {
    if (tracker->calibration_samples < CALIBRATION_SAMPLES) {
        // Akumulacja pomiarów
        tracker->accel_offset.x += gy85_data->accel.x;
        tracker->accel_offset.y += gy85_data->accel.y;
        tracker->accel_offset.z += gy85_data->accel.z - 9.81f;
        
        tracker->gyro_offset.x += gy85_data->gyro.x;
        tracker->gyro_offset.y += gy85_data->gyro.y;
        tracker->gyro_offset.z += gy85_data->gyro.z;
        
        tracker->calibration_samples++;
        
        if (tracker->calibration_samples == CALIBRATION_SAMPLES) {
            // Obliczenie średnich offsetów
            float inv_samples = 1.0f / CALIBRATION_SAMPLES;
            tracker->accel_offset.x *= inv_samples;
            tracker->accel_offset.y *= inv_samples;
            tracker->accel_offset.z *= inv_samples;
            
            tracker->gyro_offset.x *= inv_samples;
            tracker->gyro_offset.y *= inv_samples;
            tracker->gyro_offset.z *= inv_samples;
            
            tracker->is_calibrated = 1;
            
            // Debug info
            char debug[128];
            snprintf(debug, sizeof(debug), 
                    "Calibration complete\r\nAccel offset: %.3f,%.3f,%.3f\r\nGyro offset: %.3f,%.3f,%.3f\r\n",
                    tracker->accel_offset.x, tracker->accel_offset.y, tracker->accel_offset.z,
                    tracker->gyro_offset.x, tracker->gyro_offset.y, tracker->gyro_offset.z);
            sendUSBmsg(debug);
        }
    }
}

void position_tracker_update(PositionTracker *tracker, const GY85_HandleTypeDef *gy85_data) {
    // Kalibracja jeśli potrzebna
    if (!tracker->is_calibrated) {
        calibrate_sensors(tracker, gy85_data);
        return;
    }
    
    // Kompensacja offsetów
    float accel_x = gy85_data->accel.x - tracker->accel_offset.x;
    float accel_y = gy85_data->accel.y - tracker->accel_offset.y;
    float accel_z = gy85_data->accel.z - tracker->accel_offset.z;
    
    float gyro_x = gy85_data->gyro.x - tracker->gyro_offset.x;
    float gyro_y = gy85_data->gyro.y - tracker->gyro_offset.y;
    float gyro_z = gy85_data->gyro.z - tracker->gyro_offset.z;
    
    // Wykrywanie stanu spoczynku
    float accel_magnitude = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    float gyro_magnitude = sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
    
    if (fabsf(accel_magnitude - 9.81f) < STATIC_ACCEL_THRESHOLD && gyro_magnitude < 0.1f) {
        tracker->static_samples++;
        if (tracker->static_samples > STATIC_SAMPLES_THRESHOLD) {
            tracker->is_static = 1;
        }
    } else {
        tracker->static_samples = 0;
        tracker->is_static = 0;
    }
    
    // Aktualizacja filtru Kalmana
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - tracker->kf.last_update) / 1000.0f;
    tracker->kf.last_update = current_time;
    
    if (dt > 0.5f || dt <= 0.0f) return;
    
    // Predykcja
    kalman_filter_predict(&tracker->kf, dt);
    
    // Aktualizacja tylko jeśli nie jesteśmy w stanie spoczynku
    if (!tracker->is_static) {
        float measurements[3] = {accel_x, accel_y, accel_z};
        kalman_filter_update(&tracker->kf, measurements);
    }
}

void position_tracker_send_data(const PositionTracker *tracker) {
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Ograniczenie częstotliwości wysyłania do 50Hz
    if (current_time - last_print_time < 20) return;
    last_print_time = current_time;
    
    char buffer[128];
    size_t len = snprintf(buffer, sizeof(buffer), 
        "POS,%.3f,%.3f,%.3f,VEL,%.3f,%.3f,%.3f,STATIC,%d\r\n",
        tracker->kf.state[0], tracker->kf.state[1], tracker->kf.state[2],  // pozycja
        tracker->kf.state[3], tracker->kf.state[4], tracker->kf.state[5],  // prędkość
        tracker->is_static);
    
    sendUSBmsg(buffer);
}