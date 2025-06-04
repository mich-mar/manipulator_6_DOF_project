#ifndef POSITION_TRACKER_H
#define POSITION_TRACKER_H

#include "main.h"
#include "GY85.h"
#include "kalmanFilter.h"
#include <math.h>
#include <stdio.h>
#include "usb_device.h"

typedef struct {
    // Filtr Kalmana
    KalmanFilter kf;
    
    // Offsety czujników
    GY85_ScaledData accel_offset;
    GY85_ScaledData gyro_offset;
    
    // Kalibracja
    uint16_t calibration_samples;
    uint8_t is_calibrated;
    
    // Stan spoczynku
    uint16_t static_samples;
    uint8_t is_static;
} PositionTracker;

void position_tracker_init(PositionTracker *tracker);
void position_tracker_reset(PositionTracker *tracker);
void position_tracker_update(PositionTracker *tracker, const GY85_HandleTypeDef *gy85_data);
void position_tracker_send_data(const PositionTracker *tracker);

#endif // POSITION_TRACKER_H