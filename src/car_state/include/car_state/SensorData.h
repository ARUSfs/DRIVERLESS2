/*
 * SensorData.h
 *
 *  Created on: Jan 29, 2025
 *      Author: carme
 */
#include <stdint.h>
#ifndef SRC_SENSORDATA_H_
#define SRC_SENSORDATA_H_

typedef struct {
    float roll, pitch, yaw;
    int32_t longitude, latitude;
    float acceleration_x, acceleration_y, acceleration_z;
    float speed_x, speed_y, speed_z;
    float angular_x, angular_y, angular_z;
    float wheelspeed_fl, wheelspeed_fr, wheelspeed_rl, wheelspeed_rr;
    float steering_angle;
    uint16_t apps, front_brake, rear_brake;
    uint16_t current, voltage;
    float m_speed;

} SensorData;


#endif /* SRC_SENSORDATA_H_ */
