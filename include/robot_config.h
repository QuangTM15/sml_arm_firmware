/**
 * @file    robot_config.h
 * @brief   Robot joint and motion configuration.
 * @project SML Arm Firmware
 * @version 0.1.0
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

namespace RobotConfig
{
    struct JointConfig
    {
        uint32_t pulsesPerRevolution;
        float gearRatio;
        float defaultSpeed;
        float defaultAcceleration;
        float linearAccelerationDistance;
    };

    // ================= ROBOT =================
    constexpr uint8_t MAX_JOINTS = 6U;
    constexpr uint8_t ACTIVE_JOINTS = 1U;
    // ================= JOINT 1 =================
    constexpr JointConfig JOINT_1 =
        {
            1600U,  // Motor pulses per revolution
            100.0f, // Gear ratio
            18.0f,  // Joint speed (deg/s)
            13.5f,  // Joint acceleration (deg/s^2)
            0.9f    // Linear acceleration distance (deg)
    };
}

#endif // ROBOT_CONFIG_H
