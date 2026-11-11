//
// Created by yangxiao on 2026/10/25.
//

#include "Motor.hpp"
#include "Arm.hpp"

volatile bool testreach = false;

constexpr float kM3508ReductionRatio = 3591.0f / 187.0f;

Motor motor1(24.0f / 28.0f,0.0f,0.065f, 1);
Motor motor2(1,0.0f,0.065f, 4);
Arm arm(motor1, motor2);


struct MotorPidConfig {
    MotorPidConfig() {
        // Motor 1 PID parameters.
        motor1.spid = PID{0.5f, 0.0f, 0.5f, 0.0f, 10.0f, 1.0f};
        motor1.ppid = PID{5.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.1f};

        // Motor 2 PID parameters.
        motor2.spid = PID{0.05f, 0.0f, 15.0f, 0.0f, 3.0f, 0.01f};
        motor2.ppid = PID{6.3f, 0.0f, 3.0f, 0.0f, 100.0f, 0.1f};
    }
};

MotorPidConfig motor_pid_config;

