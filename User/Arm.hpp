//
// Created by yangxiao on 2026/11/10.
//

#ifndef CAN_ARM_H
#define CAN_ARM_H

#include "Motor.hpp"
#include <stdint.h>

struct Point {
    float x;
    float y;
};

class Arm {
public:
    static constexpr uint16_t kTargetCount = 800U;
    static constexpr uint16_t kPointsPerEdge = kTargetCount / 4U;

    Arm(Motor& motor1, Motor& motor2);

    void init();
    void handler();

    Point targetXY[kTargetCount] = {};
    float thetaSet[2][kTargetCount] = {};

    volatile bool stopAll = true;
    volatile bool nextStep = false;
    volatile uint16_t index = 0U;
    volatile float targetPosition1 = 0.0f;
    volatile float targetPosition2 = 0.0f;
    volatile float currentPosition1 = 0.0f;
    volatile float currentPosition2 = 0.0f;
    volatile uint32_t pointIntervalMs = 10U;
    volatile uint32_t cornerMaxIntervalMs = 30U;
    volatile uint16_t cornerSlowZonePoints = 20U;
    volatile uint32_t currentPointIntervalMs = 10U;
    volatile uint32_t cornerHoldMs = 0U;
    volatile float cornerPositionThresholdDeg = 2.0f;
    volatile float cornerSpeedThresholdRpm = 1.0f;
    volatile bool motor1CornerReady = false;
    volatile bool motor2CornerReady = false;
    volatile bool cornerReady = false;
    volatile float motor1FeedforwardSpeedRpm = 0.0f;
    volatile float motor2FeedforwardSpeedRpm = 0.0f;
    volatile uint32_t currentCycleMs = 0U;
    volatile uint32_t lastCycleMs = 0U;
    volatile float lastCycleSeconds = 0.0f;
    volatile uint32_t completedCycleCount = 0U;

private:
    Motor& motor1_;
    Motor& motor2_;
    bool initialized_ = false;
    bool motionStarted_ = false;
    uint32_t lastPointTick_ = 0U;
    uint32_t cycleStartTick_ = 0U;

    static float shortestAngleError(float target, float current);
    static float calculateFeedforwardSpeedRpm(float currentTarget, float nextTarget, uint32_t intervalMs);
    void initializeTargetPositions();
};

#endif // CAN_ARM_H
