//
// Created by yangxiao on 2026/11/10.
//

#include "Arm.hpp"
#include "main.h"
#include <math.h>

constexpr float kLink1 = 30.0f;
constexpr float kLink2 = 125.0f;
constexpr float kRadToDeg = 57.2957795131f;
constexpr float kSquareLength = 200.0f;

Arm::Arm(Motor& motor1, Motor& motor2)
    : motor1_(motor1), motor2_(motor2) {}

float Arm::shortestAngleError(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

float Arm::calculateFeedforwardSpeedRpm(float currentTarget, float nextTarget, uint32_t intervalMs) {
    if (intervalMs == 0U) return 0.0f;
    const float delta = shortestAngleError(nextTarget, currentTarget);
    return delta * 1000.0f / static_cast<float>(intervalMs) / 6.0f;
}

void Arm::initializeTargetPositions() {
    for (uint16_t i = 0U; i < kTargetCount; ++i) {
        const uint16_t edge = i / kPointsPerEdge;
        const float edgeProgress = static_cast<float>(i % kPointsPerEdge) /
                                   static_cast<float>(kPointsPerEdge);
        const float offset = kSquareLength * edgeProgress;

        if (edge == 0U) targetXY[i] = {kSquareLength/2, -kSquareLength/2 + offset};
        else if (edge == 1U) targetXY[i] = {kSquareLength/2 - offset, kSquareLength/2};
        else if (edge == 2U) targetXY[i] = {-kSquareLength/2, kSquareLength/2 - offset};
        else targetXY[i] = {-kSquareLength/2 + offset, -kSquareLength/2};

        const float x = targetXY[i].x;
        const float y = targetXY[i].y;
        float cosTheta2 = (x * x + y * y - kLink1 * kLink1 - kLink2 * kLink2) /
                          (2.0f * kLink1 * kLink2);
        if (cosTheta2 > 1.0f) cosTheta2 = 1.0f;
        if (cosTheta2 < -1.0f) cosTheta2 = -1.0f;

        const float theta2 = acosf(cosTheta2);
        const float theta1 = atan2f(y, x) -
                             atan2f(kLink2 * sinf(theta2),
                                    kLink1 + kLink2 * cosf(theta2));
        thetaSet[0][i] = theta1 * kRadToDeg;
        thetaSet[1][i] = theta2 * kRadToDeg;
    }
}

void Arm::init() {
    if (initialized_) return;
    initializeTargetPositions();
    initialized_ = true;
}

void Arm::handler() {
    if (!initialized_) init();

    if (stopAll) {
        motor1_.stop();
        motor2_.stop();
        currentPosition1 = motor1_.angle_;
        currentPosition2 = motor2_.angle_;
        motionStarted_ = false;
        currentCycleMs = 0U;
        return;
    }

    const uint32_t now = HAL_GetTick();
    if (!motionStarted_) {
        lastPointTick_ = now;
        cycleStartTick_ = now;
        motionStarted_ = true;
    }
    currentCycleMs = now - cycleStartTick_;

    const uint16_t currentIndex = index;
    const uint16_t nextIndex = (currentIndex + 1U) % kTargetCount;
    const float motor1Target = -thetaSet[0][currentIndex];
    const float motor2Target = thetaSet[1][currentIndex];
    const float motor1NextTarget = -thetaSet[0][nextIndex];
    const float motor2NextTarget = thetaSet[1][nextIndex];
    const uint16_t edgePoint = currentIndex % kPointsPerEdge;
    const bool atCorner = edgePoint == 0U;
    const uint16_t pointsToCorner = edgePoint < (kPointsPerEdge - edgePoint) ?
                                    edgePoint : (kPointsPerEdge - edgePoint);
    currentPointIntervalMs = pointIntervalMs;
    if (cornerSlowZonePoints > 0U && pointsToCorner < cornerSlowZonePoints &&
        cornerMaxIntervalMs > pointIntervalMs) {
        const uint32_t extraInterval = cornerMaxIntervalMs - pointIntervalMs;
        currentPointIntervalMs += extraInterval *
            static_cast<uint32_t>(cornerSlowZonePoints - pointsToCorner) /
            static_cast<uint32_t>(cornerSlowZonePoints);
    }

    motor1FeedforwardSpeedRpm = atCorner ? 0.0f :
        calculateFeedforwardSpeedRpm(motor1Target, motor1NextTarget, currentPointIntervalMs);
    motor2FeedforwardSpeedRpm = atCorner ? 0.0f :
        calculateFeedforwardSpeedRpm(motor2Target, motor2NextTarget, currentPointIntervalMs);

    motor1_.setPosition(motor1Target, motor1FeedforwardSpeedRpm, 0.0f);
    motor2_.setPosition(motor2Target, motor2FeedforwardSpeedRpm, 0.0f);

    if (nextStep) {
        index = nextIndex;
        lastPointTick_ = now;
        nextStep = false;
    }

    motor1_.handle();
    motor2_.handle();
    currentPosition1 = motor1_.angle_;
    currentPosition2 = motor2_.angle_;

    motor1CornerReady =
        fabsf(shortestAngleError(motor1Target, motor1_.angle_)) < cornerPositionThresholdDeg &&
        fabsf(motor1_.outputSpeedRpm()) < cornerSpeedThresholdRpm;
    motor2CornerReady =
        fabsf(shortestAngleError(motor2Target, motor2_.angle_)) < cornerPositionThresholdDeg &&
        fabsf(motor2_.outputSpeedRpm()) < cornerSpeedThresholdRpm;
    cornerReady = motor1CornerReady && motor2CornerReady;

    const uint32_t currentPointInterval = currentPointIntervalMs +
                                          (atCorner ? cornerHoldMs : 0U);
    if ((now - lastPointTick_) >= currentPointInterval && (!atCorner || cornerReady)) {
        if (currentIndex == (kTargetCount - 1U)) {
            lastCycleMs = now - cycleStartTick_;
            lastCycleSeconds = static_cast<float>(lastCycleMs) / 1000.0f;
            completedCycleCount++;
            cycleStartTick_ = now;
            currentCycleMs = 0U;
        }
        index = nextIndex;
        lastPointTick_ = now;
    }

    targetPosition1 = -thetaSet[0][index];
    targetPosition2 = thetaSet[1][index];
}
