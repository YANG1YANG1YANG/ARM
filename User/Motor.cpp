//
// Created by yangxiao on 2026/10/3.
//
#include "Motor.hpp"
#include "main.h"

uint8_t Motor::motor_tx_data_[8] = {};

Motor::Motor(float ratio,float load_mass, float load_dist, uint8_t motor_id):
    spid{0.0f,0.0f,0.0f,0.0f,1.0f,0.1f},
    ppid{0.0f,0.0f,0.0f,0.0f,18.0f,0.1f},

    motor_id_(motor_id), ratio_(ratio), load_mass_(load_mass), load_dist_(load_dist) {}

void Motor::canRxMsgCallback() {
    processData();
}

void Motor::processData() {
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    memcpy(rx_data_,rx_buff_,sizeof(rx_data_));
    has_feedback_ = false;
    __set_PRIMASK(saved_primask);
    ecd_angle_ = static_cast<float>(rx_data_[0] << 8 | rx_data_[1]);
    if (received_ == false) {
        received_ = true;
        last_ecd_ = ecd_angle_;
    } else {
        ecd_angle_ = static_cast<float>(rx_data_[0] << 8 | rx_data_[1]);
        float delta_ecd = ecd_angle_ - last_ecd_;
        delta_ecd = wrapEncoderDelta(delta_ecd);
        angle_ += scale(delta_ecd, kEncoderRange, kAngleScale) / ratio_;
        last_ecd_ = ecd_angle_;
        speedRpm_ = static_cast<float>(static_cast<int16_t>(rx_data_[2] << 8 | rx_data_[3]));
        current_speed_ = speedRpm_/ratio_;
    }
    // int16_t current_raw = static_cast<int16_t>(rx_data_[4] << 8 | rx_data_[5]);
    // currentA_ = scale(current_raw, kCurrentScale, kAmperesMax);
    // tempC_ = static_cast<float>(rx_data_[6]);
    int16_t torque_raw = static_cast<int16_t>(
        (static_cast<uint16_t>(rx_data_[4]) << 8) |
         static_cast<uint16_t>(rx_data_[5])
    );

    torque_ = static_cast<float>(torque_raw) / 16384.0f * 20.0f;
    updateReachedTarget();
}


// 出口：控制逻辑读取
float Motor::angle() {
    return angle_;
}

float Motor::speedRpm() {
    return speedRpm_;
}

float Motor::outputSpeedRpm() {
    return current_speed_;
}

bool Motor::feedbackReceived() const {
    return received_;
}

bool Motor::feedbackFresh(uint32_t now, uint32_t timeout_ms) const {
    return has_feedback_ && (now - last_feedback_tick_ <= timeout_ms);
}
// float Motor::currentAmps() {
//     return currentA_;
// }
// float Motor::temperatureC() {
//     return tempC_;
// }


void Motor::setTxCurrent(float amperes) {
    uint8_t offset = motor_id_ - 1;
    amperes = limit(amperes, -kCurrentCommandLimitAmps, kCurrentCommandLimitAmps);
    int16_t current = static_cast<int16_t>(scale(amperes, kProtocolCurrentFullScaleAmps, kCurrentScale));
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();

	motor_tx_data_[0 + offset * 2] = static_cast<uint8_t>(current >> 8);
    motor_tx_data_[1 + offset * 2] = static_cast<uint8_t>(current);
    __set_PRIMASK(saved_primask);
}

// 命令出口：TIM6 中断取用（HAL 形参非 const）
uint8_t* Motor::getTxData() {
    return motor_tx_data_;
}

void Motor::handle() {
    if (has_feedback_) {
        canRxMsgCallback();
    }
    if (fabsf(current_speed_) > kOverspeedThresholdRpm) {
        //faulted_ = true;
    }
    if (faulted_) {
        stop();
        return;
    }
    switch (controlMethod_) {
        case POSITION_SPEED:
        //setPosition(target_angle_,feedforward_speed_,feedforward_intensity_);
        Position();
        break;
        case SPEED:
        //setSpeed(feedforward_speed_,feedforward_intensity_);
        Speed();
        break;
        case TORQUE:
        //setIntensity(intensity_);
        Intensity();
        break;
        default:
        break;
    }
    if (!testreach) {
        setTxCurrent(intensity_);
    }
}

void Motor::calcGravityFeedforwardIntensity() {
    feedforward_intensity_=coeff_feedforwardintensity_*(sin(angle_*RAD2DEG))*9.8f*load_dist_*load_mass_;
}

void Motor::Position() {
    //reached_target_ = false;
    target_speed_ = limit(ppid.calc(target_angle_,angle_) + feedforward_speed_,
                          -kMaxTargetSpeedRpm, kMaxTargetSpeedRpm);
    intensity_=spid.calc(target_speed_,current_speed_)+feedforward_intensity_;
}

void Motor::Speed() {
    intensity_=spid.calc(target_speed_,current_speed_)+feedforward_intensity_;
}

void Motor::Intensity() {
    controlMethod_ = TORQUE;
}

void Motor::stop() {
    if (has_feedback_) {
        canRxMsgCallback();
    }
    target_speed_ = 0.0f;
    feedforward_speed_ = 0.0f;
    feedforward_intensity_ = 0.0f;
    intensity_=0.0f;
    controlMethod_ = TORQUE;
    ppid.reset();
    spid.reset();
    setTxCurrent(intensity_);
}


float Motor::limit(float value, float min, float max) {
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    }
    return value;
}

float Motor::scale(float x, float in_scale, float out_scale) {
    return x / in_scale * out_scale;
}

float Motor::wrapEncoderDelta(float delta_ecd) {
    if (delta_ecd > kEncoderWrapThreshold)
        delta_ecd -= kEncoderRange;
    else if (delta_ecd < -kEncoderWrapThreshold)
        delta_ecd += kEncoderRange;
    return delta_ecd;
}

void Motor::setPosition(float target_position,float feedforward_speed,float feedforward_intensity) {
    if (faulted_) return;
    if (controlMethod_ != POSITION_SPEED) {
        ppid.reset();
        spid.reset();
    }
    float target_delta = target_position - angle_;
    while (target_delta > 180.0f) target_delta -= 360.0f;
    while (target_delta < -180.0f) target_delta += 360.0f;
    const float nearest_target = angle_ + target_delta;
    if (fabsf(nearest_target - target_angle_) > 1.0f) {
        reached_stable_count_ = 0;
        reached_target_ = false;
    }
    target_angle_ = nearest_target;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
    controlMethod_ = POSITION_SPEED;
}
void Motor::setSpeed(float target_speed, float feedforward_intensity) {
    if (faulted_) return;
    if (controlMethod_ != SPEED) {
        spid.reset();
    }
    target_speed_ = limit(target_speed, -kMaxTargetSpeedRpm, kMaxTargetSpeedRpm);
    feedforward_intensity_ = feedforward_intensity;
    controlMethod_ = SPEED;
}
void Motor::setIntensity(float intensity) {
    if (faulted_) return;
    intensity_ = limit(intensity, -kCurrentCommandLimitAmps, kCurrentCommandLimitAmps);
    controlMethod_ = TORQUE;
}

bool Motor::faulted() const {
    return faulted_;
}

void Motor::updateReachedTarget() {
    // if (reached_target_) return;
    // if (controlMethod_ == POSITION_SPEED &&
    //     fabsf(target_angle_ - angle_) < kReachedPositionThreshold &&
    //     fabsf(current_speed_) < kReachedSpeedThresholdRpm) {
    // if (controlMethod_ == POSITION_SPEED &&
    //      fabsf(target_angle_ - angle_) < kReachedPositionThreshold) {
    if (fabsf(target_angle_ - angle_) < kReachedPositionThreshold) {
        if (reached_stable_count_ < kReachedStableSamples) {
            reached_stable_count_++;
        }
    }
    else {
        reached_stable_count_ = 0;
    }
    reached_target_ = reached_stable_count_ >= kReachedStableSamples;
}
