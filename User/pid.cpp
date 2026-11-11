//
// Created by yangxiao on 2026/10/11.
//
//
// Created by yangxiao on 2026/10/11.
//

#include "pid.h"

PID::PID(float kp, float ki, float kd, float i_max,float out_max,float d_filter_k)
: kp_(kp), ki_(ki), kd_(kd), d_filter_k_(d_filter_k), i_max_(i_max), out_max_(out_max) {
    reset();
}

void PID::reset() {
    integral_ = 0.0f;
    last_error_ = 0.0f;
    dout_ = 0.0f;
    kout_ = 0.0f;
    iout_ = 0.0f;
    output_ = 0.0f;
    dout_now_ = 0.0f;
    error_ = 0.0f;
    d_error_ = 0.0f;
    last_fdb_ = 0.0f;
    d_fdb_ = 0.0f;
    derivative_initialized_ = false;
}

float PID::calc(float ref,float fdb) {
    last_error_ = error_;
    error_ = ref-fdb;
    kout_ = kp_ * error_;
    integral_ += error_;
    if (integral_ > i_max_) integral_ = i_max_;
    else if (integral_ < -i_max_) integral_ = -i_max_;
    iout_ = ki_ * integral_;
    //filter after kd
    // float error_d = error - last_error_;
    // dout_now_ = - kd_ * error_d;
    // dout_ = d_filter_k_ * dout_now_ + (1 - d_filter_k_) * dout_;
    //filter before kd
    //d_error_ = d_filter_k_ * (error_-last_error_) + (1-d_filter_k_) * d_error_;
    if (!derivative_initialized_) {
        last_fdb_ = fdb;
        d_fdb_ = 0.0f;
        derivative_initialized_ = true;
    } else {
        d_fdb_ = d_filter_k_ * (fdb-last_fdb_) + (1-d_filter_k_) * d_fdb_;
        last_fdb_ = fdb;
    }
    // dout_ = -kd_ * d_error_;
    dout_ = -kd_ * d_fdb_;
    output_ = kout_ + iout_ + dout_;
    if (output_ > out_max_) output_ = out_max_;
    else if (output_ < -out_max_) output_ = -out_max_;
    return output_;
}

bool PID::isReached() {
    return (fabs(error_) < THRESHOLD);
}
