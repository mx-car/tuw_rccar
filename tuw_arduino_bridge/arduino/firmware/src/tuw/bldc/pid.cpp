
#include "pid.h"

using namespace tuw;

PID::PID (){
    init();
}

void PID::init ( float kp, float ki, float kd, float output_min, float output_max ) {
    kp_ = kp, ki_ = ki, kd_ = kd, output_min_ = output_min, output_max_ = output_max;
    integral_ = 0;
}

float PID::compute ( float target, float measurment ) {

    output_last_ = output_now_;
    measurment_last_ = measurment_now_;

    measurment_now_ = measurment;
    target_ = target;

    error_ = target_ - measurment_now_;
    proportional_ = kp_ * error_;
    integral_ += ki_ * error_;
    derivative_ = kd_ * ( measurment_now_ - measurment_last_ );

    // Anti-Wind-Up
    if (integral_ > output_max_) integral_ = output_max_;
    if (integral_ < output_min_) integral_ = output_min_;

    output_now_ = proportional_ + integral_ - derivative_;
    if ( output_now_ > output_max_ ) output_now_ = output_max_;
    if ( output_now_ < output_min_ ) output_now_ = output_min_;
    return output_now_;
}

