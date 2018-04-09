/**
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @license Simplified BSD License
 */


#include "motion.h"
#include "vehicle_types.h"
#include <tuw/bldc/bldc.h>
#include <tuw/servos/servos.h>
#include <tuw/imu/imu.h>

using namespace tuw;

Motion::Motion()
    : bldc_ ( NULL )
    , servos_ ( NULL )
    , imu_ ( NULL ) {
    ackermann_config_.l = ACKERMANN_CONFIG_L;
    ackermann_config_.d = ACKERMANN_CONFIG_D;
    ackermann_config_.gear_ratio = ACKERMANN_CONFIG_GEAR_RATIO;
    ackermann_config_.wheel_radius = ACKERMANN_CONFIG_WHEEL_RADIUS;
    ackermann_config_.max_steer_angle = ACKERMANN_CONFIG_MAX_STEER_ANGLE;
    ackermann_config_.weight = ACKERMANN_CONFIG_WEIGHT;
    servo_filtered_ = 0.0;
    servo_target_ = 0.0;
}

int Motion::command ( const char *cmd, char *response ) {

}

void Motion::init ( int hall_a, int hall_b, int hall_c, int pwm_a, int pwm_b, int pwm_c, int dir_a, int dir_b, int dir_c, int servo_pin, float rps ) {
    if ( bldc_ == NULL ) {
        bldc_ = &tuw::BLDC::getInstance();
        bldc_->init ( hall_a, hall_b, hall_c, pwm_a, pwm_b, pwm_c, dir_a, dir_b, dir_c );
        bldc_->set_rps ( rps );
    }
    if ( servos_ == NULL ) {
        servo_pin_ = servo_pin;
        servos_ = &tuw::Servos::getInstance();
        servos_->init ( TCC1 );
        servos_->setMicroSecondsPerRad ( ACTUATOR_CONFIG_SERVO_US_PER_RAD, servo_pin_ );
        servos_->enable ( servo_pin_ );
        servos_->setNeutral ( ACTUATOR_CONFIG_SERVO_NEUTRAL, servo_pin_ );
    }
    imu_ = &tuw::IMU::getInstance();
    imu_->init();
}

void Motion::setActuators ( float rps, float rad ) {
    bldc_->set_rps ( -rps );
    if (rad < ACTUATOR_CONFIG_SERVO_RAD_MIN) rad = ACTUATOR_CONFIG_SERVO_RAD_MIN;
    if (rad > ACTUATOR_CONFIG_SERVO_RAD_MAX) rad = ACTUATOR_CONFIG_SERVO_RAD_MAX;
    servo_target_ = rad;
}

void Motion::setMotorPID ( float kp, float ki, float kd ) {
    bldc_->pid().Kp() = kp;
    bldc_->pid().Ki() = ki;
    bldc_->pid().Kd() = kd;
}

void Motion::filter_servos () {
    servo_filtered_ = servo_target_; //0.95 * servo_filtered_ + 0.05 * servo_target_;
    servos_->setRad ( ACTUATOR_CONFIG_SERVO_DIRECTION * servo_filtered_, servo_pin_ );
}

void Motion::update_pose () {
    static int millis_last = millis();
    float delta_t = (millis() - millis_last) / 1000.0;
    millis_last = millis();

    if (motor_state_.rps.actual == INFINITY) return;

    float vel = delta_t * motor_state_.rps.actual * (2 * ACKERMANN_CONFIG_WHEEL_RADIUS * M_PI) / ACKERMANN_CONFIG_GEAR_RATIO;

    pose_.x += vel * cos(pose_.theta);
    pose_.y += vel * sin(pose_.theta);
    pose_.theta += vel * tan(servo_state_.angle.actual) / ACKERMANN_CONFIG_D;
    if (pose_.theta < M_PI) pose_.theta += 2 * M_PI;
    if (pose_.theta > M_PI) pose_.theta -= 2 * M_PI;
}

void Motion::update_objects () {
    imu_->update();

    motor_state_.pwm.actual = bldc_->get_pwm();
    //motor_state_.pwm.target = bldc_->get_pwm();
    motor_state_.rps.actual = bldc_->get_rps();
    motor_state_.rps.target = bldc_->get_rps_target();
    motor_state_.force.actual = bldc_->get_force();

    motor_pid_.kp = bldc_->pid().Kp();
    motor_pid_.ki = bldc_->pid().Ki();
    motor_pid_.kd = bldc_->pid().Kd();

    servo_state_.angle.actual = servos_->getRad ( servo_pin_ );
    //servo_state_.angle.target = servos_->getRad(servo_pin_);
    servo_state_.high_time.actual = servos_->getHighTime ( servo_pin_ );
    //servo_state_.high_time.target = servos_->getHighTime(servo_pin_);

    update_pose();

    imu_accelerometer_.state.x = imu_->get_accel(AXIS_X) / 100.0;
    imu_accelerometer_.state.y = imu_->get_accel(AXIS_Y) / 100.0;
    imu_accelerometer_.state.z = imu_->get_accel(AXIS_Z) / 100.0;

    imu_gyroscope_.state.x = imu_->get_gyro(AXIS_X);
    imu_gyroscope_.state.y = imu_->get_gyro(AXIS_Y);
    imu_gyroscope_.state.z = imu_->get_gyro(AXIS_Z);

    imu_magnetometer_.state.x = imu_->get_magnet(AXIS_X) / 1000.0;
    imu_magnetometer_.state.y = imu_->get_magnet(AXIS_Y) / 1000.0;
    imu_magnetometer_.state.z = imu_->get_magnet(AXIS_Z) / 1000.0;

    imu_environment_.state.x = imu_->get_temperature() / 10.0;
    imu_environment_.state.y = imu_->get_pressure() / 100.0;
    imu_environment_.state.z = imu_->get_compass();
}
