/**
* @author Markus Bader <markus.bader@tuwien.ac.at>
* @license Simplified BSD License
*/


#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include <tuw/serial/serial_structs.h>

namespace tuw {
class BLDC;
class Servos;
class IMU;

class Motion  {
public:

    Motion();
    /**
     * @cmd command as string
     * @response response encoded as string
     * @return 0 on success
     **/
    int command ( const char *cmd, char *response );
    /**
     * init system
     * @param hall_a pin
     * @param hall_b
     * @param hall_c
     * @param pwm_a
     * @param pwm_b
     * @param pwm_c
     * @param dir_a
     * @param dir_b
     * @param dir_c
     * @param servo
     * @param rps   rotation per seconds (default = 0.0)
     **/
    void init ( int hall_a, int hall_b, int hall_c, int pwm_a, int pwm_b, int pwm_c, int dir_a, int dir_b, int dir_c, int servo, float rps = 0.0 );
    void setActuators ( float rps, float rad );
    void setMotorPID ( float kp, float ki, float kd );
    void filter_servos ();
    void update_pose ();
    void update_objects ();
    tuw::arduino::TAckermannConfig ackermann_config_;
    tuw::arduino::TPose pose_;
    tuw::arduino::TMat3x3 pose_cov_;
    tuw::arduino::TMotor motor_state_;
    tuw::arduino::TPID motor_pid_;
    tuw::arduino::TServo servo_state_;
    tuw::arduino::TPID servo_pid_;
    tuw::arduino::TIMU_Accelerometer imu_accelerometer_;
    tuw::arduino::TIMU_Gyroscope imu_gyroscope_;
    tuw::arduino::TIMU_Magnetometer imu_magnetometer_;
    tuw::arduino::TIMU_Environment imu_environment_;
private:
    BLDC *bldc_;
    Servos *servos_;
    IMU *imu_;
    int servo_pin_;
    float servo_filtered_, servo_target_;
};

}
#endif

