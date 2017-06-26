#ifndef TUW_SHM_OBJECTS_H
#define TUW_SHM_OBJECTS_H


#include <tuw/arduino/arduino_objects.h>
#include <shmfw/variable.h>
#include <shmfw/vector.h>

namespace tuw {
namespace shm {


struct Parameters {
    bool clear;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
};


struct Objects {
public:
    Objects ();
    Objects (const Parameters &param);
    void init(const Parameters &param);

    ShmFw::HandlerPtr shmHdl;

    /// rx elements to receive
    ShmFw::Var<tuw::arduino::Text> message;
    ShmFw::Var<tuw::arduino::Pose> pose_estimated;
    ShmFw::Var<tuw::arduino::Mat3x3> pose_covariance_estimated;
    ShmFw::Var<tuw::arduino::AckermannConfig> ackermann_config;
    ShmFw::Var<tuw::arduino::Motor> motor_state;
    ShmFw::Var<tuw::arduino::PID> motor_pid;
    ShmFw::Var<tuw::arduino::Servo> servo_state;
    ShmFw::Var<tuw::arduino::PID> servo_pid;
    ShmFw::Var<tuw::arduino::IMU_Accelerometer> imu_accelerometer;
    ShmFw::Var<tuw::arduino::IMU_Gyroscope> imu_gyroscope;
    ShmFw::Var<tuw::arduino::IMU_Magnetometer> imu_magnetometer;
    ShmFw::Var<tuw::arduino::IMU_Environment> imu_environment;

    /// tx elements to send
    ShmFw::Var<tuw::arduino::Actuators> command_actuators;
    ShmFw::Var<tuw::arduino::Actuators> command_ackermann; // TODO: datatype


    /// debug
    ShmFw::Vector<double> oszi_motor;

};
};
};
#endif // TUW_SHM_OBJECTS_H
