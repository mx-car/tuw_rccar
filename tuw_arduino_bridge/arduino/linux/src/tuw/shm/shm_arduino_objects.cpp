#include <tuw_shm/shm_arduino_objects.h>

using namespace tuw::shm;

Objects::Objects() {};
Objects::Objects ( const Parameters &param ) {
    init ( param);
}
void Objects::init ( const Parameters &param ) {
    if ( param.clear ) {
        ShmFw::Handler::removeSegment ( param.shm_memory_name );
        std::cout << "Shared Memory " << param.shm_memory_name << " cleared" << std::endl;
    }
    shmHdl = ShmFw::Handler::create ( param.shm_memory_name, param.shm_memory_size );

    /// Inputs
    message.construct ( "message", shmHdl );
    pose_estimated.construct ( "pose_estimated", shmHdl );
    pose_covariance_estimated.construct ( "pose_covariance_estimated", shmHdl );
    ackermann_config.construct ( "ackermann_config", shmHdl );
    motor_state.construct ( "motor_state", shmHdl );
    motor_pid.construct ( "motor_pid", shmHdl );
    servo_state.construct ( "servo_state", shmHdl );
    servo_pid.construct ( "servo_pid", shmHdl );
    imu_accelerometer.construct ( "imu_accelerometer", shmHdl );
    imu_gyroscope.construct ( "imu_gyroscope", shmHdl );
    imu_magnetometer.construct ( "imu_magnetometer", shmHdl );
    imu_environment.construct ( "imu_environment", shmHdl );


    /// Outputs
    command_actuators.construct ( "command_actuators", shmHdl );
    command_ackermann.construct ( "command_ackermann", shmHdl );


    /// Debug
    oszi_motor.construct("oszi_motor", shmHdl);
}
