#include "tuw_rccar/tuw_rccar.h"
#include "tuw_rccar/serial_message.h"
#include <opencv/cv.h>
#include <boost/concept_check.hpp>

using namespace cv;
using namespace tuw;


RCCar::RCCar ( const std::string &ns ) {

    tuw::serial::Parameters params;
    params.port = "/dev/ttyACM0";
    params.baudrate = 115200;

    auto callback_fnc = std::bind ( &RCCar::callback_serial, this, std::placeholders::_1,  std::placeholders::_2 );

    serial_arduino.init(params, callback_fnc);

    service_ackermann_config = true; // get config on startup
    service_pid_controller = true; // get PID-config on startup

    imu_.orientation_covariance[0] = -1; // IMU has no orientation -> set covariance[0] to -1 according to sensor_msgs/Imu doc
}

RCCar::~RCCar (void) {
    serial_arduino.close();
}

void RCCar::init() {
    ROS_INFO("RCCar::init()");
}

void RCCar::callback_serial ( tuw::serial::Message &header, tuw::serial::Objects & objects ) {

    tuw::arduino::Pose pose;
    std::cout << header << std::endl;
    for ( tuw::serial::Objects::iterator it=objects.begin(); it!=objects.end(); ++it ) {
        tuw::serial::Object &object = it->second;
        switch ( it->first ) {
        case tuw::serial::TYPE_ACKERMANN_CONFIG:
            {
            object.get(amcfg_);
            printf("TYPE_ACKERMANN_CONFIG received:\n");
            printf("\tAxle Distance: %3.2f m\n", amcfg_.d);
            printf("\tWheelbase: %3.2f m\n", amcfg_.l);
            printf("\tWeight: %3.2f kg\n", amcfg_.weight);
            printf("\tSteering Angle: %4.3f rad\n", amcfg_.max_steer_angle);
            }
            break;
        case tuw::serial::TYPE_MOTOR_STATE:
            break;
        case tuw::serial::TYPE_MOTOR_PID:
            {
//            object.get(amcfg_);
            printf("TYPE_MOTOR_PID received:\n");
//            printf("\tAxle Distance: %3.2f m\n", amcfg_.d);
//            printf("\tWheelbase: %3.2f m\n", amcfg_.l);
//            printf("\tWeight: %3.2f kg\n", amcfg_.weight);
//            printf("\tSteering Angle: %4.3f rad\n", amcfg_.max_steer_angle);
            }
            break;
        case tuw::serial::TYPE_SERVO_STATE:
            break;
        case tuw::serial::TYPE_SERVO_PID:
            break;
        case tuw::serial::TYPE_MOTION_POSE_ESTIMATED:
            break;
        case tuw::serial::TYPE_MOTION_POSE_COVARIANCE_ESTIMATED:
            break;
        case tuw::serial::TYPE_IMU_ACCELEROMETER:
            {
                tuw::arduino::IMU_Accelerometer imu_acc;
                object.get(imu_acc);
                imu_.linear_acceleration.x = imu_acc.state.x;
                imu_.linear_acceleration.y = imu_acc.state.y;
                imu_.linear_acceleration.z = imu_acc.state.z;
                imu_last_received = ros::Time::now();
            }
            break;
        case tuw::serial::TYPE_IMU_GYROSCOPE:
            {
                tuw::arduino::IMU_Gyroscope imu_gyro;
                object.get(imu_gyro);
                imu_.angular_velocity.x = imu_gyro.state.x;
                imu_.angular_velocity.y = imu_gyro.state.y;
                imu_.angular_velocity.z = imu_gyro.state.z;
                imu_last_received = ros::Time::now();
            }
            break;
        case tuw::serial::TYPE_IMU_MAGNETOMETER:
            break;
        case tuw::serial::TYPE_IMU_ENVIRONMENT:
            {
//              tuw::arduino::IMU_Environment imu_env;
//              object.get(imu_env);
//              printf("IMU Environment Temperature = %2.1f °C\n", imu_env.state.x);
//              printf("IMU Environment Pressure = %3.2f hPa\n", imu_env.state.y);
//              printf("IMU Environment Compass = %3.0f °\n", imu_env.state.z);
            }
            break;
        case tuw::serial::TYPE_TEXT:
            std::cout << object;
            break;
        case tuw::serial::TYPE_POSE:
            object.get ( pose );
//            if(print_rx) std::cout << std::setw(30) << "Pose: " << pose << std::endl;
            break;
        default:
            break;
  //          if(print_rx) std::cout << std::setw(30) << "Type id: " << object.type << ", of size: " << object.size << std::endl;
        }
    }
    if (actuators_last_sent != actuators_last_received && (actuators_.rps != actuators_last_.rps || actuators_.rad != actuators_last_.rad) ) {
        serial_arduino.addObject ( tuw::serial::Object ( actuators_, tuw::serial::TYPE_COMMAND_ACTUATORS ) );
        actuators_last_sent = actuators_last_received; // NOT ros::Time::now()
        actuators_last_.rps = actuators_.rps;
        actuators_last_.rad = actuators_.rad;
//      if ( print_rx ) std::cout <<  std::setw ( 30 ) << shm_->command_actuators.name() << ": " << *shm_->command_actuators << std::endl;
    }

    if (service_ackermann_config) {
        service_ackermann_config = false;
        serial_arduino.addObject(tuw::serial::Object(NULL, tuw::serial::TYPE_COMMAND_ACKERMANN_CONFIG) );
    }

    if (service_pid_controller) {
        service_pid_controller = false;
        serial_arduino.addObject(tuw::serial::Object(NULL, tuw::serial::TYPE_COMMAND_MOTOR_PID) );
    }

/*    if(shm_->command_ackermann.hasChanged()){
      shm_->command_ackermann.lock();
      addObject ( tuw::serial::Object ( *shm_->command_ackermann, tuw::serial::TYPE_COMMAND_ACKERMANN ) );
      shm_->command_ackermann.dataProcessed();
      shm_->command_ackermann.unlock();
      if ( print_rx ) std::cout <<  std::setw ( 30 ) << shm_->command_ackermann.name() << ": " << *shm_->command_ackermann << std::endl;
    }*/

}
