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
            break;
        case tuw::serial::TYPE_MOTOR_STATE:
            break;
        case tuw::serial::TYPE_MOTOR_PID:
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
            break;
        case tuw::serial::TYPE_IMU_GYROSCOPE:
            break;
        case tuw::serial::TYPE_IMU_MAGNETOMETER:
            break;
        case tuw::serial::TYPE_IMU_ENVIRONMENT:
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
    if(1 /*shm_->command_actuators.hasChanged()*/){
        tuw::arduino::Actuators _actuators;
        _actuators.rps = twist_velocity;
        _actuators.rad = twist_steering_angle;

        serial_arduino.addObject ( tuw::serial::Object ( _actuators, tuw::serial::TYPE_COMMAND_ACTUATORS ) );
//      if ( print_rx ) std::cout <<  std::setw ( 30 ) << shm_->command_actuators.name() << ": " << *shm_->command_actuators << std::endl;
    }

/*    if(shm_->command_ackermann.hasChanged()){
      shm_->command_ackermann.lock();
      addObject ( tuw::serial::Object ( *shm_->command_ackermann, tuw::serial::TYPE_COMMAND_ACKERMANN ) );
      shm_->command_ackermann.dataProcessed();
      shm_->command_ackermann.unlock();
      if ( print_rx ) std::cout <<  std::setw ( 30 ) << shm_->command_ackermann.name() << ": " << *shm_->command_ackermann << std::endl;
    }*/

}
