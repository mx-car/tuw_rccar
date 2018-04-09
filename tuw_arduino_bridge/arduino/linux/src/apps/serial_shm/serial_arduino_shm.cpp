#include "serial_arduino_shm.h"
#include <tuw_shm/shm_arduino_objects.h>


SerialArduinoShm::SerialArduinoShm ( Parameters &params ) {
    using namespace std::placeholders;

    print_rx = params.print_rx;

    shm_ = std::shared_ptr<tuw::shm::Objects>(new tuw::shm::Objects(params.shm));
    auto callback_fnc = std::bind ( &SerialArduinoShm::callback_serial, this, _1,  _2 );
    init (params.serial, callback_fnc );

};

void SerialArduinoShm::callback_serial ( tuw::serial::Message &header,  tuw::serial::Objects & objects ) {
    tuw::arduino::Pose pose;
    std::cout << header << std::endl;
    for ( tuw::serial::Objects::iterator it=objects.begin(); it!=objects.end(); ++it ) {
        tuw::serial::Object &object = it->second;
        switch ( it->first ) {
        case tuw::serial::TYPE_ACKERMANN_CONFIG:
	    copy2shm (object, shm_->ackermann_config);
            break;
        case tuw::serial::TYPE_MOTOR_STATE:
	    copy2shm (object, shm_->motor_state);
            break;
        case tuw::serial::TYPE_MOTOR_PID:
	    copy2shm (object, shm_->motor_pid);
            break;
        case tuw::serial::TYPE_SERVO_STATE:
	    copy2shm (object, shm_->servo_state);
            break;
        case tuw::serial::TYPE_SERVO_PID:
	    copy2shm (object, shm_->servo_pid);
            break;
        case tuw::serial::TYPE_MOTION_POSE_ESTIMATED:
	    copy2shm (object, shm_->pose_estimated);
            break;
        case tuw::serial::TYPE_MOTION_POSE_COVARIANCE_ESTIMATED:
	    copy2shm (object, shm_->pose_covariance_estimated);
            break;
        case tuw::serial::TYPE_IMU_ACCELEROMETER:
	    copy2shm (object, shm_->imu_accelerometer);
            break;
        case tuw::serial::TYPE_IMU_GYROSCOPE:
	    copy2shm (object, shm_->imu_gyroscope);
            break;
        case tuw::serial::TYPE_IMU_MAGNETOMETER:
	    copy2shm (object, shm_->imu_magnetometer);
            break;
        case tuw::serial::TYPE_IMU_ENVIRONMENT:
	    copy2shm (object, shm_->imu_environment);
            break;
        case tuw::serial::TYPE_TEXT:
	    copy2shm (object, shm_->message);
            break;
        case tuw::serial::TYPE_POSE:
            object.get ( pose );
            if(print_rx) std::cout << std::setw(30) << "Pose: " << pose << std::endl;
            break;
        default:
            if(print_rx) std::cout << std::setw(30) << "Type id: " << object.type << ", of size: " << object.size << std::endl;
        }
    }
    if(shm_->command_actuators.hasChanged()){
      shm_->command_actuators.lock();
      addObject ( tuw::serial::Object ( *shm_->command_actuators, tuw::serial::TYPE_COMMAND_ACTUATORS ) );
      shm_->command_actuators.dataProcessed();
      shm_->command_actuators.unlock();
      if ( print_rx ) std::cout <<  std::setw ( 30 ) << shm_->command_actuators.name() << ": " << *shm_->command_actuators << std::endl;
    }

    if(shm_->command_ackermann.hasChanged()){
      shm_->command_ackermann.lock();
      addObject ( tuw::serial::Object ( *shm_->command_ackermann, tuw::serial::TYPE_COMMAND_ACKERMANN ) );
      shm_->command_ackermann.dataProcessed();
      shm_->command_ackermann.unlock();
      if ( print_rx ) std::cout <<  std::setw ( 30 ) << shm_->command_ackermann.name() << ": " << *shm_->command_ackermann << std::endl;
    }

    shm_->oszi_motor.lock();
    shm_->oszi_motor.resize(6);
    shm_->oszi_motor[0] = shm_->motor_state->pwm.actual;
    shm_->oszi_motor[1] = shm_->motor_state->pwm.target;
    shm_->oszi_motor[2] = shm_->motor_state->rps.actual;
    shm_->oszi_motor[3] = shm_->motor_state->rps.target;
    shm_->oszi_motor[4] = shm_->motor_state->force.actual;
    shm_->oszi_motor[5] = shm_->motor_state->force.target;
    shm_->oszi_motor.itHasChanged();
    shm_->oszi_motor.unlock();

};
