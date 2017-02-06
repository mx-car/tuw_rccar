#ifndef __TUW_RCCAR_H
#define __TUW_RCCAR_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <tuw_geometry/tuw_geometry.h>
#include <sensor_msgs/Imu.h>
#include <tuw_rccar/RCCarConfig.h>
#include "tuw_rccar/serial_arduino.h"

namespace tuw {
/**
 * Robot class
 */
class RCCar {
public:
    RCCar(const std::string &ns);        // Constructor
    ~RCCar(void);
    void init();                         // initialization

    void callback_serial ( tuw::serial::Message &header,  tuw::serial::Objects & objects );

protected:

    tuw::serial::SerialArduino serial_arduino;

    tuw_rccar::RCCarConfig config_;

    tuw::arduino::Actuators actuators_;
    tuw::arduino::Actuators actuators_last_;
    ros::Time actuators_last_received;
    ros::Time actuators_last_sent;
    bool service_ackermann_config;
    bool service_pid_controller;

    sensor_msgs::Imu imu_;
    ros::Time imu_last_received;
    ros::Time imu_last_sent;

    tuw::arduino::AckermannConfig amcfg_;

};

}

#endif // __TUW_RCCAR_H


