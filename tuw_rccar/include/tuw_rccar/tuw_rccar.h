#ifndef __TUW_RCCAR_H
#define __TUW_RCCAR_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <tuw_geometry/tuw_geometry.h>
#include <tuw_rccar/RCCarConfig.h>
#include "tuw_rccar/serial_arduino.h"

namespace tuw {
/**
 * Robot class
 */
class RCCar {
public:
    RCCar(const std::string &ns);        // Constructor
    void init();                         // initialization

    void callback_serial ( tuw::serial::Message &header,  tuw::serial::Objects & objects );

protected:

    tuw::serial::SerialArduino serial_arduino;

    tuw_rccar::RCCarConfig config_;

    double twist_velocity;
    double twist_steering_angle;


};

}

#endif // __TUW_RCCAR_H


