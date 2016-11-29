#ifndef __SERIAL_ARDUINO_H
#define __SERIAL_ARDUINO_H

#include <stdint.h>
#include <iostream>
#include <memory>
#include <vector>
#include <functional>
#include <thread>
#include <mutex>
#include "tuw_rccar/serial_message.h"

class TimeoutSerial;

namespace tuw {
namespace serial {

class SerialArduino {
    Message msg_tx_;
    Objects obj_tx_;
    Message msg_rx_;
    Objects obj_rx_;
    std::mutex mutex_obj_tx_;

    std::function<void (Message &,  Objects &)> callback;
    void serial_monitor ( const std::string& devname, unsigned int baud_rate );
    std::thread serial_monitor_thread_;
    TimeoutSerial *serial_timeout_;
public:
    SerialArduino();
    ~SerialArduino();
    void init ( const Parameters &param, std::function<void (Message &,  Objects &)> callback_fnc );
    void close();
    bool loop;
    void addObject(const Object &object);

};


}
}

#endif // __SERIAL_ARDUINO_H
