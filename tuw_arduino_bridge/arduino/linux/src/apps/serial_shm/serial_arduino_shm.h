#ifndef TUW_SERIAL_ARDUINO_SHM_H
#define TUW_SERIAL_ARDUINO_SHM_H


#include <tuw_serial_arduino/serial_arduino.h>
#include <tuw_shm/shm_arduino_objects.h>


struct Parameters {
    tuw::serial::Parameters serial;
    tuw::shm::Parameters shm;
    bool print_rx;
};

class SerialArduinoShm : public tuw::serial::SerialArduino {
    bool print_rx;
public:
    SerialArduinoShm ( Parameters &params );
    void callback_serial ( tuw::serial::Message &header,  tuw::serial::Objects & objects ) ;
    std::shared_ptr<tuw::shm::Objects> shm_;
    template <class T> void copy2shm ( const tuw::serial::Object &src, T& des ) {
        des.lock();
        src.get ( *des );
        des.itHasChanged();
        des.unlock();
        if ( print_rx ) std::cout <<  std::setw ( 30 ) << des.name() << ": " << *des << std::endl;
    }
};

#endif //TUW_SERIAL_ARDUINO_SHM_H
