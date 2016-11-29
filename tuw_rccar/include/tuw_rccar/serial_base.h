#ifndef __SERIAL_BASE_H
#define __SERIAL_BASE_H

#include <stdint.h>
#include <stdlib.h>
#include <cstring>
#include "tuw_rccar/arduino_objects.h"

namespace tuw {
namespace serial {

static const uint16_t TYPE_ERROR = 0;    /// error on decodeding
static const uint16_t TYPE_EMPTY = 1;    /// no type encoded
static const uint16_t TYPE_SYNC_REQUEST = 10; /// sync request type
static const uint16_t TYPE_SYNC = 11;    /// sync type

static const uint16_t TYPE_TEXT = 100;
static const uint16_t TYPE_POSE = 200;
static const uint16_t TYPE_MOTION_POSE_ESTIMATED = 201;
static const uint16_t TYPE_MOTION_POSE_COVARIANCE_ESTIMATED = 251;
static const uint16_t TYPE_ACKERMANN_CONFIG = 400;
static const uint16_t TYPE_MOTOR = 500;
static const uint16_t TYPE_MOTOR_STATE = 501;
static const uint16_t TYPE_MOTOR_PID = 502;
static const uint16_t TYPE_SERVO = 600;
static const uint16_t TYPE_SERVO_STATE = 601;
static const uint16_t TYPE_SERVO_PID = 602;
static const uint16_t TYPE_IMU_ACCELEROMETER = 700;
static const uint16_t TYPE_IMU_GYROSCOPE = 701;
static const uint16_t TYPE_IMU_MAGNETOMETER = 702;
static const uint16_t TYPE_IMU_ENVIRONMENT = 703;
static const uint16_t TYPE_COMMAND_ACTUATORS = 5010;
static const uint16_t TYPE_COMMAND_ACKERMANN = 5020;
static const uint16_t TYPE_COMMAND_MOTOR_PID = 5030;
static const uint16_t TYPE_COMMAND_SERVO_PID = 5040;

typedef uint16_t ObjectType;

struct TObjectHeader {
    TObjectHeader();
    TObjectHeader ( const TObjectHeader &o );
    TObjectHeader ( ObjectType type, int16_t size );
    ObjectType type;
    int16_t size;
};

class TObject : public TObjectHeader {
public:
    char* buffer;
    TObject();
    TObject ( ObjectType type );
    TObject ( const TObject &o );
    template <class T> TObject ( const T& src, ObjectType type ) : TObjectHeader ( type, sizeof ( T ) ), buffer ( NULL ) {
        set ( src, type );
    }
    ~TObject ();
    TObject& empty() ;
    TObject& error() ;
    bool isValid() ;
    /**
     * @param data array with the serialized object
     * @param len array length
     * @return number of consumed bytes or the length argument
     */
    unsigned int deserialize ( const char *data, unsigned int len );
    TObject& operator= ( const TObject &o );
    template <class T>const T& set ( const T& src, uint16_t type ) {
        this->type = type;
        this->size = sizeof ( T );
        alloc ( this->size );
        memcpy ( buffer, &src, this->size );
        return src;
    }
    template <class T> T& get ( T& des ) const {
        if ( ( sizeof ( T ) == size ) && ( size > 0 ) ) {
            memcpy ( &des, buffer, this->size );
        }
        return des;
    }
protected:
    void alloc ( size_t size );
    void dealloc();
    void copy_buffer_from ( const char *src );
};


/**
 * Header for serial message
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 **/
struct TMessageHeader {
    TMessageHeader();
    TMessageHeader ( uint16_t size, uint32_t seq, const tuw::arduino::TTime &stamp );
    uint16_t size;                       /// total message size if size
    uint32_t seq;                        /// consecutively increasing ID
    tuw::arduino::TTime stamp;                          /// time since epoch
    void zeros();
};

class TMessage : public TMessageHeader {
protected:
public:
    TMessage();
    static const int MAX_BUFFER_SIZE = 0x1FF;
    char buffer[MAX_BUFFER_SIZE];
    int stack_pointer;
    bool isValid();
    void reset();
    unsigned int deserialize ( const char *data, unsigned int len );
    bool push_sync () ;
    TObject &pop_object ( TObject &object ) ;
    TMessage &push_object ( const TObject &object );
};

}
}

#endif // __SERIAL_BASE_H


