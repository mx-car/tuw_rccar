/**
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @license Simplified BSD License
 */


#ifndef TUW_SERIAL_MESSAGE_H
#define TUW_SERIAL_MESSAGE_H

#include <Arduino.h>
#include <tuw/serial/serial_base.h>

namespace tuw {
namespace serial {

class Time : public tuw::arduino::TTime {
public:
    Time();
    Time ( int32_t sec, int32_t nsec );
    void set ( int32_t sec, int32_t nsec );
    static void setClock ( const tuw::arduino::TTime& now );
    static void setClock ( const tuw::arduino::TTime& now, uint32_t millisecond );
    void now ( uint32_t millisecond );
    void now ();
    static bool isSet();
    static Time offest();
private:
    static Time OFFSET;
    static bool CLOCK_SYNC;
};

class  Message : public TMessage {
public:
    static uint32_t tx_count;  	/// static variale to increase message count
    static uint32_t rx_count;  	/// static variale to increase message count
    static uint16_t sync_count; 	/// number of sync attempts
    Message() : TMessage() {};
    Time &time();
    const Time &time() const;
    int send();
    int receive();
    void try_sync();
private:
    void writeByte( uint8_t bt );
    int8_t readByte( uint8_t* bt );
    uint8_t serialbuffer[MAX_BUFFER_SIZE];
    uint16_t serialbuffer_used;
};



class  Object : public TObject {
public:
    Object() : TObject() {};
    Object ( ObjectType type ) : TObject ( type ) {};
    Object ( Object const &o ) : TObject ( o ) {};
    template <class T> Object ( const T& src, ObjectType type ) : TObject ( src, type ) {};
};




};
};
#endif //TUW_SERIAL_MESSAGE_H

