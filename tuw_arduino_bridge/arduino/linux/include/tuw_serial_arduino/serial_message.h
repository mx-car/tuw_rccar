#ifndef TUW_SERIAL_MESSAGE_H
#define TUW_SERIAL_MESSAGE_H

#include <stdint.h>
#include <tuw/arduino/arduino_objects.h>
#include <tuw/serial/serial_base.h>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace tuw {
namespace serial {

struct Parameters {
    std::string port;
    int baudrate;
};

class Time : public tuw::arduino::TTime {
public:
    Time();
    Time ( int32_t sec, int32_t nsec );
    void now();
    std::string toString() const;
    boost::posix_time::ptime toBoost() const;
};

class  Object;
using Objects = std::map<ObjectType,Object>;

class  Object : public TObject {
public:
    Object() : TObject() {};
    Object ( ObjectType type ) : TObject ( type ) {};
    Object ( Object const &o ) : TObject ( o ) {};
    template <class T> Object ( const T& src, ObjectType type ) : TObject ( src, type ) {};

    friend std::ostream &operator << ( std::ostream &os, const Object &o ) {
        os << "[" << o.type <<  ", " << o.size << "]";
        return os;
    };
};


class  Message : public TMessage {
public:
    Message();
    Time &time();
    const Time &time() const;
    friend std::ostream &operator << ( std::ostream &os, const Message &o ) {
        os << "[" << o.size <<  ", " << o.seq << ", " << o.time().toString() << "]";
        return os;
    };
};



class MessageHeader : public TMessageHeader {
public:
    MessageHeader();
    Time &time();
    const Time &time() const;
    friend std::ostream &operator << ( std::ostream &os, const MessageHeader &o ) {
        os << "[" << o.size <<  ", " << o.seq << ", " << o.time().toString() << "]";
        return os;
    };
};

}
}

#endif // TUW_SERIAL_MESSAGE_H
