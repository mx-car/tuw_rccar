#include "tuw_rccar/serial_message.h"

using namespace tuw::serial;



Time::Time() : TTime() {};
Time::Time ( int32_t sec, int32_t nsec ) :TTime ( sec,nsec ) {};


void Time::now() {
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - boost::posix_time::from_time_t ( 0 );
    sec = diff.total_seconds();
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
    nsec = diff.fractional_seconds();
#else
    nsec = diff.fractional_seconds() *1000;
#endif
}


boost::posix_time::ptime Time::toBoost() const {
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
    return boost::posix_time::from_time_t ( sec ) + boost::posix_time::nanoseconds ( nsec );
#else
    return boost::posix_time::from_time_t ( sec ) + boost::posix_time::microseconds ( nsec/1000.0 );
#endif
}

std::string Time::toString() const {
    return boost::posix_time::to_simple_string ( toBoost() );
}

MessageHeader::MessageHeader()
    : TMessageHeader() {};


Message::Message()
    : TMessage() {};

Time &Message::time(){
  return ( Time & ) this->stamp;
}

const Time &Message::time() const{
  return ( Time & ) this->stamp;
}

Time &MessageHeader::time(){
  return ( Time & ) this->stamp;
}

const Time &MessageHeader::time() const{
  return ( Time & ) this->stamp;
}
