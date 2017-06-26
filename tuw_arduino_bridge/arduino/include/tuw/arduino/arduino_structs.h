#ifndef TUW_ARDUINO_STRUCTS_H
#define TUW_ARDUINO_STRUCTS_H

#include <stdint.h>
#include <stdlib.h>
#include <cstring>
#ifndef USB_PRODUCT
#include <iostream>
#include <boost/algorithm/string.hpp>
#endif

namespace tuw {
namespace arduino {

/**
 * Time for serial communication
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 **/
struct TTime {
    TTime() : sec ( 0 ) , nsec ( 0 ) {};
    TTime ( int32_t sec, int32_t nsec )  :sec ( sec ) , nsec ( nsec ) {};
    int32_t sec;      /// seconds (stamp_secs) since epoch
    int32_t nsec;     /// nanoseconds = 0.000000001 sec since stamp_secs
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TTime &o ) {
        os << "[" << o.sec << ", " << o.nsec <<  "]";
        return os;
    };

#endif
};

struct TPose {
    TPose() : x ( 0 ), y ( 0 ), theta ( 0 ) {};
    TPose ( float x, float y, float theta ) : x ( x ), y ( y ), theta ( theta ) {};
    float x;
    float y;
    float theta;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TPose &o ) {
        os << "[" << o.x << ", " << o.y << ", " << o.theta <<  "]";
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TPose &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %6.4f, %6.4f, %6.4f]", x, y, theta );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f,%f", &x, &y, &theta ) == EOF ) return false;
        return true;
    }
#endif
};
struct TMat3x3 {
    float m[9];
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TMat3x3 &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TMat3x3 &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[%4.3f, %4.3f, %4.3f; %4.3f, %4.3f, %4.3f; %4.3f, %4.3f, %4.3f]"
                  ,    m[0],  m[1],  m[2],  m[3],  m[4],  m[5],  m[6],  m[7],  m[8] );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f, %f, %f; %f, %f, %f; %f, %f, %f", &m[0],  &m[1],  &m[2],  &m[3],  &m[4],  &m[5],  &m[6],  &m[7],  &m[8] ) == EOF ) return false;
        return true;
    }
#endif
};

struct  TText {
    TText() {
        clear();
    }
    static const int MAX_BUFFER_SIZE = 32;
    char txt[MAX_BUFFER_SIZE+1];
    void clear() {
        memset ( txt, '\0', MAX_BUFFER_SIZE+1 );
    }
    int write ( const char *msg ) {
        int bytes_remaining = MAX_BUFFER_SIZE - strlen ( msg );
        if ( bytes_remaining > 0 ) {
            strcpy ( txt, msg );
        }
        return bytes_remaining;
    }

    int size() {
        return strlen ( txt );
    }
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TText &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TText &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        return std::string ( txt );
    }
    bool setFromString ( const std::string &str ) {
        /// ToDo check for error
        write ( str.c_str() );
        return true;
    }
#endif
};
struct  TActuators {
    TActuators() : rps ( 0 ), rad ( 0 ) {};
    TActuators ( float rps, float rad ) : rps ( rps ), rad ( rad ) {};
    float rps;
    float rad;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TActuators &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TActuators &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f]", rps, rad );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f", &rps, &rad ) == EOF ) return false;
        return true;
    }
#endif
};

struct  TAckermann {
    TAckermann() : v ( 0 ), alpha ( 0 ) {};
    TAckermann ( float v, float alpha ) : v ( v ), alpha ( alpha ) {};
    float v;
    float alpha;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TAckermann &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TAckermann &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f]", v, alpha );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f", &v, &alpha ) == EOF ) return false;
        return true;
    }
#endif
};
struct  TAckermannConfig {
    TAckermannConfig() : d ( 0 ), l ( 0 ), gear_ratio ( 0 ), wheel_radius ( 0 ), max_steer_angle ( 0 ), weight ( 0 ) {};
    TAckermannConfig ( float d, float l, float gear_ratio, float wheel_radius, float max_steer_angle, float weight ) : d ( d ), l ( l ), gear_ratio ( gear_ratio ), wheel_radius ( wheel_radius ), max_steer_angle ( max_steer_angle ), weight ( weight ) {};
    float d; /// distance front <-> back wheels [m]
    float l; /// distance between wheels [m]
    float gear_ratio; /// rps [Motor] -> rps [wheels]
    float wheel_radius; /// wheel radius [m]
    float max_steer_angle; /// max steering angle [rad]
    float weight; /// total weight of car [kg]
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TAckermannConfig &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TAckermannConfig &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", d, l, gear_ratio, wheel_radius, max_steer_angle, weight );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f,%f,%f,%f,%f", &d, &l, &gear_ratio, &wheel_radius, &max_steer_angle, &weight ) == EOF ) return false;
        return true;
    }
#endif
};

struct TPoint {
    TPoint() : x ( 0 ),y ( 0 ) {};
    TPoint ( float x, float y ) : x ( x ),y ( y ) {};
    float x;
    float y;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TPoint &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TPoint &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f]", x, y );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f", &x, &y ) == EOF ) return false;
        return true;
    }
#endif
};

struct TVector2 {
    TVector2() : x ( 0 ),y ( 0 ) {};
    TVector2 ( float x, float y ) : x ( x ),y ( y ) {};
    float x;
    float y;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TVector2 &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TVector2 &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f]", x, y );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f", &x, &y ) == EOF ) return false;
        return true;
    }
#endif
};

struct TVector3 {
    TVector3() : x ( 0 ), y ( 0 ), z( 0 ) {};
    TVector3 ( float x, float y, float z) : x ( x ), y ( y ), z ( z ) {};
    float x;
    float y;
    float z;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TVector3 &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TVector3 &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[255];
        sprintf ( buf, "[ %4.2f, %4.2f, %4.2f]", x, y, z );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f,%f", &x, &y, &z ) == EOF ) return false;
        return true;
    }
#endif
};

struct TActualTarget {
    TActualTarget() : actual ( 0 ), target ( 0 ) {};
    TActualTarget ( float actual, float target ) : actual ( actual ),target ( target ) {};
    float actual;
    float target;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TActualTarget &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TActualTarget &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f]", actual, target );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f", &actual, &target ) == EOF ) return false;
        return true;
    }
#endif
};

struct TPID {
    TPID() : kp ( 0 ), ki ( 0 ), kd ( 0 ) {};
    TPID ( float kp, float ki, float kd ) : kp ( kp ), ki ( ki ), kd ( kd ) {};
    float kp;
    float ki;
    float kd;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TPID &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TPID &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %6.4f, %6.4f, %6.4f]", kp, ki, kp );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f,%f", &kp, &ki, &kp ) == EOF ) return false;
        return true;
    }
#endif
};

struct TMotor {
    TMotor() : rps (), pwm (), force () {};
    TMotor ( const TActualTarget &rps, const TActualTarget &pwm, const TActualTarget &force ) : rps ( rps ),pwm ( pwm ),force ( force ) {};
    TActualTarget rps;
    TActualTarget pwm;
    TActualTarget force;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TMotor &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TMotor &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[[ %6.4f, %6.4f], [ %6.4f, %6.4f], [ %6.4f, %6.4f]]", rps.actual, rps.target, pwm.actual, pwm.target, force.actual, force.target );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "[%f,%f],[%f,%f],[%f,%f]", &rps.actual, &rps.target, &pwm.actual, &pwm.target, &force.actual, &force.target ) == EOF ) return false;
        return true;
    }
#endif
};


struct TServo {
    TServo() : angle (), high_time () {};
    TServo ( const TActualTarget &angle, const TActualTarget &pwm ) : angle ( angle ), high_time ( high_time ) {};
    TActualTarget angle;
    TActualTarget high_time;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TServo &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TServo &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[[ %6.4f, %6.4f], [ %6.4f, %6.4f]]", angle.actual, angle.target, high_time.actual, high_time.target );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "[%f,%f],[%f,%f]", &angle.actual, &angle.target, &high_time.actual, &high_time.target ) == EOF ) return false;
        return true;
    }
#endif
};

struct TIMU_Accelerometer {
    TIMU_Accelerometer() : state () {};
    TIMU_Accelerometer ( const TVector3 &state ) : state ( state ) {};
    TVector3 state;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TIMU_Accelerometer &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TIMU_Accelerometer &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f, %4.2f ]", state.x, state.y, state.z );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f,%f", &state.x, &state.y, &state.z ) == EOF ) return false;
        return true;
    }
#endif
};

struct TIMU_Gyroscope {
    TIMU_Gyroscope() : state () {};
    TIMU_Gyroscope ( const TVector3 &state ) : state ( state ) {};
    TVector3 state;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TIMU_Gyroscope &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TIMU_Gyroscope &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f, %4.2f ]", state.x, state.y, state.z );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f,%f", &state.x, &state.y, &state.z ) == EOF ) return false;
        return true;
    }
#endif
};

struct TIMU_Magnetometer {
    TIMU_Magnetometer() : state () {};
    TIMU_Magnetometer ( const TVector3 &state ) : state ( state ) {};
    TVector3 state;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TIMU_Magnetometer &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TIMU_Magnetometer &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f, %4.2f ]", state.x, state.y, state.z );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f,%f", &state.x, &state.y, &state.z ) == EOF ) return false;
        return true;
    }
#endif
};

struct TIMU_Environment {
    TIMU_Environment() : state () {};
    TIMU_Environment ( const TVector3 &state ) : state ( state ) {};
    TVector3 state;
#ifndef USB_PRODUCT
    friend std::ostream &operator << ( std::ostream &os, const TIMU_Environment &o ) {
        os << o.getToString();
        return os;
    };
    friend std::istream& operator>>(std::istream &input, TIMU_Environment &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    std::string getToString() const {
        char buf[0xFF];
        sprintf ( buf, "[ %4.2f, %4.2f, %4.2f ]", state.x, state.y, state.z );
        return std::string ( buf );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "%f,%f,%f", &state.x, &state.y, &state.z ) == EOF ) return false;
        return true;
    }
#endif
};

};
};

#endif // TUW_ARDUINO_STRUCTS_H

