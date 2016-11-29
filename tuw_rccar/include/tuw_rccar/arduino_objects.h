#ifndef __ARDUINO_OBJECTS_H
#define __ARDUINO_OBJECTS_H

#include "tuw_rccar/arduino_structs.h"

namespace tuw {
namespace arduino {

class  Text : public TText {
public:
    Text(): TText() {}
    Text ( const char *text ): TText() {
      this->write(text);
    }
};

class  Pose : public TPose {
public:
    Pose() {}
    Pose ( float x, float y , float theta ) {
        this->x=x, this->y = y, this->theta = theta;
    }
};

class  Mat3x3 : public TMat3x3 {
public:
    Mat3x3() {}
};

class  Actuators : public TActuators {
public:
    Actuators() {}
    Actuators ( float rps, float rad ) {
        this->rps=rps, this->rad = rad;
    }
    void zero() {
      this->rps = 0, this->rad = 0;
    }
};

class  Point : public TPoint {
public:
    Point() {}
    Point ( float x, float y ) {
        this->x=x, this->y = y;
    }
};

class  Vector2 : public TVector2 {
public:
    Vector2() {}
    Vector2 ( float x, float y ) {
        this->x=x, this->y = y;
    }
};

class  Vector3 : public TVector3 {
public:
    Vector3() {}
    Vector3 ( float x, float y, float z ) {
        this->x=x, this->y = y, this->z = z;
    }
};

class  AckermannConfig : public TAckermannConfig {
public:
    AckermannConfig() : TAckermannConfig() {}
};

class  PID : public TPID {
public:
    PID() : TPID() {}
};

class  Motor : public TMotor {
public:
    Motor() : TMotor() {}
};


class  Servo : public TServo {
public:
    Servo() : TServo() {}
};

class  IMU_Accelerometer : public TIMU_Accelerometer {
public:
    IMU_Accelerometer() : TIMU_Accelerometer() {}
};

class  IMU_Gyroscope : public TIMU_Gyroscope {
public:
    IMU_Gyroscope() : TIMU_Gyroscope() {}
};

class  IMU_Magnetometer : public TIMU_Magnetometer {
public:
    IMU_Magnetometer() : TIMU_Magnetometer() {}
};

class  IMU_Environment : public TIMU_Environment {
public:
    IMU_Environment() : TIMU_Environment() {}
};

}
}

#endif // __ARDUINO_OBJECTS_H
