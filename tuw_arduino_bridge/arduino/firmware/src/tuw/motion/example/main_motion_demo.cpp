/*
 * serial_using_structs
 * demo which sends and receives a struct using the serial interface
 * the struct Pose will send to a PC (python program and returned)
 * @see pythpon program
 * @author Markus Bader
 */

#include <tuw/com/serial_message.h>
#include <tuw/serial/serial_structs.h>
#include <tuw/motion/motion.h>

#define PRINT_FLOAT4(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*10000.)))
#define PRINT_FLOAT2(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*100.)))

tuw::serial::Message msg_tx;   				/// object to hande the serial communication
tuw::serial::Message msg_rx;   				/// object to hande the serial communication
tuw::arduino::TText text;             			/// object to send
unsigned int loop_count;
tuw::Motion  motion;
bool command_ackermann_config = 0;

void setup() {
    init();
    Serial.begin ( 115200 );			/// init serial
    msg_tx.try_sync();   			        /// blocks until a sync message arrives
    delay ( 1000 );
    motion.init ( A0,A1,A2,2,4,8,3,5,7,9, 0 );
    text.write ( "Motion ready!" );
    loop_count = 0;
}

void loop() {
    delay ( 10 );
    motion.filter_servos();
    msg_tx.reset();				/// removes all objects in message
    msg_tx.time().now(); 			/// update time stamp
    motion.update_objects();
//    msg_tx.push_object ( tuw::serial::Object ( text, tuw::serial::TYPE_TEXT ) );
    if (command_ackermann_config) {
        command_ackermann_config = 0;
        msg_tx.push_object ( tuw::serial::Object ( motion.ackermann_config_, tuw::serial::TYPE_ACKERMANN_CONFIG ) );
    }
//    msg_tx.push_object ( tuw::serial::Object ( motion.motor_state_, tuw::serial::TYPE_MOTOR_STATE ) );
//    msg_tx.push_object ( tuw::serial::Object ( motion.motor_pid_, tuw::serial::TYPE_MOTOR_PID ) );
//    msg_tx.push_object ( tuw::serial::Object ( motion.servo_state_, tuw::serial::TYPE_SERVO_STATE ) );
//    msg_tx.push_object ( tuw::serial::Object ( motion.servo_pid_, tuw::serial::TYPE_SERVO_PID ) );
//    msg_tx.push_object ( tuw::serial::Object ( motion.pose_, tuw::serial::TYPE_MOTION_POSE_ESTIMATED ) );
//    msg_tx.push_object ( tuw::serial::Object ( motion.pose_cov_, tuw::serial::TYPE_MOTION_POSE_COVARIANCE_ESTIMATED ) );
    msg_tx.push_object ( tuw::serial::Object ( motion.imu_accelerometer_, tuw::serial::TYPE_IMU_ACCELEROMETER ) );
    msg_tx.push_object ( tuw::serial::Object ( motion.imu_gyroscope_, tuw::serial::TYPE_IMU_GYROSCOPE ) );
//    msg_tx.push_object ( tuw::serial::Object ( motion.imu_magnetometer_, tuw::serial::TYPE_IMU_MAGNETOMETER ) );
    msg_tx.push_object ( tuw::serial::Object ( motion.imu_environment_, tuw::serial::TYPE_IMU_ENVIRONMENT ) );
    msg_tx.send();				        /// sends the message

    if ( msg_rx.receive() ) {			/// check for messages
        static tuw::serial::Object object;
        while ( msg_rx.pop_object ( object ).isValid() ) {
            switch ( object.type ) {
            case tuw::serial::TYPE_SYNC: 	/// case sync object
                tuw::serial::Time::setClock ( msg_rx.stamp ); /// set clock
                break;
            case tuw::serial::TYPE_COMMAND_ACTUATORS: {
                static tuw::arduino::TActuators o;
                object.get ( o );    	/// update pose
                motion.setActuators ( o.rps, o.rad );
                sprintf ( text.txt, "new ACTUATORS: %s%i.%02d, %s%i.%02d", PRINT_FLOAT2 ( o.rps ), PRINT_FLOAT2 ( o.rad ) );
                }
                break;
            case tuw::serial::TYPE_COMMAND_MOTOR_PID: {
                static tuw::arduino::TPID o;
                object.get ( o );    	/// update pose
                motion.setMotorPID ( o.kp, o.ki, o.kd );
                sprintf ( text.txt, "new MOTOR_PID: %s%i.%02d, %s%i.%02d", PRINT_FLOAT2 ( o.kp ), PRINT_FLOAT2 ( o.ki ), PRINT_FLOAT2 ( o.kd ) );
                }
                break;
            case tuw::serial::TYPE_COMMAND_ACKERMANN_CONFIG:
                command_ackermann_config = 1;
                break;
            default:/// case unkown type
                text.write ( "Unknown type received" );
                continue;
            }
        }
    }
    loop_count++;
}
