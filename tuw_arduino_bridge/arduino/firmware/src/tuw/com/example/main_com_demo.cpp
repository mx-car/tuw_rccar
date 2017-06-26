/*
 * serial_using_structs
 * demo which sends and receives a struct using the serial interface
 * the struct Pose will send to a PC (python program and returned)
 * @see python program
 * @author Markus Bader
 */

//#include <tuw/serial_msg.h>
#include <Arduino.h>
#include <tuw/com/serial_message.h>
#include <tuw/serial/serial_structs.h>


tuw::arduino::TPose pose;             /// object to send
tuw::arduino::TText text;             /// object to send
tuw::serial::Message msg_tx;   /// object to hande the serial communication
tuw::serial::Message msg_rx;   /// object to hande the serial communication
int loop_count;        /// defines which message should be sent

void setup() {
    init();
    Serial.begin ( 115200 );			/// init serial
    delay ( 1000 );
    pose.x = 140, pose.y = 10., pose.theta = 0.2;
    text.write ( "Hello World!" );
    loop_count = 0;
    msg_rx.try_sync();
}

void loop() {
    delay ( 1000 );
    loop_count++;
    msg_tx.reset();				        /// removes all objects in message
    msg_tx.time().now();				/// update time stamp
    msg_tx.push_object ( tuw::serial::Object( pose, tuw::serial::TYPE_POSE ) );
    msg_tx.push_object ( tuw::serial::Object( text, tuw::serial::TYPE_TEXT ) );
    msg_tx.send();

    if ( msg_rx.receive() ) {      		/// check for messages
        tuw::serial::Object object;
        while ( msg_rx.pop_object ( object ).isValid() ) {
            switch ( object.type ) {
	      case tuw::serial::TYPE_POSE:
		object.get(pose);
                break;
            }
        }
    }

}
