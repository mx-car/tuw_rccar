/*
  sketch Demo
  @author Markus Bader
 */
#include <Arduino.h>
#include <tuw/bldc/bldc.h>

uint32_t pinServo = 9;

int i = 0;
void setup() {
    init();
    // initialize digital pin 13 as an output.
    Serial.begin(115200);
    SerialUSB.begin(115200);
    Serial.println("bldc");
    delay(10);
    
    tuw::BLDC::getInstance().init(A0,A1,A2,2,4,8,3,5,7);
    tuw::BLDC::getInstance().set_rps(50);
    
}
// the loop function runs over and over again forever
void loop() {
  char msg[0xFF];
  delay(50);
  tuw::BLDC &bldc = tuw::BLDC::getInstance();
  if (Serial.available() >= 2) {
    if (Serial.read() == '#') {
      int command = Serial.read(); // Commands
      float v = Serial.parseFloat();
      switch (command) {
	case 't':
	    bldc.set_rps(v);
	    sprintf ( msg, "$t %s%i.%04d", PRINT_FLOAT4 (v));
	  break;
	case 'p':
            bldc.pid().Kp() = v;
            sprintf ( msg, "$p %s%i.%04d", PRINT_FLOAT4 (v));
	  break;
	case 'i':
            bldc.pid().Ki() = v;
            sprintf ( msg, "$i %s%i.%04d", PRINT_FLOAT4 (v));
	  break;
	case 'd':
            bldc.pid().Kd() = v;
            sprintf ( msg, "$d %s%i.%04d", PRINT_FLOAT4 (v));
	  break;
	default:
	    sprintf ( msg, "$v: %s%i.%04d", PRINT_FLOAT4 (v) );
      }
      Serial.println(msg); 
      //bldc.set_offset(Serial.parseInt());
    }
  }
  int n = bldc.debug_msg(msg);
  for(int i = 0; i < n;i++){
    Serial.write(msg[i]); 
  }
  //if(i % 100 == 0) SerialUSB.println(i); 
  i++;
}
