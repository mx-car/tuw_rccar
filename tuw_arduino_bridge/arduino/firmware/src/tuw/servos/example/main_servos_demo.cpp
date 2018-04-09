/*
  sketch Demo
  @author Markus Bader
 */
#include <Arduino.h>
#include <tuw/servos/servos.h>

tuw::Servos *servos;
uint32_t pinServo = 9;

int i = 0;
void setup() {
    init();
    // initialize digital pin 13 as an output.
    Serial.begin(115200);
    Serial.println("servo");
    delay(10);

    servos = &tuw::Servos::getInstance();
    servos->init ( TCC1 );
    servos->setMicroSecondsPerRad ( 955, pinServo );
    servos->enable ( pinServo );
}

// the loop function runs over and over again forever
void loop() {

  if (Serial.available() ) {
    float angle0 = Serial.parseInt();
    servos->setDegrees(angle0, 0);
    Serial.print(servos->getHighTime(0));
    Serial.print(" us, ");
    Serial.print(servos->getRad(0));
    Serial.print(" rad, ");
    Serial.print(servos->getDegrees(0));
    Serial.print(" deg");
    Serial.print(" -- micro seconds per rad ");
    Serial.print(servos->getMicroSecondsPerRad(0));
    Serial.println(" us, ");
  }
  delay(50);
}
