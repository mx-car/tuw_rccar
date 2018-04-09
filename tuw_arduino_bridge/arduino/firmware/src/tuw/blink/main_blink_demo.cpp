/**
 * Blink demo
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 */

#include <Arduino.h>

int led = 13;

void setup() {
  // initialize digital pin 13 as an output.
  pinMode(led, OUTPUT);
  char *p = (char*) malloc(sizeof(char)*4);
  p[1] = 's';
  //free(p);
}

void loop() {
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);
}
