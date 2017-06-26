#include <Arduino.h>
#include <tuw/imu/imu.h>

tuw::IMU imu;

void setup() {
	init();
	Serial.begin(115200);
	SerialUSB.begin(115200);
	Serial.println("IMU Demo Program");
	delay(10);

	imu.init();
}

// the loop function runs over and over again forever
void loop() {
	char msg[0xFF];
	delay(500);
	imu.update();

	Serial.print("Temperature: ");
	Serial.print(imu.get_temperature());
	Serial.println(" dC");


	Serial.print("Pressure: ");
	Serial.print(imu.get_pressure());
	Serial.println(" /100 mbar");

	Serial.print("Gyro-X: ");
	Serial.print(imu.get_gyro(AXIS_X));
	Serial.println(" dps");

	Serial.print("Gyro-Y: ");
	Serial.print(imu.get_gyro(AXIS_Y));
	Serial.println(" dps");

	Serial.print("Gyro-Z: ");
	Serial.print(imu.get_gyro(AXIS_Z));
	Serial.println(" dps");

	Serial.print("Accel-X: ");
	Serial.print(imu.get_accel(AXIS_X));
	Serial.println(" cm/s2");

	Serial.print("Accel-Y: ");
	Serial.print(imu.get_accel(AXIS_Y));
	Serial.println(" cm/s2");

	Serial.print("Accel-Z: ");
	Serial.print(imu.get_accel(AXIS_Z));
	Serial.println(" cm/s2");

	Serial.print("Magnet-X: ");
	Serial.print(imu.get_magnet(AXIS_X));
	Serial.println(" mGs");

	Serial.print("Magnet-Y: ");
	Serial.print(imu.get_magnet(AXIS_Y));
	Serial.println(" mGs");

	Serial.print("Magnet-Z: ");
	Serial.print(imu.get_magnet(AXIS_Z));
	Serial.println(" mGs");

	Serial.print("Compass: ");
	Serial.print(imu.get_compass());
	Serial.println(" deg");

	Serial.println("");
	Serial.println("");
}
