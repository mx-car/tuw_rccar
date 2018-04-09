#include "Wire.h"

uint8_t I2C_WriteBytes(uint8_t SlaveAdress, uint8_t* Buffer, uint8_t Register, uint8_t Length) {
	Wire.beginTransmission(SlaveAdress >> 1);
	Wire.write(Register);
	uint8_t i;
	for (i = 0; i < Length; i++) {
		Wire.write(*(Buffer+i));
	}
	Wire.endTransmission();
	return 1;
}

uint8_t I2C_ReadBytes(uint8_t SlaveAdress, uint8_t* Buffer, uint8_t Register, uint8_t Length) {
	Wire.beginTransmission(SlaveAdress >> 1);
	Wire.write(Register);
	Wire.endTransmission();

	Wire.requestFrom(SlaveAdress >> 1, Length);

	if (Length <= Wire.available()) {
		uint8_t i;
		for (i = 0; i < Length; i++) {
			*(Buffer+i) = Wire.read();
		}
	}
	return 1;
}
