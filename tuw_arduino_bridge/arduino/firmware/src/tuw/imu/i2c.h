#ifndef _I2C_H_INCLUDED_
#define _I2C_H_INCLUDED_

uint8_t I2C_WriteBytes(uint8_t SlaveAdress, uint8_t* Buffer, uint8_t Register, uint8_t Length);

uint8_t I2C_ReadBytes(uint8_t SlaveAdress, uint8_t* Buffer, uint8_t Register, uint8_t Length);

#endif
