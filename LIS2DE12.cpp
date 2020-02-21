/*
	Created by: Benjamin Schulz
	Created on: 2/18/2020
	Purpose: Allow I2C communication with STI Tri-axial accelerometer LIS2DE12.
*/

#include "LIS2DE12.h"

#define LIS2DE12_ADDRESS            0x19 // address on I2C bus

#define LIS2DE12_WHO_AM_I           0x0f 
#define LIS2DE12_CTRL_REG1          0x20
#define LIS2DE12_STATUS_REG         0x27
#define LIS2DE12_CTRL_REG4          0x23 //LSM9DS1_CTRL_REG6_XL
#define LIS2DE12_CTRL_REG5          0x24 //LSM9DS1_CTRL_REG8
#define LIS2DE12_OUT_X              0x29 //LSM9DS1_OUT_X_XL
#define LIS2DE12_OUT_Y				0x2B
#define LIS2DE12_OUT_Z				0x2D

LIS2DE12Class::LIS2DE12Class(TwoWire& wire) :
  continuousMode(false), _wire(&wire)
{

}

LIS2DE12Class::~LIS2DE12Class()
{

}

int LIS2DE12Class::begin()
{
	_wire->begin();

	delay(10);

	if (readRegister(LIS2DE12_ADDRESS, LIS2DE12_WHO_AM_I) != 0x33) {
		end();

		return 0;
	}


	writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x5F);		// write ODR 100 Hz by default
	writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG4, 0x10);		// write 4g acceleration by default
	sensitivityFactor = 0.0312f;


	delay(10);

	return 1;
}
int LIS2DE12Class::begin(int aScale, int frequency)
{
	_wire->begin();

	sampleRate = frequency;
	accelerationScale = aScale;

	delay(10);

	if (readRegister(LIS2DE12_ADDRESS, LIS2DE12_WHO_AM_I) != 0x33) {
		end();

		return 0;
	}

	switch (frequency) {
		case 0:	// power down
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x0F);		// write ODR POWER DOWN MODE
			break;
		case 1:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x1F);		// write ODR 1 Hz
			break;
		case 10:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x2F);		// write ODR 10 Hz
			break;
		case 25:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x3F);		// write ODR 25 Hz
			break;
		case 50:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x4F);		// write ODR 50 Hz
			break;
		case 100:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x5F);		// write ODR 100 Hz
			break;
		case 200:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x6F);		// write ODR 200 Hz
			break;
		case 400:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x7F);		// write ODR 400 Hz
			break;
		case 1620:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x8F);		// write ODR 1620 Hz
			break;
		case 5376:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x9F);		// write ODR 5376 Hz
			break;
		default:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x5F);		// write ODR 100 Hz by default
			break;
	}

	switch (aScale) {
		case 2:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG4, 0x00);		// write 2g acceleration
			sensitivityFactor = 0.0156f;
			break;
		case 4:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG4, 0x10);		// write 4g acceleration
			sensitivityFactor = 0.0312f;
			break;
		case 8:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG4, 0x20);		// write 8g acceleration
			sensitivityFactor = 0.0625f;
			break;
		case 16:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG4, 0x30);		// write 16g acceleration
			sensitivityFactor = 0.1875f;
			break;
		default:
			writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG4, 0x10);		// write 4g acceleration by default
			sensitivityFactor = 0.0312f;
			break;
	}

	delay(10);

	return 1;
}

void LIS2DE12Class::setContinuousMode() {
	writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG5, 0x40);			// Enable FIFO
	writeRegister(LIS2DE12_ADDRESS, 0x2E, 0x80);							// Set streaming mode (FIFO Control reg  = 0x2E)

	continuousMode = true;
}

void LIS2DE12Class::setOneShotMode() {
	writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG5, 0x00);			// Disable FIFO
	writeRegister(LIS2DE12_ADDRESS, 0x2E, 0x00);						// Disable streaming mode (FIFO Control reg = 0x2E)

	continuousMode = false;
}

void LIS2DE12Class::end()
{
	writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG1, 0x0F);
	writeRegister(LIS2DE12_ADDRESS, LIS2DE12_CTRL_REG4, 0x00);

	_wire->end();
}

int LIS2DE12Class::readAcceleration(float& x, float& y, float& z)
{
	int8_t data[3];

	data[0] = readRegister(LIS2DE12_ADDRESS, LIS2DE12_OUT_X);
	data[1] = readRegister(LIS2DE12_ADDRESS, LIS2DE12_OUT_Y);
	data[2] = readRegister(LIS2DE12_ADDRESS, LIS2DE12_OUT_Z);

	x = data[0] * sensitivityFactor;
	y = data[1] * sensitivityFactor;
	z = data[2] * sensitivityFactor;

	return 1;
}

int LIS2DE12Class::accelerationAvailable()
{
	if (continuousMode) {
    // Read FIFO_SRC. If any of the rightmost 8 bits have a value, there is data.
		if (readRegister(LIS2DE12_ADDRESS, 0x2F) & 63) {
			return 1;
		}
	} 
	else {
		if (readRegister(LIS2DE12_ADDRESS, LIS2DE12_STATUS_REG) & 0x01) {
			return 1;
		}
	}
	return 0;
}

int LIS2DE12Class::accelerationSampleRate()
{
	return sampleRate;
}

int LIS2DE12Class::readRegister(uint8_t slaveAddress, uint8_t address)
{
	_wire->beginTransmission(slaveAddress);
	_wire->write(address);
	if (_wire->endTransmission() != 0) {
		return -1;
	}

	if (_wire->requestFrom(slaveAddress, 1) != 1) {
		return -1;
	}

	return _wire->read();
}

int LIS2DE12Class::writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value)
{
	_wire->beginTransmission(slaveAddress);
	_wire->write(address);
	_wire->write(value);
	if (_wire->endTransmission() != 0) {
		return 0;
	}

	return 1;
}

#ifdef ARDUINO_ARDUINO_GENUINO_UNO
LIS2DE12Class ACCELEROMETER(Wire1);
#else
LIS2DE12Class ACCELEROMETER(Wire);
#endif