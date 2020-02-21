/*
	Created by: Benjamin Schulz
	Created on: 2/18/2020
	Purpose: Allow I2C communication with STI Tri-axial accelerometer LIS2DE12.
*/

#include <Arduino.h>
#include <Wire.h>

class LIS2DE12Class{

  public:
    LIS2DE12Class(TwoWire& wire);
    virtual ~LIS2DE12Class();

	int begin(int aScale, int frequency);
    int begin();
    void end();

    void setContinuousMode();
    void setOneShotMode();

    virtual int readAcceleration(float& x, float& y, float& z);
    virtual int accelerationAvailable();
    virtual int accelerationSampleRate();

  private:
    bool continuousMode;

	int accelerationScale;
	float sensitivityFactor;
	int sampleRate;

    int readRegister(uint8_t slaveAddress, uint8_t address);
    int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);

    TwoWire* _wire;
};

extern LIS2DE12Class ACCELEROMETER;