#ifndef BMP180_H
#define BMP180_H

#include "Arduino.h"

class BMP180
{
public:
	BMP180(void) {};
	~BMP180(void) {};
	
	// Initialize library for subsequent pressure measurements
	char Init(void);

	// Begin a temperature reading.
	// Will return delay in ms to wait, or 0 if I2C error
	char StartTemperature(void);

	// Retrieve a previously-started temperature reading.
	// Requires begin() to be called once prior to retrieve calibration parameters.
	// Requires startTemperature() to have been called prior and sufficient time elapsed.
	// T: external variable to hold result.
	// Returns 1 if successful, 0 if I2C error.
	char GetTemperature(double &T);

	// Begin a pressure reading.
	// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
	// Will return delay in ms to wait, or 0 if I2C error.
	char StartPressure(char oversampling);

	// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
	// Requires begin() to be called once prior to retrieve calibration parameters.
	// Requires startPressure() to have been called prior and sufficient time elapsed.
	// Requires recent temperature reading to accurately calculate pressure.

	// P: external variable to hold pressure.
	// T: previously-calculated temperature.
	// Returns 1 for success, 0 for I2C error.

	// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
	char GetPressure(double &P, double &T);

	// Given a pressure P (mb) taken at a specific altitude (meters),
	// return the equivalent pressure (mb) at sea level.
	// This produces pressure readings that can be used for weather measurements.
	double Sealevel(double P, double A);

	// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
	// return altitude (meters) above baseline.
	double Altitude(double P, double P0);

	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error
	char GetError(void);

private:
	// Read an array of bytes from device
	// values: external array to hold data. Put starting register in values[0].
	// length: number of bytes to read
	char readBytesBmp180(unsigned char *values, char length);

	// Write an array of bytes to device
	// values: external array of data to write. Put starting register in values[0].
	// length: number of bytes to write
	char writeBytes(unsigned char *values, char length);

	// Read a signed integer (two bytes) from device
	// address: register to start reading (plus subsequent register)
	// value: external variable to store data (function modifies value)
	char readInt(char address, int &value);

	// Read an unsigned integer (two bytes) from device
	// address: register to start reading (plus subsequent register)
	// value: external variable to store data (function modifies value)
	char readUInt(char address, unsigned int &value);

	int AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
	unsigned int AC4,AC5,AC6; 
	double c5,c6,mc,md,x0,x1,x2,my0,my1,my2,p0,p1,p2;
	char _error;
};

#endif
