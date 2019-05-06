#ifndef CCS811_H
#define CCS811_H

#include "Arduino.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define CCS811_ADDRESS                (0x5A)
/*=========================================================================*/

class CCS811
{
public:
	CCS811(void);
	~CCS811(void) {};

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware and checks for communication.
    @param  addr Optional I2C address the sensor can be found on. Default is 0x5A
    @returns 0 if device is set up
             -1 if communication fails
             -2 error occured during init
             -3 device is in boot mode
*/
/**************************************************************************/
	int Begin(uint8_t addr = CCS811_ADDRESS);

  bool IsInitialised();

  /**************************************************************************/
  /*
    @brief  set the humidity and temperature compensation for the sensor.
    @param humidity the humidity data as a percentage. For 55% humidity, pass in integer 55.
    @param temperature the temperature in degrees C as a decimal number. For 25.5 degrees C, pass in 25.5
  */
  /**************************************************************************/
	void SetEnvironmentalData(uint8_t humidity, double temperature);

  /**************************************************************************/
  /* 
      @brief  calculate the temperature using the onboard NTC resistor.
      @returns temperature as a double.
  */
  /**************************************************************************/
	double CalculateTemperature();
	
	void SetThresholds(uint16_t low_med, uint16_t med_high, uint8_t hysteresis = 50);
	
	void SetDriveMode(uint8_t mode);
	void EnableInterrupt();
	void DisableInterrupt();
	
	/**************************************************************************/
	/*! 
		@brief  returns the stored total volatile organic compounds measurement. This does does not read the sensor. To do so, call readData()
		@returns TVOC measurement as 16 bit integer
	*/
	/**************************************************************************/
	uint16_t GetTVOC() { return _TVOC; }

	/**************************************************************************/
	/*! 
		@brief  returns the stored estimated carbon dioxide measurement. This does does not read the sensor. To do so, call readData()
		@returns eCO2 measurement as 16 bit integer
	*/
	/**************************************************************************/
	uint16_t GeteCO2() { return _eCO2; }
	
	/**************************************************************************/
	/*! 
		@brief  set the temperature compensation offset for the device. This is needed to offset errors in NTC measurements.
		@param offset the offset to be added to temperature measurements.
	*/
	/**************************************************************************/
	void SetTempOffset(float offset) { _tempOffset = offset; }
	
  /**************************************************************************/
  /*
    @brief  check if data is available to be read
    @returns true if new data is available, else false
  */
  /**************************************************************************/
	bool IsDataAvailable();
 
  /**************************************************************************/
  /*
    @brief  read and store the sensor data. This data can be accessed with getTVOC() and geteCO2()
    @returns 0 if no error, error code otherwise.
  */
  /**************************************************************************/
	uint8_t ReadData();
	
  /**************************************************************************/
  /*
      @brief   read the status register and store any errors.
      @returns the error bits from the status register of the device.
  */
  /**************************************************************************/
	bool CheckError();
  uint8_t GetErrorId();
  
private:
  bool mIsInitialised;

	uint8_t _i2caddr;
	float _tempOffset;
	
	uint16_t _TVOC;
	uint16_t _eCO2;

  void swReset();

	void      write8(byte reg, byte value);
	void      write16(byte reg, uint16_t value);
	uint8_t   read8(byte reg);
	
	void read(uint8_t reg, uint8_t *buf, uint8_t num);
	void write(uint8_t reg, uint8_t *buf, uint8_t num);
		
/*=========================================================================
	REGISTER BITFIELDS
    -----------------------------------------------------------------------*/
	// The status register
	struct status 
	{   
		/* 0: no error
		*  1: error has occurred
		*/ 
		uint8_t ERROR: 1;

		// reserved : 2

		/* 0: no samples are ready
		*  1: samples are ready
		*/ 
		uint8_t DATA_READY: 1;
		uint8_t APP_VALID: 1;

		// reserved : 2

		/* 0: boot mode, new firmware can be loaded
		*  1: application mode, can take measurements
		*/
		uint8_t FW_MODE: 1;

		void set(uint8_t data){
			ERROR = data & 0x01;
			DATA_READY = (data >> 3) & 0x01;
			APP_VALID = (data >> 4) & 0x01;
			FW_MODE = (data >> 7) & 0x01;
		}
	};
	status _status;

	//measurement and conditions register
	struct meas_mode {
		// reserved : 2

		/* 0: interrupt mode operates normally
		*  1: Interrupt mode (if enabled) only asserts the nINT signal (driven low) if the new
			ALG_RESULT_DATA crosses one of the thresholds set in the THRESHOLDS register
			by more than the hysteresis value (also in the THRESHOLDS register)
		*/ 
		uint8_t INT_THRESH: 1;

		/* 0: int disabled
		*  1: The nINT signal is asserted (driven low) when a new sample is ready in
			ALG_RESULT_DATA. The nINT signal will stop being driven low when
			ALG_RESULT_DATA is read on the I²C interface.
		*/ 
		uint8_t INT_DATARDY: 1;

		uint8_t DRIVE_MODE: 3;

		uint8_t get(){
			return (INT_THRESH << 2) | (INT_DATARDY << 3) | (DRIVE_MODE << 4);
		}
	};
	meas_mode _meas_mode;
	
	struct error_id {
		/* The CCS811 received an I²C write request addressed to this station but with
		invalid register address ID */
		uint8_t WRITE_REG_INVALID: 1;

		/* The CCS811 received an I²C read request to a mailbox ID that is invalid */
		uint8_t READ_REG_INVALID: 1;

		/* The CCS811 received an I²C request to write an unsupported mode to
		MEAS_MODE */        	
		uint8_t MEASMODE_INVALID: 1;

		/* The sensor resistance measurement has reached or exceeded the maximum
		range */
		uint8_t MAX_RESISTANCE: 1;

		/* The Heater current in the CCS811 is not in range */
		uint8_t HEATER_FAULT: 1;

		/*  The Heater voltage is not being applied correctly */
		uint8_t HEATER_SUPPLY: 1;

		void set(uint8_t data){
			WRITE_REG_INVALID = data & 0x01;
			READ_REG_INVALID = (data & 0x02) >> 1;
			MEASMODE_INVALID = (data & 0x04) >> 2;
			MAX_RESISTANCE = (data & 0x08) >> 3;
			HEATER_FAULT = (data & 0x10) >> 4;
			HEATER_SUPPLY = (data & 0x20) >> 5;
		}
	};
	error_id _error_id;

/*=========================================================================*/
};

#endif
