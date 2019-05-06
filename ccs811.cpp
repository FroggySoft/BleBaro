#include "ccs811.h"
#include <wire.h>

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
        CCS811_STATUS = 0x00,
        CCS811_MEAS_MODE = 0x01,
        CCS811_ALG_RESULT_DATA = 0x02,
        CCS811_RAW_DATA = 0x03,
        CCS811_ENV_DATA = 0x05,
        CCS811_NTC = 0x06,
        CCS811_THRESHOLDS = 0x10,
        CCS811_BASELINE = 0x11,
        CCS811_HW_ID = 0x20,
        CCS811_HW_VERSION = 0x21,
        CCS811_FW_BOOT_VERSION = 0x23,
        CCS811_FW_APP_VERSION = 0x24,
        CCS811_ERROR_ID = 0xE0,
        CCS811_SW_RESET = 0xFF,
    };
	
	//bootloader registers
	enum
	{
		CCS811_BOOTLOADER_APP_ERASE = 0xF1,
		CCS811_BOOTLOADER_APP_DATA = 0xF2,
		CCS811_BOOTLOADER_APP_VERIFY = 0xF3,
		CCS811_BOOTLOADER_APP_START = 0xF4
	};
	
	enum
	{
		CCS811_DRIVE_MODE_IDLE = 0x00,
		CCS811_DRIVE_MODE_1SEC = 0x01,
		CCS811_DRIVE_MODE_10SEC = 0x02,
		CCS811_DRIVE_MODE_60SEC = 0x03,
		CCS811_DRIVE_MODE_250MS = 0x04,
	};

/*=========================================================================*/

#define CCS811_HW_ID_CODE			0x81

#define CCS811_REF_RESISTOR			100000

CCS811::CCS811():
  mIsInitialised(false)
{}


int CCS811::Begin(uint8_t addr)
{
  mIsInitialised = false;
	_i2caddr = addr;
	
	swReset();
	delay(100);
	
	//check that the HW id is correct
  if( read8(CCS811_HW_ID) != CCS811_HW_ID_CODE)
  {
		return -1;
  }
  
	//try to start the app
	this->write(CCS811_BOOTLOADER_APP_START, NULL, 0);
	delay(100);
	
	//make sure there are no errors and we have entered application mode
	if(CheckError()) 
	  return -2;
    
	if(!_status.FW_MODE) 
	  return -3;   // device is in boot mode
	
	DisableInterrupt();
	
	//default to read every 10 seconds
	SetDriveMode(CCS811_DRIVE_MODE_10SEC);

  mIsInitialised = true;
	return 0;
}

bool CCS811::IsInitialised()
{
  return mIsInitialised;
}

/**************************************************************************/
/*! 
    @brief  sample rate of the sensor.
    @param  mode one of CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC, CCS811_DRIVE_MODE_250MS.
*/
void CCS811::SetDriveMode(uint8_t mode)
{
	_meas_mode.DRIVE_MODE = mode;
	this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*! 
    @brief  enable the data ready interrupt pin on the device.
*/
/**************************************************************************/
void CCS811::EnableInterrupt()
{
	_meas_mode.INT_DATARDY = 1;
	this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*! 
    @brief  disable the data ready interrupt pin on the device
*/
/**************************************************************************/
void CCS811::DisableInterrupt()
{
	_meas_mode.INT_DATARDY = 0;
	this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*! 
    @brief  checks if data is available to be read.
    @returns True if data is ready, false otherwise.
*/
/**************************************************************************/
bool CCS811::IsDataAvailable()
{
  bool result = false;

  if (mIsInitialised)
  {
  	_status.set(read8(CCS811_STATUS));
  	if(_status.DATA_READY)
		  result = true;
  }
	return result;
}

uint8_t CCS811::ReadData()
{
	if(!IsDataAvailable())
		return false;
	else
	{
		uint8_t buf[8];
		this->read(CCS811_ALG_RESULT_DATA, buf, 8);

		_eCO2 = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
		_TVOC = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);
		
		if(_status.ERROR)
			return buf[5];
			
		else return 0;
	}
}

void CCS811::SetEnvironmentalData(uint8_t humidity, double temperature)
{
	/* Humidity is stored as an unsigned 16 bits in 1/512%RH. The
	default value is 50% = 0x64,  0x00. As an example 48.5%
	humidity would be 0x61, 0x00.*/
	
	/* Temperature is stored as an unsigned 16 bits integer in 1/512
	degrees; there is an offset: 0 maps to -25°C. The default value is
	25°C = 0x64, 0x00. As an example 23.5% temperature would be
	0x61, 0x00.
	The internal algorithm uses these values (or default values if
	not set by the application) to compensate for changes in
	relative humidity and ambient temperature.*/
	
	uint8_t hum_perc = humidity << 1;
	
	float fractional = modf(temperature, &temperature);
	uint16_t temp_high = (((uint16_t)temperature + 25) << 9);
	uint16_t temp_low = ((uint16_t)(fractional / 0.001953125) & 0x1FF);
	
	uint16_t temp_conv = (temp_high | temp_low);

	uint8_t buf[] = {hum_perc, 0x00,
		(uint8_t)((temp_conv >> 8) & 0xFF), (uint8_t)(temp_conv & 0xFF)};
	
	this->write(CCS811_ENV_DATA, buf, 4);

}

double CCS811::CalculateTemperature()
{
	uint8_t buf[4];
	this->read(CCS811_NTC, buf, 4);

	uint32_t vref = ((uint32_t)buf[0] << 8) | buf[1];
	uint32_t vntc = ((uint32_t)buf[2] << 8) | buf[3];
	
	//from ams ccs811 app note
	uint32_t rntc = vntc * CCS811_REF_RESISTOR / vref;
	
	double ntc_temp;
	ntc_temp = log((double)rntc / CCS811_REF_RESISTOR); // 1
	ntc_temp /= 3380; // 2
	ntc_temp += 1.0 / (25 + 273.15); // 3
	ntc_temp = 1.0 / ntc_temp; // 4
	ntc_temp -= 273.15; // 5
	return ntc_temp - _tempOffset;

}

/**************************************************************************/
/*! 
    @brief  set interrupt thresholds
    @param low_med the level below which an interrupt will be triggered.
    @param med_high the level above which the interrupt will ge triggered.
    @param hysteresis optional histeresis level. Defaults to 50
*/
/**************************************************************************/
void CCS811::SetThresholds(uint16_t low_med, uint16_t med_high, uint8_t hysteresis)
{
	uint8_t buf[] = {(uint8_t)((low_med >> 8) & 0xF), (uint8_t)(low_med & 0xF),
	(uint8_t)((med_high >> 8) & 0xF), (uint8_t)(med_high & 0xF), hysteresis};
	
	this->write(CCS811_THRESHOLDS, buf, 5);
}

void CCS811::swReset()
{
	//reset sequence from the datasheet
	uint8_t seq[] = {0x11, 0xE5, 0x72, 0x8A};
	this->write(CCS811_SW_RESET, seq, 4);
}

bool CCS811::CheckError()
{
	_status.set(read8(CCS811_STATUS));
	return _status.ERROR;
}

uint8_t CCS811::GetErrorId()
{
  return read8(CCS811_ERROR_ID);
}


void CCS811::write8(byte reg, byte value)
{
	this->write(reg, &value, 1);
}

/**************************************************************************/
/*! 
    @brief  read one byte of data from the specified register
    @param  reg the register to read
    @returns one byte of register data
*/
/**************************************************************************/
uint8_t CCS811::read8(byte reg)
{
	uint8_t ret;
	this->read(reg, &ret, 1);
	
	return ret;
}

void CCS811::read(uint8_t reg, uint8_t *buf, uint8_t num)
{
	uint8_t value;
	uint8_t pos = 0;
	
	//on arduino we need to read in 32 byte chunks
	while(pos < num){
		
		uint8_t read_now = min((uint8_t)32, (uint8_t)(num - pos));
		Wire.beginTransmission((uint8_t)_i2caddr);
		Wire.write((uint8_t)reg + pos);
		Wire.endTransmission();
		Wire.requestFrom((uint8_t)_i2caddr, read_now);
		
		for(int i=0; i<read_now; i++){
			buf[pos] = Wire.read();
			pos++;
		}
	}
}

void CCS811::write(uint8_t reg, uint8_t *buf, uint8_t num)
{
	Wire.beginTransmission((uint8_t)_i2caddr);
	Wire.write((uint8_t)reg);
	Wire.write((uint8_t *)buf, num);
	Wire.endTransmission();
}
