#include "history.h"

//#include <EEPROM.h>

// take the number of samples each period. 120 in total
// preesure once every hour -> 24 samples in one section
// others: take an equal amount a samples in one section where one section correspondes with one hour
// so 1hour/24 = 3600s/24 = one sample every 150 seconds 

const unsigned long cIntervalTemperatureMs = 150000;
const unsigned long cIntervalPressureMs = (60*60000);
const unsigned long cIntervalHumidityMs = 150000;
const unsigned long cIntervalCO2Ms = 150000;
const unsigned long cIntervalTVOCMs = 150000;

history::history():
  mLastStoredTemperatureMs(0),
  mLastStoredPressureMs(0),
  mLastStoredHumidityMs(0),
  mLastStoredCO2Ms(0),
  mLastStoredTVOCMs(0)
{
  ClearHistory();
  readHistory();
}

void history::ClearHistory()
{
  memset(mTemperature,0,sizeof(mTemperature));  
  memset(mPressure,0,sizeof(mPressure));  
  memset(mHumidity,0,sizeof(mHumidity));  
  memset(mCO2,0,sizeof(mCO2));  
  memset(mTVOC,0,sizeof(mTVOC));	
}

void history::AddTemperature(double aTemperature)
{
  if (checkTime(mLastStoredTemperatureMs,cIntervalTemperatureMs))
  {
    int t = (int)(aTemperature*10);
    moveOneSample(mTemperature,NR_OF_TEMPERATURES);
    mTemperature[NR_OF_TEMPERATURES] = t;
  }
}
void history::AddPressure(int aPressure)
{
  if (checkTime(mLastStoredPressureMs,cIntervalPressureMs))
  {
    // only store the offset to the min pressure (so it fits in 1 byte) 
    int p = aPressure - MIN_PRESSURE;
    
    moveOneSample(mPressure, NR_OF_PRESSURES);
    mPressure[NR_OF_PRESSURES] = p;
//    storePressure();
  }
}
void history::AddHumidity(int aHumidity)
{
  if (checkTime(mLastStoredHumidityMs,cIntervalHumidityMs))
  {
    moveOneSample(mHumidity, NR_OF_HUMIDITYS);
    mHumidity[NR_OF_HUMIDITYS] = aHumidity; 
  }
}

void history::AddTVOC(int aPpbTVOC)
{
  if (checkTime(mLastStoredTVOCMs,cIntervalTVOCMs))
  {
    moveOneSample(mTVOC, NR_OF_TVOCS);
    mTVOC[NR_OF_TVOCS] = aPpbTVOC; 
  }
}

void history::AddCO2(int aPpmCO2)
{  
  if (checkTime(mLastStoredCO2Ms,cIntervalCO2Ms))
  
  {
    moveOneSample(mCO2, NR_OF_CO2S);
    mCO2[NR_OF_CO2S] = aPpmCO2;
  }
}

double history::GetTemperature(byte aSampleAgo)
{
  double t = 0;
  if( aSampleAgo < NR_OF_TEMPERATURES)
  {
    t = mTemperature[aSampleAgo];
    t /= 10;
  }
  return t;
}

int history::GetPressure(byte aSampleAgo)
{
  int p = 0;
  if( aSampleAgo < NR_OF_PRESSURES)
  {
    p = mPressure[aSampleAgo];
    p += MIN_PRESSURE;
  }
  return p;
}

int history::GetHumidity(byte aSampleAgo)
{
  int h = 0;
  if( aSampleAgo < NR_OF_HUMIDITYS)
  {
    h = mHumidity[aSampleAgo];
  }
  return h;
}

int history::GetCO2(byte aSampleAgo)
{
  int t = 0;
  if( aSampleAgo < NR_OF_CO2S)
  {
    t = mCO2[aSampleAgo];
  }
  return t;
}

int history::GetTVOC(byte aSampleAgo)
{
  int t = 0;
  if( aSampleAgo < NR_OF_TVOCS)
  {
    t = mTVOC[aSampleAgo];
  }
  return t;
}

bool history::checkTime(unsigned long& aLastTime, unsigned long aInterval)
{
  bool passed = false;
  unsigned long now = millis();
  
  if (now < aLastTime)   // we got an overflow of the millis (once every 50 days)
  {
    aLastTime = 0;  // just reset and accept the difference
  }
  if ((aLastTime + aInterval) < now)
  {
    passed = true;
    aLastTime = now;
  }
  return passed;
}

void history::moveOneSample(int* aArray, int aLen)
{
  for( int i=1; i<=aLen; i++)
  {
    aArray[i-1] = aArray[i];
  }
}

void history::storePressure(void)
{
  for(int i=0; i<=NR_OF_PRESSURES; i++)
  {
    //EEPROM.write(i+2,mPressHistory[i] - MIN_PRESSURE);
  }
}

void history::readHistory()
{
  /***
  // when magic numbers do not exists: create then
  if(EEPROM.read(0)!=0x5A || EEPROM.read(1)!=0xBC)
  {
    EEPROM.write(0, 0x5A);
    EEPROM.write(1, 0xBC);
    memset(mPressHistory,-1,sizeof(mPressHistory));
  }
  else  // read old values form NV mem
  {
    for(int i=0; i<=NR_OF_PRESSURES; i++)
    {
      mPressHistory[i] = EEPROM.read(i+2) + MIN_PRESSURE;
    }
  }
  ***/
}
