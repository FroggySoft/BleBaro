#ifndef HISTORY_H
#define HISTORY_H

#include "Arduino.h"

#define NR_OF_PRESSURES     120   // 1 sample every hour -> 5 days
#define NR_OF_TEMPERATURES  120   // 1 sample every minute -> 2 hours
#define NR_OF_HUMIDITYS     120
#define NR_OF_CO2S          120   // 2 hours
#define NR_OF_TVOCS         120   // 2 hours

#define MIN_PRESSURE  960
#define MAX_PRESSURE  1040

#define MIN_CO2		    0
#define MAX_CO2       32000

class history
{
public:
	history(void);
	~history(void) {};

	void ClearHistory();
  
  void AddTemperature(double aTemperature);
  void AddPressure(int aPressure);
  void AddHumidity(int aHumidity);
	void AddCO2(int aPpmCO2);
  void AddTVOC(int aPpbTVOC);
  
  double GetTemperature(byte aSampleAgo);
  int GetPressure(byte aSampleAgo);
  int GetHumidity(byte aSampleAgo);
  int GetCO2(byte aSampleAgo);
  int GetTVOC(byte aSampleAgo);
	
private:
  void readHistory();
  void storePressure();
  bool checkTime(unsigned long& aLastTime, unsigned long aInterval);
  void moveOneSample(int* aArray, int aLen);
  
  int  mTemperature[NR_OF_TEMPERATURES+1];    // in units of 1/10 degree C
  int  mPressure[NR_OF_PRESSURES+1];          // as offset to minimum pressure (960mBar)
  int  mHumidity[NR_OF_HUMIDITYS+1];
	int  mCO2[NR_OF_CO2S+1];
  int  mTVOC[NR_OF_TVOCS+1];

  unsigned long mLastStoredTemperatureMs;
  unsigned long mLastStoredPressureMs;
  unsigned long mLastStoredHumidityMs;
  unsigned long mLastStoredCO2Ms;
  unsigned long mLastStoredTVOCMs;
};

#endif
