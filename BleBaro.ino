
/*
 * Niveaus van kooldioxide in lucht worden meestal uitgedrukt in delen per miljoen (ppm) in volume. De normale concentratie 
 * van CO2 in de buitenlucht is tussen 300 en 400 ppm. Indoor levels zijn meestal een stuk hoger, voornamelijk als gevolg 
 * van de concentratie van de uitgeademde lucht van de mensen in het gebouw. CO2-gehalte in kantoorgebouwen variëren meestal 
 * tussen 350-2500 ppm. Het is aangetoond dat niveaus boven 1000 ppm neiging te resulteren in gezondheidsklachten en een 
 * algemene richtlijn is dat het niveau dient beneden 800 ppm ieders comfort.
 * 
 * The equivalent CO2 (eCO2) output range for CCS811 is from 400ppm up to 32768ppm.
 * 
 * TVOC – concentratie-afhankelijke werking
 * < 0,2 mg/m³ Geen irritaties of beïnvloeding van het welzijn.
 * 0,2 - 3,0 mg/m³  Irritaties of beïnvloeding van het welzijn mogelijk, als er sprake is van wisselwerking met andere blootstellingsparameters.
 * 3,0 - 25 mg/m³  Blootstelling leidt tot een effect, hoofdpijn mogelijk, als er sprake is van wisselwerking met andere blootstellingsparameters.
 * > 25 mg/m³  Hoofdpijn. Andere neurotoxische effecten naast hoofdpijn mogelijk.
 * 
 * 1 mg/m³ = 1000ppb
 * The equivalent Total Volatile Organic Compound (eTVOC) output range for CCS811 is from 0ppb up to 32768ppb.
 */
 
//#define Use_Serial 
//#define PublishString
#define PublishArray
//#define USE_BMP180
#define USE_BME280

#include <Wire.h>
#include "ble.h"
#ifdef USE_BMP180
#include "BMP180.h"
#endif
#ifdef USE_BME280
#include "BME280.h"
#endif
#include "CCS811.h"
#include "history.h"

// IO for the I2C bus to the sensor
const int PinScl = 6;
const int PinSda = 5;

BLE     mBle;
#ifdef USE_BMP180
BMP180  mBmp180;
#endif
#ifdef USE_BME280
BME280  mBme280;
#endif
CCS811  mCcs811;
history mHistory;

void setup()
{
#ifdef Use_Serial
  Serial.begin(9600);
  Serial.println("Reset");
#endif

  Wire.beginOnPins(PinScl, PinSda);
  
#ifdef USE_BMP180
  if (!mBmp180.Init())
#endif
#ifdef USE_BME280
  if (!mBme280.Init())
#endif
  {
#ifdef Use_Serial
    Serial.println(F("Failed to get correct press/temp sensor."));
#endif
  }

  int r = mCcs811.Begin();
  if (r != 0)
  {
#ifdef Use_Serial
    Serial.print(F("Failed to start CO2 sensor, error = "));
    Serial.println(r,DEC);
#endif
  }

  mBle.Init();
  mBle.RegisterListener(OnCommand);
}

void loop()
{
  double temperature = 0;
  int humidity = 50;
  int pressure = 0;
  int ppmCO2 = 0;
  int ppbTVOC = 0;
  byte ccsError = 0;
  
#ifdef USE_BMP180

  byte delay1 = mBmp180.StartTemperature();
  byte delay2 = 0;

  if( delay1 > 0 )
  {
    delay(delay1);
    if( mBmp180.GetTemperature(temperature)==1 )
    {
      //temperature -= 3.8f;
      mHistory.AddTemperature(temperature);
      mHistory.AddHumidity(humidity);
      delay2 = mBmp180.StartPressure(1);    // start for next sample
#ifdef Use_Serial
      Serial.print(F("Temp="));
      Serial.print(temperature,1);
#endif
      if( delay2 > 0 )
      {
        delay(delay2);
        double p;
        if( mBmp180.GetPressure(p,temperature)==1 )
        {
          pressure = p;
#ifdef Use_Serial
          Serial.print(F(" Pressure="));
          Serial.print(pressure);
#endif
          mHistory.AddPressure(pressure);
        }
      }
    }
  }
  else
  {
#ifdef Use_Serial
    Serial.println("Failed to read BMP180");
#endif
  }
#endif

#ifdef USE_BME280
  if (mBme280.IsAvailable())
  {
    temperature = mBme280.readTemperature();
    float p = mBme280.readPressure() / 100.0f;
    pressure = (int) p;
  
    float h = mBme280.readHumidity();
    humidity = (int)h;
  
    mHistory.AddTemperature(temperature);
    mHistory.AddPressure(pressure);
    mHistory.AddHumidity(humidity);
  #ifdef Use_Serial
        Serial.print(F("Temp="));
        Serial.print(temperature,1);
        Serial.print(F(" Pressure="));
        Serial.print(pressure);
        Serial.print(F(" Humidity="));
        Serial.print(humidity);
  #endif
  }
  else
  {
#ifdef Use_Serial
    Serial.println("Failed to read BME280");
#endif
  }
#endif

  if (mCcs811.IsDataAvailable())
  {
    mCcs811.SetEnvironmentalData(humidity,temperature);
    if(mCcs811.ReadData())
    {
      ppmCO2 = mCcs811.GeteCO2();
      ppbTVOC = mCcs811.GetTVOC();
#ifdef Use_Serial
      Serial.print(F(" CO2: "));
      Serial.print(ppmCO2);
      Serial.print(F("ppm, TVOC: "));
      Serial.print(ppbTVOC);
      Serial.print(F("ppb"));
#endif
      mHistory.AddCO2(ppmCO2);
      mHistory.AddTVOC(ppbTVOC);

      if (mCcs811.CheckError())
      {
        ccsError = mCcs811.GetErrorId();
#ifdef Use_Serial
        Serial.print(F(" Error: "));
        Serial.print(ccsError,BIN);        
#endif      
      }
    }
  }

#ifdef Use_Serial
  Serial.println();
#endif
  // if the deviceName and advertisementData are too long to fix into the 31 byte
  // ble advertisement packet, then the advertisementData is truncated first down to
  // a single byte, then it will truncate the deviceName

  if (mBle.IsConnected())
  {
    #ifdef PublishString
    int tempi = temperature ;
    int tempf = 10*(temperature-tempi);
    char info[31];
    sprintf(info,"%2d.%1d,%4d,%2d,%4d,%3d",tempi,tempf,pressure,humidity,ppmCO2,ppbTVOC);  
    mBle.Send(info,strlen(info));
    #endif
    
    #ifdef PublishArray
    char buffer[12];
    packData(0, temperature, pressure, humidity, ppmCO2, ppbTVOC, ccsError, buffer);
    mBle.Send(buffer,sizeof(buffer));
    #endif
  }

  mBle.Delay(10000);
}

void packData(int aNr, double aT, int aP, int aH, int aC, int aV, byte aError, char *aBuffer)
{
  int temp = (int)(aT*10.0) ;
  aBuffer[0] = aNr & 0x00FF;
  aBuffer[1] = temp >> 8;
  aBuffer[2] = temp & 0x00FF;
  aBuffer[3] = aP >> 8;
  aBuffer[4] = aP & 0x00FF;
  aBuffer[5] = aH >> 8;
  aBuffer[6] = aH & 0x00FF;
  aBuffer[7] = aC >> 8;
  aBuffer[8] = aC & 0x00FF;
  aBuffer[9] = aV >> 8;
  aBuffer[10]= aV & 0x00FF;
  aBuffer[11] = aError;
}

void OnCommand(char aCmd, char* aData, char aLen)
{
  if (aCmd == 'H')
  {
    int d = aData[0];  
    char buffer[12];
    double temperature = mHistory.GetTemperature(d);
    int pressure = mHistory.GetPressure(d);
    int humidity = mHistory.GetHumidity(d);
    int ppmCO2 = mHistory.GetCO2(d);
    int ppbTVOC = mHistory.GetTVOC(d);
    packData(d, temperature, pressure, humidity, ppmCO2, ppbTVOC, 0, buffer); // error is not stored in history
    mBle.Send(buffer,sizeof(buffer));
  }
}
