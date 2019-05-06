#include "ble.h"
#include <RFduinoBLE.h>

static ListenerFunction mListenerFunction = NULL;
static bool mConnected = false;

BLE::BLE()
{
}

BLE::~BLE()
{
}

void BLE::Init()
{
  RFduinoBLE.deviceName = "AirQ";
  RFduinoBLE.advertisementInterval = 1000; //Sets the interval to 1 second
  RFduinoBLE.txPowerLevel = +4; //Sets the transmit power to max +4dBm
  RFduinoBLE.begin();
}

void BLE::RegisterListener(ListenerFunction aFunction)
{
  mListenerFunction = aFunction;
}

void BLE::Send(char* aData, byte aLen)
{
  byte len = max(aLen,sizeof(mInfo));
  memcpy(mInfo,aData,len);
  RFduinoBLE.send(mInfo,len);
  RFduinoBLE.advertisementData = mInfo;
}

void BLE::Delay(int aDelayMs)
{
  RFduino_ULPDelay( aDelayMs );
}

ListenerFunction BLE::GetListener()
{
  return mListenerFunction;
}

void RFduinoBLE_onReceive(char *data, int len)
{
  if(mListenerFunction != NULL)
  {
    (BLE::GetListener())(data[0],&data[1],len-1);
  }
}

bool BLE::IsConnected()
{
  return mConnected;
}

void RFduinoBLE_onConnect()
{
  mConnected = true;
}

void RFduinoBLE_onDisconnect()
{
  mConnected = false;
}
