#ifndef BLE_H
#define BLE_H

#include "Arduino.h"

typedef void (*ListenerFunction)(char aCmd, char* aData, char aLen);

class BLE
{
public:
	BLE(void);
	~BLE(void);

  void Init();
  static ListenerFunction GetListener();
  void RegisterListener(ListenerFunction aFunction);
	void Send(char* aData, byte aLen);
	void Delay(int aDelayMs);
  bool IsConnected();
  
private:
	char mInfo[31];
};

#endif
