#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <BLECharacteristic.h>


extern bool recording;
extern bool shouldRestart; 
extern uint32_t recordingStartTime;


class RemoteControl : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic *pCharacteristic) override;
};

#endif
