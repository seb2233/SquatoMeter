#include "remoteControl.h"
#include <Arduino.h>

void RemoteControl::onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    
    if (value == "START") {
        recording = true;
        recordingStartTime = 0;
        Serial.println("Recording started");
    }
    else if (value == "STOP") {
        recording = false;
        Serial.println("Recording stopped");
    }
    else if (value == "RESET") {
        Serial.println("Device resetting...");
        shouldRestart = true;
    }
    else if (value == "SLEEP") {
        recording = false;
        bufferPending = false;
        Serial.println("Sleeping...");
        goToSleep();
    }
}