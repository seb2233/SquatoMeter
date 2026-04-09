//i2c control
#include <Wire.h>

//self made header files
#include "tof.h"
#include "angleDetect.h"
#include "remoteControl.h"

//bluetooth header files
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define WAKE_PIN GPIO_NUM_44
#define LED_RED  7
#define LED_GREEN 8

#define SDA_PIN 5
#define SCL_PIN 6

//bluetooth id's for sending the data over bluetooth
#define SERVICE_UUID        "1a43422c-f676-401b-bdd2-be3d8695ca60"
#define CHARACTERISTIC_UUID "d8d02583-8540-4411-8b27-deec23578ca3"
//samples size max sent from Squato-Meter to phone 
#define MAX_SAMPLES 35000  // 100 Hz * 280 sec ~ 4.2 min

//mark that there is data to send after recording stops 
bool bufferPending = false;

//data structure of packets
struct Sample {
    float timestamp;
    uint16_t distance;
    float roll;
    float pitch;
    float accX;
    float accY;
    float accZ;
    float gyroRoll;
    float gyroPitch;
    float gyroYaw;
};

//buffer for when sending, had to do this so samples as they are recorded are saved on the PSRAM of the ESP32
Sample* buffer = nullptr;    
uint16_t sampleIndex = 0;


//setting up the pins from the header files of the used sensors 
Tof tof(SDA_PIN, SCL_PIN);
angDet imu(SDA_PIN, SCL_PIN);

//  true/false if recording 
bool recording = false;


// tof will have a different sampling rate
uint32_t lastToFTime = 0;
const uint32_t tofPeriod = 33; // ~30 Hz
bool freshToF = false;
uint16_t lastDist = 0;




uint32_t recordingStartTime = 0; // time at start of recording
const uint32_t samplePeriod = 10; // 100 Hz for imu, tof will be the same
uint32_t lastSampleTime = 0;
bool shouldRestart = false;  



//pointer to bluetooth characteristics object 
BLECharacteristic *pCharacteristic;

//this enables itself (runs automatically) depending on the status of a bluetooth connection; 
//if the device gets disconnected/connected the Squato-Meter will act in accordance going back to advertising
//if disconnected
class MyServerCallbacks:
     public BLEServerCallbacks {

        //predefined event function found in BLE header file
        void onConnect(BLEServer* pServer) override {
            Serial.println("Device connected");
            digitalWrite(LED_GREEN, HIGH);  // green on when connected
        }

        //same thing here
        void onDisconnect(BLEServer* pServer) override {
            Serial.println("Device disconnected");
            digitalWrite(LED_GREEN, LOW);   // green off when disconnected

            //restart advertising
            BLEDevice::startAdvertising();  
            Serial.println("Advertising restarted");
        }
};

//sleep
void goToSleep() {
    Serial.println("Going to deep sleep...");
    
    imu.sleep();
    delay(100);

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    
    if (BLEDevice::getServer()->getConnectedCount() > 0) {
        BLEDevice::getServer()->disconnect(0);
    }
    delay(500);
    
    
    
    Serial.println("Sleeping now");
    delay(100);
    esp_deep_sleep_start();
}



void setup() {

    Serial.begin(115200);
    
    //set buffer size 
    buffer = (Sample*)ps_malloc(MAX_SAMPLES * sizeof(Sample));

    //used for debugging, was used to enable the PSRAM where the data would be stored 
    /*
    if (!buffer) {
        Serial.println("PSRAM allocation failed! Halting.");
        while(true);
    }
    Serial.printf("Buffer allocated: %d bytes in PSRAM\n", MAX_SAMPLES * sizeof(Sample));
    */

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(WAKE_PIN, INPUT_PULLUP);

    // red on immediately to show device is on
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);

    //set up the sensors from their functions found in their respective header files
    if (!tof.begin()) {
        Serial.println("ToF sensor not detected");
        delay(1000);
    }
    else{Serial.println("ToF sensor initialized");}


    if (!imu.begin()) {
        Serial.println("IMU not detected");
        delay(1000);
    }
    else{Serial.println("IMU initialized");}


    //name of device on bluetooth list 
    BLEDevice::init("Squato-Meter");
    BLEDevice::setMTU(512);  // size of transmission buffer 

    //start device basically by activating the "server" part of the bluetooth layer 
    BLEServer *pServer = BLEDevice::createServer();

    //sets up the class above to handle Squato-Meter connections and discconections from a phone or other device
    pServer->setCallbacks(new MyServerCallbacks());

    //bluetooth "service" to enable transmission of data from sensors 
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_WRITE |
                            BLECharacteristic::PROPERTY_NOTIFY
                        );


    //enables device connecting to Squato-Meter to get push updates(used for seeing data in real time on phone)
    pCharacteristic->addDescriptor(new BLE2902());

    //start the ble "server"
    pService->start();

    //custom callback class found in remote.h file - used to start/stop recording 
    pCharacteristic->setCallbacks(new RemoteControl()); 

    //creates advertising object to start advertising for a phone to connect
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
}



//send a single sample via BLE
void sendSampleBLE(Sample &s) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
             s.timestamp, s.distance,
             s.roll, s.pitch,
             s.accX, s.accY, s.accZ,
             s.gyroRoll, s.gyroPitch, s.gyroYaw);

    if (BLEDevice::getServer()->getConnectedCount() > 0) {
        pCharacteristic->setValue(payload);
        pCharacteristic->notify();
    }
}




void loop() {

    // button press = restart
    if (digitalRead(WAKE_PIN) == LOW) {
        delay(50);
        if (digitalRead(WAKE_PIN) == LOW) {
            Serial.println("Button pressed, restarting...");
            ESP.restart();
        }
    }

    //command from phone to restart
    if (shouldRestart) {
        Serial.println("Restarting ESP32...");
        delay(100);
        ESP.restart();
    }

    if (!recording && bufferPending) {
        delay(10);
        for (uint16_t i = 0; i < sampleIndex; i++) {
            sendSampleBLE(buffer[i]);
            delay(3);
        }
        sampleIndex = 0;
        bufferPending = false;
    }

    if (recording) {
        uint32_t now = millis();

        if (recordingStartTime == 0) {
            recordingStartTime = now;
            lastSampleTime = now;
            lastToFTime = now;
        }

        // ToF runs independently at ~30 Hz
        if (now - lastToFTime >= tofPeriod) {
            lastToFTime += tofPeriod;
            if (tof.update()) {         // only if data was ready
                lastDist = tof.getDistance();
                freshToF = true;
            }
        }

        // IMU runs at 100 Hz
        if (now - lastSampleTime >= samplePeriod) {
            lastSampleTime += samplePeriod;
            imu.update();

            float timestamp = (millis() - recordingStartTime) / 1000.0f;
            uint16_t dist   = freshToF ? lastDist : 65535;
            freshToF = false;

            float roll      = imu.getRoll();
            float pitch     = imu.getPitch();
            float accX      = imu.getAccX();
            float accY      = imu.getAccY();
            float accZ      = imu.getAccZ();
            float gyroRoll  = imu.getRateRoll();
            float gyroPitch = imu.getRatePitch();
            float gyroYaw   = imu.getRateYaw();

            Sample s = {timestamp, dist, roll, pitch, 
                        accX, accY, accZ, 
                        gyroRoll, gyroPitch, gyroYaw};

            if (sampleIndex < MAX_SAMPLES) {
                buffer[sampleIndex++] = s;
                bufferPending = true;
            }

            sendSampleBLE(s);


        }
    }
}




            //debugging sync. issues
            /*
            uint32_t t1 = micros();
            tof.update();
            uint32_t t2 = micros();
            Serial.print("tof: "); Serial.println(t2 - t1);
            
            uint32_t t3 = micros();
            imu.update();
            uint32_t t4 = micros();
            Serial.print("imu: "); Serial.println(t4 - t3);


                        // keep serial for debugging, remove later for performance
            Serial.print("Time: "); Serial.print(timestamp, 2);
            Serial.print(" | Dist: "); Serial.print(dist);
            Serial.print(" | Roll: "); Serial.print(roll, 2);
            Serial.print(" | Pitch: "); Serial.print(pitch, 2);
            Serial.print(" | Acc: ");
            Serial.print(accX, 3); Serial.print(", ");
            Serial.print(accY, 3); Serial.print(", ");
            Serial.print(accZ, 3);
            Serial.print(" | Gyro: ");
            Serial.print(gyroRoll, 2); Serial.print(", ");
            Serial.print(gyroPitch, 2); Serial.print(", ");
            Serial.println(gyroYaw, 2);
            */

