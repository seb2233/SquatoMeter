#ifndef TOF_H // if the file hasnt been included yet
#define TOF_H // then include it 

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

class Tof{
    public: //what must main be able to do? create the object, initialize it, update it, read distance
        Tof(uint8_t sda, uint8_t scl);

        bool begin();
        bool update(); // i2C read

        uint16_t getDistance(); //returns stored value, does not trigger i2C
        uint8_t getStatus(); // returns stored value of status of latest sensor read; not used really

    private: // what data must be storred internally? sensor object, result and latest measurement
        uint8_t _sda;
        uint8_t _scl;

        SparkFun_VL53L5CX _sensor;
        VL53L5CX_ResultsData _results;

        uint16_t _distance; //related to getDistance
        uint8_t _status; // related to GetStatus


};

#endif
