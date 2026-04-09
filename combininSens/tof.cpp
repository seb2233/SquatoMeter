#include "tof.h"

//class constructor
Tof::Tof(uint8_t sda, uint8_t scl){
    
    _sda = sda;
    _scl = scl;

    _distance = 0;
    _status = 0;

}

//setup the sensor
bool Tof::begin(){

    Wire.begin(_sda, _scl);
    Wire.setClock(400000);

    if(!_sensor.begin(0x29, Wire)){
        return false;
    }

    _sensor.setResolution(16);
    _sensor.setRangingFrequency(33);
    _sensor.startRanging();

    return true;
}

//read from the sensor and choose what box/boxes of the camera grid to read
//it has a 4x4 grid of 16 zones that each measure distance independently, like a very low resolution depth camera.
bool Tof::update()
{
    if (_sensor.isDataReady()){
#include "tof.h"

//class constructor
Tof::Tof(uint8_t sda, uint8_t scl){
    
    _sda = sda;
    _scl = scl;

    _distance = 0;
    _status = 0;

}

//setup the sensor
bool Tof::begin(){

    Wire.begin(_sda, _scl);
    Wire.setClock(400000);

    if(!_sensor.begin(0x29, Wire)){
        return false;
    }

    _sensor.setResolution(16);
    _sensor.setRangingFrequency(33);
    _sensor.startRanging();

    return true;
}

//read from the sensor and choose what box/boxes of the camera grid to read
//it has a 4x4 grid of 16 zones that each measure distance independently, like a very low resolution depth camera.
bool Tof::update()
{
    if (_sensor.isDataReady()){

        if (_sensor.getRangingData(&_results)){
            
            //out of the 16 zones, only reading from zone 5
            int centerIndex = 5;

            _distance = _results.distance_mm[centerIndex];
            _status = _results.target_status[centerIndex];

            return true;
        }
    }

    return false;
}


uint16_t Tof::getDistance(){

    return _distance;
}

uint8_t Tof::getStatus(){

    return _status;
}

        if (_sensor.getRangingData(&_results)){
            
            //out of the 16 zones, only reading from zone 5
            int centerIndex = 7;

            _distance = _results.distance_mm[centerIndex];
            _status = _results.target_status[centerIndex];

            return true;
        }
    }

    return false;
}


uint16_t Tof::getDistance(){

    return _distance;
}

uint8_t Tof::getStatus(){

    return _status;
}
