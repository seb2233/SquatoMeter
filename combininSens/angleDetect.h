#ifndef ANGLE_DETECT_H
#define ANGLE_DETECT_H 

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <SparkFun_BMI270_Arduino_Library.h>

class angDet {
public:
    
    angDet(uint8_t sda, uint8_t scl);

   
    bool begin();

   
    void update();
    
    void sleep();

    
    float getRoll() const { 
        return KalmanAngleRoll; 
        }


    float getPitch() const { 
        return KalmanAnglePitch;
        }

    float getRateRoll() const { 
        return RateRoll; 
        }


    float getRatePitch() const { 
        return RatePitch; 
        }

    float getRateYaw() const { 
        return RateYaw; 
        }

    float getAccX() const { 
        return AccX; 
        }

    float getAccY() const { 
        return AccY; 
        }
        
    float getAccZ() const {
        return AccZ; 
        }

private:
    uint8_t _sda;
    uint8_t _scl;

    
    //change of Acc x Gyro sensor -- more complex 
    BMI270 _imu;  
  
    float RateCalibrationRoll;
    float RateCalibrationPitch;
    float RateCalibrationYaw;
    int   RateCalibrationNumber;

   
    float RateRoll;
    float RatePitch;
    float RateYaw;


    
    float AccX;
    float AccY;
    float AccZ;

    
    float AngleRoll;
    float AnglePitch;


    
    float KalmanAngleRoll;
    float KalmanUncertaintyAngleRoll;
    float KalmanAnglePitch;
    float KalmanUncertaintyAnglePitch;

    float Kalman1DOutput[2];

    uint32_t LoopTimer;

    
    void gyro_signals();
    void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
};

#endif