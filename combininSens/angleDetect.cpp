#include "angleDetect.h"
#include <Wire.h>
#include <math.h>
#include <SparkFun_BMI270_Arduino_Library.h>
/*

    IMU angle estimation using BMI270

    This file was based on the the followin git repo: https://github.com/CarbonAeronautics/Part-XIX-2D-Kalman-filter/blob/main/ArduinoCode

    Features:
    - Reads accelerometer and gyroscope via I2C
    - Performs 2000-sample gyro bias calibration at startup
    - Computes roll and pitch from accelerometer
    - Applies 1D Kalman filter for sensor fusion
    - Runs at fixed 4 ms loop period (250 Hz)

    Units:
    - Gyro output: degrees/second
    - Acceleration: g
    - Angles: degrees

    IMPORTANT:
    - Sensor must remain stationary during calibration
    - 4 ms loop timing is required for filter stability
*/


//class constructor
angDet::angDet(uint8_t sda, uint8_t scl) {
  _sda = sda;
  _scl = scl;


  RateRoll = 0;
  RatePitch = 0;
  RateYaw = 0;

  AccX = 0;
  AccY = 0;
  AccZ = 0;

  AngleRoll = 0;
  AnglePitch = 0;

  //need sensor to be calibrated in at the start
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;
  RateCalibrationNumber = 0;



  KalmanAngleRoll = 0;
  KalmanUncertaintyAngleRoll = 4;  // 2*2
  KalmanAnglePitch = 0;
  KalmanUncertaintyAnglePitch = 4;

  Kalman1DOutput[0] = 0;
  Kalman1DOutput[1] = 0;

  LoopTimer = 0;
}



bool angDet::begin() {
  Wire.begin(_sda, _scl);
  Wire.setClock(400000);


  //address i2c 0x6 
  if (_imu.beginI2C(0x68, Wire) != BMI2_OK) {
    return false;
  }


   //this is for setting the rate of the sensor 
  _imu.setAccelODR(BMI2_ACC_ODR_400HZ);
  _imu.setGyroODR(BMI2_GYR_ODR_400HZ);

  /*needed for debugging the rate and change if needed
  int8_t result1 = _imu.setAccelODR(BMI2_ACC_ODR_400HZ);
  int8_t result2 = _imu.setGyroODR(BMI2_GYR_ODR_400HZ);
  Serial.print("ACC ODR set result: "); Serial.println(result1);
  Serial.print("GYR ODR set result: "); Serial.println(result2);
*/
  delay(100);

  // Calibration
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  LoopTimer = micros();
  return true;
}

void angDet::gyro_signals() {
  _imu.getSensorData();



  RateRoll = _imu.data.gyroX;
  RatePitch = _imu.data.gyroY;
  RateYaw = _imu.data.gyroZ;

  AccX = _imu.data.accelX;
  AccY = _imu.data.accelY;
  AccZ = _imu.data.accelZ;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / 3.142;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / 3.142;
}


//Kalman Filter is solving the problem of measuring the angle of the squat-bar by using both acc and gyro instead of just one
//of the sensors; one can do it but they create errors on their own, that is why we combine it
//this predicts the next measurement using the gyro. and correct it with the acc.
//basically this filter tracks how uncertain it is and decides based on that what sensor to trust fundamentally
void angDet::kalman_1d(float &KalmanState,
                       float &KalmanUncertainty,
                       float KalmanInput,
                       float KalmanMeasurement) {


  //KalmanState is the current best angle estimate.
  //KalmanInput is the gyro rate in degrees/second.
  //multiply by 0.04 (40ms) to get degrees rotated this step.
  //so if the gyro says we're rotating at 10 deg/s, after 40ms we add 0.4 degrees.
  //BASIC TAKEAWAY: take current angle estimate and add what the gyro has read in the last 40ms (this number was chosen by me)
  KalmanState = KalmanState + 0.01 * KalmanInput;  //was 0.004 instead of 0.04 before but needed to change it to hav 25hz sync, not 250hz

  //with every new predection the drift of the gyro becomes more prevalent
  //4 * 4 represents how noisy and how much we trust the gyro, if the number is bigger there is less trust
  KalmanUncertainty = KalmanUncertainty + 0.01 * 0.01 * 4 * 4;  //was 0.004 instead of 0.04 before but needed to change it to hav 25hz sync, not 250hz

  //the KalmanGain number gathered here will tell us how much to trust the acc. correction
  //if number is closer to 1 then we trust the acc. and if closer to 0 we ignore it since it does not need correction
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);


  //KalmanMeasurement is the angle measured from the acc
  //here we compare what the acc read vs the prediction we made with the calculation just above this
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);


  //reduce uncertainty using the acc.
  //this will be used in the next calculation
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}



void angDet::sleep() {
    Wire.beginTransmission(0x68);
    Wire.write(0x7C);
    Wire.write(0x03);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.println("IMU suspend  successful");
    } else {
        Serial.println("IMU suspend failed ");
    }
}


void angDet::update() {

  //read the data from sensor
  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;


  //calc roll
  kalman_1d(KalmanAngleRoll,
            KalmanUncertaintyAngleRoll,
            RateRoll,
            AngleRoll);

  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  //calc pitch
  kalman_1d(KalmanAnglePitch,
            KalmanUncertaintyAnglePitch,
            RatePitch,
            AnglePitch);

  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];


  //Might not use since squatting is a slower movement and therefore the accuracy does not have to be 100% perfect
  //250hz update rate
  //while (micros() - LoopTimer < 4000);
  //LoopTimer = micros();
}
