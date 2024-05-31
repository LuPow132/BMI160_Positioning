#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x69    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
BMI160 IMU;    
calData calib = { 0 }; 
AccelData accelData;    //Sensor data
unsigned long currentTime = millis();
unsigned long previoustTime = millis();


double AccX, AccY, AccZ;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  previoustTime = currentTime;
  currentTime = millis();
  IMU.update();
  IMU.getAccel(&accelData);
  
  AccX = accelData.accelX;
  AccY = accelData.accelY;
  AccZ = accelData.accelZ;
  int et = currentTime - previoustTime;

  Serial.print("ET:");
  Serial.print(et);
  Serial.print(",");
  Serial.print("AccX:");
  Serial.print(AccX);
  Serial.print(",");
  Serial.print("AccY:");
  Serial.print(AccY);
  Serial.print(",");  
  Serial.print("AccZ:");
  Serial.print(AccZ);
  Serial.println();

  delay(9);
}
