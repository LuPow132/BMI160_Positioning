#include "FastIMU.h"
#include <Wire.h>
#include "WiFi.h"
#include <esp_now.h>

#define IMU_ADDRESS 0x69    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
BMI160 IMU;               //Change to the name of any supported IMU! 

uint8_t broadcastAddress1[] = {0xE4, 0x65, 0xB8, 0x21, 0x43, 0x20};

typedef struct dataPacket {
  float AccX;
  float AccY;
  float AccZ;
  float GyroX;
  float GyroY;
  float GyroZ;
} dataPacket;

dataPacket packet;

esp_now_peer_info_t peerInfo;

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  while (!Serial) {
    ;
  }

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");
  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  // Serial.print(accelData.accelX);
  // Serial.print("\t");
  // Serial.print(accelData.accelY);
  // Serial.print("\t");
  // Serial.print(accelData.accelZ);
  // Serial.print("\t");
  // Serial.print(gyroData.gyroX);
  // Serial.print("\t");
  // Serial.print(gyroData.gyroY);
  // Serial.print("\t");
  // Serial.print(gyroData.gyroZ);
  // Serial.println();

  packet.AccX = accelData.accelX;
  packet.AccY = accelData.accelY;
  packet.AccZ = accelData.accelZ;
  packet.GyroX = gyroData.gyroX;
  packet.GyroY = gyroData.gyroY;
  packet.GyroZ = gyroData.gyroZ;

  esp_err_t result = esp_now_send(0, (uint8_t *) &packet, sizeof(dataPacket));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(50);
}
