#include "FastIMU.h"
#include <Wire.h>
#include "WiFi.h"
#include <esp_now.h>

#define IMU_ADDRESS 0x69    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
BMI160 IMU;    
calData calib = { 0 }; 
AccelData accelData;    //Sensor data
unsigned long currentTime = millis();
unsigned long previoustTime = millis();

double AccX, AccY, AccZ;

//ESP NOW CONFIG
uint8_t broadcastAddress1[] = {0xE4, 0x65, 0xB8, 0x21, 0x43, 0x20};
typedef struct dataPacket {
  float AccX;
  float AccY;
  float AccZ;
  float time;
} dataPacket;

dataPacket packet;

esp_now_peer_info_t peerInfo;

// callback when data is sent
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
  // put your setup code here, to run once:
  Serial.begin(19200);
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

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
  delay(3000);
}

void loop() {

  // put your main code here, to run repeatedly:
  previoustTime = currentTime;
  currentTime = millis();
  IMU.update();
  IMU.getAccel(&accelData);

  packet.AccX = accelData.accelX;
  packet.AccY = accelData.accelY;
  packet.AccZ = accelData.accelZ;
  packet.time = currentTime - previoustTime;

  esp_err_t result = esp_now_send(0, (uint8_t *) &packet, sizeof(dataPacket));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  


  // Serial.print("ET:");
  // Serial.print(packet.time);
  // Serial.print(",");
  // Serial.print("AccX:");
  // Serial.print(packet.AccX);
  // Serial.print(",");
  // Serial.print("AccY:");
  // Serial.print(packet.AccY);
  // Serial.print(",");  
  // Serial.print("AccZ:");
  // Serial.print(packet.AccZ);
  // Serial.println();

  delay(10);
}
