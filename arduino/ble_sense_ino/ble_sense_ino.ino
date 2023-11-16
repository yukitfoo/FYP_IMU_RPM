/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <Adafruit_MMC56x3.h>

// set characteristics for ble
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
/* Assign a unique ID to this sensor at the same time */
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);

String start = "0,0,0";
bool connected = false;
float acclX;
float acclY;
float acclZ;
float gyroX;
float gyroY;
float gyroZ;
float magX;
float magY;
float magZ;
long int prev_time;
long int current_time;
long int time_diff;
long int iter; 

BLEService gestureService(deviceServiceUuid); 
BLEStringCharacteristic gestureCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLENotify, 150);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  while (!Serial);  
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
      Serial.println("IMU error");
  } else {
      Serial.println("IMU OK!");
  }
  // Call begin to configure magnetometer
  if (!mmc.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
    /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
    while (1) delay(10);
  } else {
    Serial.println("MMC OK!");
  }
  // Call begin to configure BLE
  if (!BLE.begin()) {
    Serial.println("- Starting Bluetooth® Low Energy module failed!");
    while (1);
  } else {
    Serial.println("BLE advertised");
  }
  BLE.setLocalName("Seeed");
  BLE.setDeviceName("Seeed");
  BLE.setAdvertisedService(gestureService);
  gestureService.addCharacteristic(gestureCharacteristic);
  BLE.addService(gestureService);
  gestureCharacteristic.writeValue(start);
  BLE.advertise();
  Serial.println("Nano 33 BLE (Peripheral Device)");
  prev_time = 0.0;
  current_time = 0.0;
  iter = 0;
}

// the loop function runs over and over again forever
void loop() {
  sensors_event_t event;
  
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    connected = true;
    digitalWrite(LED_BUILTIN, HIGH); 
  }

  while(central.connected()) {
    time_diff = current_time - prev_time;
    prev_time = current_time;
    current_time = millis();
    // get accelerometer data
    acclX = myIMU.readFloatAccelX();
    acclY = myIMU.readFloatAccelY();
    acclZ = myIMU.readFloatAccelZ();
    // get gyroscope data
    gyroX = myIMU.readFloatGyroX();
    gyroY = myIMU.readFloatGyroY();
    gyroZ = myIMU.readFloatGyroZ();
    // get magnetometer data
    mmc.getEvent(&event);
    magX = event.magnetic.x;
    magY = event.magnetic.y;
    magZ = event.magnetic.z;
    
    gestureCharacteristic.writeValue(String(iter) + ","+ String(time_diff , 4) + "," +String(acclX, 2)+","+String(acclY, 2)+","+String(acclZ, 2)+","+String(gyroX, 2)+","+String(gyroY, 2)+","+String(gyroZ, 2)+","+String(magX, 2)+","+String(magY, 2)+","+String(magZ, 2));
    iter++;
  }
}
