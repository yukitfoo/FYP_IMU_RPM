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
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// set characteristics for ble
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

// Create instance of BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);


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
bool first = true;
imu::Vector<3> euler;
// imu::Quaternion prev_quat;


BLEService gestureService(deviceServiceUuid); 
BLEStringCharacteristic gestureCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLENotify, 150);
int calibration = 0;
enum {
  CALIBRATION_STATE = 0,
  RUNNING_STATE
} STATE;


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  while (!Serial);  
  //Call .begin() to configure the IMUs
  if (!bno.begin()) {
      Serial.println("IMU error");
      while (1);
  } else {
      Serial.println("IMU OK!");
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
  delay(1000);
  STATE = CALIBRATION_STATE;
}
String displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  String f = String(system, DEC) +","+String(gyro, DEC) +","+String(accel, DEC) +","+String(mag, DEC);
  return f;
}

String sendData(void) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY );
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE  );
  // imu::Quaternion quat = bno.getQuat();
  // double RPM = 0;
  // if (first) {
  //   prev_quat = quat;
  //   first = false;
  // } else {
  //   double theta_1 = acos(prev_quat.w());
  //   double theta_2 = acos(quat.w());
  //   double dot_prod = prev_quat.x()*quat.x() + prev_quat.y()*quat.y() + prev_quat.z()*quat.z();
  //   double angle = acos((cos(theta_1/2)*cos(theta_2/2))+(dot_prod*(sin(theta_1/2)*sin(theta_2/2))));
  //   RPM = angle*60/6.283185;
  //   Serial.println(String(RPM, 3) + " , " + String(angle, 4));
  // }
  accel[0] -= gravity[0];
  accel[1] -= gravity[1];
  accel[2] -= gravity[2];
  // double angular_velocity = pow(pow(gyro.x(), 2) + pow(gyro.y(), 2) + pow(gyro.z(), 2), 0.5);

  // double RPM = angular_velocity*60/6.283185;
  String f = String(euler.x(), 4) + "," + String(euler.y(), 4) + "," + String(euler.z(), 4) + "," + String(accel.x(), 4) + "," + String(accel.y(), 4) + "," + String(accel.z(), 4) + "," + String(gravity.x(), 4) + "," + String(gravity.y(), 4) + "," + String(gravity.z(), 4);
  return f;
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

  if (STATE == CALIBRATION_STATE) {
    String stat = displayCalStatus();
    gestureCharacteristic.writeValue(stat);
    Serial.println(stat);
    if (stat == "3,3,3,3") {
      if (calibration > 10){
        STATE = RUNNING_STATE;
      }
      
      calibration++;
    }
  } else if (STATE == RUNNING_STATE) {
    String toSend = sendData();
    gestureCharacteristic.writeValue(toSend);
    Serial.println(toSend);
  }
  /* Get a new sensor event */ 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  // Serial.print("X: ");
  // Serial.print(event.orientation.x, 4);
  // Serial.print("\tY: ");
  // Serial.print(event.orientation.y, 4);
  // Serial.print("\tZ: ");
  // Serial.print(event.orientation.z, 4);
  // Serial.println("");

  
  delay(10);
}
