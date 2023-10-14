#include <ArduinoBLE.h>

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

String start = "0,0,0";
int x = 0;
int y = 10;
int z = 20;

BLEService gestureService(deviceServiceUuid); 
BLEStringCharacteristic gestureCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLENotify, 20);



void setup() {
  Serial.begin(9600);
  while (!Serial);  
  
  if (!BLE.begin()) {
    Serial.println("- Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  BLE.setLocalName("Seeed");
  BLE.setDeviceName("Seeed");
  BLE.setAdvertisedService(gestureService);
  gestureService.addCharacteristic(gestureCharacteristic);
  BLE.addService(gestureService);
  gestureCharacteristic.writeValue(start);
  BLE.advertise();

  Serial.println("Nano 33 BLE (Peripheral Device)");
  Serial.println(" ");
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
  }
  while (central.connected()) {
    x++;
    y++;
    z++;
    gestureCharacteristic.writeValue(String(x)+","+String(y)+","+String(z));
    delay(10);
  }

}
