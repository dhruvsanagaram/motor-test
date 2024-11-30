// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEClient.h>
// #include <BLE2902.h>

// #define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
// #define CHARACTERISTIC_UUID "87654321-4321-4321-4321-0987654321ba"

// BLEClient *pClient;
// BLERemoteCharacteristic *pRemoteCharacteristic;
// BLEAdvertisedDevice *myDevice;
// bool deviceFound = false;

// void notifyCallback(BLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
//     String distance = String((char *)pData);
//     Serial.println("Received Distance: " + distance);
// }

// class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
//     void onResult(BLEAdvertisedDevice advertisedDevice) {
//         Serial.print("Found device: ");
//         Serial.println(advertisedDevice.toString().c_str());

//         // Check if the device name matches
//         if (advertisedDevice.getName() == "ESP32_UWB_Anchor") {
//             deviceFound = true;
//             myDevice = new BLEAdvertisedDevice(advertisedDevice); // Save the device
//             advertisedDevice.getScan()->stop(); // Stop scanning
//         }
//     }
// };

// BLEDevice::init("");
  // BLEScan *pBLEScan = BLEDevice::getScan();
  // pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  // pBLEScan->setActiveScan(true);
  // pBLEScan->start(10); // Scan for 10 seconds

  // while (!deviceFound) {
  //     delay(100); // Wait for the target device to be found
  // }
  // Serial.println("Connecting to BLE server...");
  // pClient = BLEDevice::createClient();
  // pClient->connect(myDevice); // Replace with server's name or MAC
  // Serial.println("Connected to BLE server.");
  // BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  // if (pRemoteService == nullptr) {
  //     Serial.println("Failed to find service.");
  //     return;
  // }
  // pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  // if (pRemoteCharacteristic == nullptr) {
  //     Serial.println("Failed to find characteristic.");
  //     return;
  // }
  // if (pRemoteCharacteristic->canNotify()) {
  //     pRemoteCharacteristic->registerForNotify(notifyCallback);
  // }