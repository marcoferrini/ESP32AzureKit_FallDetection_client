# ESP32AzureKit_FallDetection_client

Fall detection system based on ESP32 Azure IoT kit. 
This repository is the esp-idf project forlder used to develop the client of the fall detection system

## Source code
The main folder contains the sorce code.
  
-`main.c` contains main task, the BLE client gatt and gap cb and also the user-defined button callback 
-`sensors.c` constains the display task and i2c config functions
-`button.c` API for user-defined button
-`ssd1306.c` API for esp32 azure iot kit display ssd1306
-`fonts.c` structures definition for ssd1306


## Author 
Marco Ferrini


 
