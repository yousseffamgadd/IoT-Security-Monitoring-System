/*
AlarmInterrupt.ino
Jacob Nuernberg
08/22

Example on using interrupts with DS3231 alarms.

Hardware setup:
  Connect DS3231 SQW pin to Arduino interrupt pin 2

Tested on:
- Arduino UNO
- Arduino nano

Added to this example:

1. Descriptively named variables to pass parameter values.

2. Modify AlarmBits for Alarm 1 to 0b00001111,
   for clarity, because Alarm 1 uses only bits 3:0.
   
3. Add code to prevent Alarm 2 from interfering with the interrupt,
   by setting A2Minute to a value that can never match the time
   and setting AlarmBits to 0b01100000: alarm "when minutes match".
   Also clear the A2 alarm flag.

David Sparks, September 2022
*/
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 2Kb
#include <TinyGsmClient.h>
#include "PubSubClient_Edited.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <DS3231.h>
#include "Adafruit_SHT31.h"
LSM6DS3 myIMU(I2C_MODE, 0x6A); // IMU 
#define int2Pin PIN_LSM6DS3TR_C_INT1

// myRTC interrupt pin
#define CLINT D8
#define SHT_SIM D2
#define water D0
#define door D1
Adafruit_SHT31 sht30 = Adafruit_SHT31(); // Create an SHT30 object
// Setup clock
DS3231 myRTC;

// Variables for use in method parameter lists
byte alarmDay;
byte alarmHour;
byte alarmMinute;
byte alarmSecond;
byte alarmBits;
bool alarmDayIsDay;
bool alarmH12;
bool alarmPM;
bool door_detected=false;
bool prev_door_detected=false;
bool water_detected=false;
bool prev_water_detected=false;
bool vibration_detected=false;
bool prev_vibration_detected=false;
bool wakeupinterrupt=false;
bool timerinterrupt=false;
unsigned long waking_timer=0;
unsigned long sensing_time=0;
unsigned long vibration_timer=0;
bool modeminitializedd=false;
float humidity=0;
float temperature=0;
int vibration_count=0;
const char apn[]  = ""; //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";
String IMEI ;
// #define SerialAT  GSM_AT

// #ifdef DUMP_AT_COMMANDS
//   #include <StreamDebugger.h>
//   StreamDebugger debugger(SerialAT, Serial);
//   TinyGsm modem(debugger);
// #else
//   TinyGsm modem(SerialAT);
// #endif
TinyGsm modem(Serial1);
TinyGsmClient client4G(modem);
PubSubClient client4G_mqtt(client4G);


// Interrupt signaling byte
String mqtt_server = "app.sstm-eg.com";
const int mqtt_port = 1883;
String Topic_to_publish="v1/devices/me/telemetry";
String client_id = "nrftest_123";     // Client ID
String mqtt_username = "nrftest_123"; // MQTT username (access token in ThingsBoard)
String mqtt_password = "nrftest_123"; // MQTT password (can be empty or same token)


uint8_t interruptCount = 0; // Amount of received interrupts
uint8_t prevInterruptCount = 0; // Interrupt Counter from last loop
uint16_t detectCount = 0;

void setup() {

    // Begin I2C communication
    Wire.begin();

    Serial.begin(115200);   // USB Serial for debugging
    Serial1.begin(115200);  // Hardware Serial (D6 TX, D7 RX)
    Serial.println("Starting Serial");
    digitalWrite(SHT_SIM, LOW); // Set D2 high
   
    // Assign parameter values for Alarm 1
    alarmDay = 0;
    alarmHour = 0;
    alarmMinute = 0;
    alarmSecond = 0;
    // alarmBits = 0b00001111; // Alarm 1 every second
    // alarmBits = 0b00000110; // Mask seconds, match minute
    alarmBits = 0b00001100; // Alarm 1 hour
    alarmDayIsDay = false;
    alarmH12 = false;
    alarmPM = false;    
    // Set alarm 1 to fire at one-second intervals
    myRTC.turnOffAlarm(1);
    myRTC.setA1Time(
       alarmDay, alarmHour, alarmMinute, alarmSecond,
       alarmBits, alarmDayIsDay, alarmH12, alarmPM);
    // enable Alarm 1 interrupts
    myRTC.turnOnAlarm(1);
    // clear Alarm 1 flag
    myRTC.checkIfAlarm(1);

    alarmMinute = 0xFF; // a value that will never match the time
    alarmBits = 0b01100000; // Alarm 2 when minutes match, i.e., never
    
    // Upload the parameters to prevent Alarm 2 entirely
    myRTC.setA2Time(
        alarmDay, alarmHour, alarmMinute,
        alarmBits, alarmDayIsDay, alarmH12, alarmPM);
    // disable Alarm 2 interrupt
    myRTC.turnOffAlarm(2);
    // clear Alarm 2 flag
    myRTC.checkIfAlarm(2);

    // NOTE: both of the alarm flags must be clear
    // to enable output of a FALLING interrupt

    // attach clock interrupt

     
    
     myIMU.settings.gyroEnabled = 0; // Gyro currently not used, disabled to save power 
    if (myIMU.begin() != 0) {
        Serial.println("IMU error");
    } else {
        Serial.println("IMU OK!");
    }
    setupDoubleTapInterrupt();
    // setupFreeFallInterrupt();
    pinMode(int2Pin, INPUT);
    pinMode(SHT_SIM, OUTPUT);
    pinMode(CLINT, INPUT_PULLUP);
    pinMode(D1, INPUT_PULLUP);
    pinMode(D0, INPUT_PULLDOWN);  
    digitalWrite(SHT_SIM, LOW); // Set D2 high

    attachInterrupt(digitalPinToInterrupt(CLINT), timer_found, FALLING);
    // attachInterrupt(digitalPinToInterrupt(D0), door_found, RISING);
    // attachInterrupt(digitalPinToInterrupt(D1), water_found, RISING);
    attachInterrupt(digitalPinToInterrupt(int2Pin), vibration_found, RISING);
    // delay(15000);
    String wakePin = checkWakeupPin();
    Serial.println("Returned wake-up pin: " + wakePin);

    if (wakePin.equals("P0.11")) {
        wakeupinterrupt = true;
    }else if(wakePin.equals("P1.13")) {
        timerinterrupt = true;
    }

    
    Serial.println("Wake-up interrupt flag: " + String(wakeupinterrupt));
    if(!wakeupinterrupt){
      // digitalWrite(SHT_SIM, HIGH); // Set D2 high

    initmodem();
    publishData("Sleep mode","Woke up"); 
    if(!digitalRead(D1)){
      publishData("Cover","Closed");
    }
    if(!digitalRead(D0)){
      publishData("Water","NOT Detected");
    }  
    publishData("Vibration","NOT Detected");
    }


}

void loop() {
  if(!wakeupinterrupt){
    if(!modeminitializedd){
      initmodem();
    publishData("Sleep mode","Woke up"); 
    if(!digitalRead(D1)){
      publishData("Cover","Closed");
    }
    if(!digitalRead(D0)){
      publishData("Water","NOT Detected");
    }  
    publishData("Vibration","NOT Detected");
    
    }
  if(!client4G_mqtt.connected()){
    connectToMQTT();
  }
    check_vibration();
    check_cover_water();
    if(millis()-sensing_time>=20000){
      sensing_time=millis();
      measure_temp();
    }
    
    if(millis()-waking_timer>=30000){
      if(!digitalRead(D0) && !digitalRead(D1) && !vibration_detected){ 
      publishData("Sleep mode","Sleeping");     
      goToPowerOff();
      }
    }
      
    // client4G_mqtt.loop();
    

    
    
  }
  else{
    if(digitalRead(D0) || digitalRead(D1) || vibration_detected){
      wakeupinterrupt=false;
    }
    if(millis()-waking_timer>=30000){
      if(!digitalRead(D0) && !digitalRead(D1) && !vibration_detected){
          goToPowerOff();
      }

  }
}
}


// void setupFreeFallInterrupt() {
//   uint8_t errorAccumulator = 0;
//   uint8_t dataToWrite = 0;  //Temporary variable

//   //Setup the accelerometer******************************
//   dataToWrite = 0; //Start Fresh!
//   dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
//   dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
//   dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;

//    // //Now, write the patched together data
//    errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60 );
//    //errorAccumulator += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
//    //errorAccumulator += myIMU.writeRegister( LSM6DS3_ACC_GYRO_CTRL4_C, 0x10 );
//    // Write 00h into WAKE_UP_DUR 
// 	 errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00 );
//     // Set FF threshold (FF_THS[2:0] = 011b)
//     // Set six samples event duration (FF_DUR[5:0] = 000110b)
//     // Write 33h into FREE_FALL 
//    errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_FREE_FALL, 0x33);
//     // FF interrupt driven to INT1 pin
//     // Write 10h into MD1_CFG
// 	 errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x10 );
//     // Also route to INT2 pin
//     // Write 10h into MD1_CFG
// 	 errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD2_CFG, 0x10 );
// 	  // Latch interrupt
//     // Write 01h into TAP_CFG 
//    errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x81);
	
// 	if( errorAccumulator )
//   {
// 	  Serial.println("Problem configuring the device.");
//   }
//   else
//   {
// 	  Serial.println("Device O.K.");
//   }	
// }
void check_vibration(){
if(vibration_detected){
 
  if(vibration_detected!=prev_vibration_detected){
    prev_vibration_detected=vibration_detected;
     vibration_count=0;
    // sendPostRequest("Vibration","Detected");
    publishData("Vibration","Detected");
  }
  if(millis()-vibration_timer>=10000){
    vibration_detected=false;
    if(vibration_detected!=prev_vibration_detected){
    prev_vibration_detected=vibration_detected;
     vibration_count=0;
    // sendPostRequest("Vibration","NOT Detected");
    publishData("Vibration","NOT Detected");
  }
  }
}

}
void check_cover_water(){
      
    if(digitalRead(D1)){
      door_detected=true;
      if(door_detected!=prev_door_detected){
        prev_door_detected=door_detected;
        // sendPostRequest("Cover","Opened");
        publishData("Cover","Opened");
      }
    }else{
      door_detected=false;
      if(door_detected!=prev_door_detected){
        prev_door_detected=door_detected;
        // sendPostRequest("Cover","Closed");
        publishData("Cover","Closed");
      }
    }
    if(digitalRead(D0)){
      water_detected=true;
      if(water_detected!=prev_water_detected){
        prev_water_detected=water_detected;
        waking_timer=millis();
        // sendPostRequest("Water Status","WET");
        publishData("Water","Detected");
      }
    }else{
      water_detected=false;
      if(water_detected!=prev_water_detected){
        prev_water_detected=water_detected;
        waking_timer=millis();
        // sendPostRequest("Water Status","DRY");
        publishData("Water","NOT Detected");
      }
    }    
}
void initmodem(){
  delay(500);
    digitalWrite(SHT_SIM, HIGH); // Set D2 high
    delay(5000);
Serial.println(F("Initializing modem..."));
delay(5000);
  modem.init();
  
  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);
  IMEI = modem.getIMEI();
  
  Serial.println("IMEI : "+IMEI); 


  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(F("Network connection failed"));
    
  }
  
  if (modem.isNetworkConnected()) {
    Serial.println(F("Network connected"));
  } else {
    Serial.println("Network not connected");
  }
  int i=0;
      if(!modem.isGprsConnected()){
      while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          // Wait for 5 seconds before retrying
        Serial.println("Failed to connect to network, retrying in 2 seconds...");
        delay(2000);
        i++;
        if (i>5){
          // sendPostRequest_wifi(key,value);
          return;
          break;
        }
      }
    }
      
    
  client4G_mqtt.setServer(mqtt_server.c_str(), mqtt_port);  // Configure MQTT server
  connectToMQTT();

}


void setupDoubleTapInterrupt() {
  uint8_t error = 0;
  uint8_t dataToWrite = 0;

  // Double Tap Config
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60); //* Acc = 416Hz (High-Performance mode)// Turn on the accelerometer
  // ODR_XL = 416 Hz, FS_XL = 2g
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);// INTERRUPTS_ENABLE, SLOPE_FDS// Enable interrupts and tap detection on X, Y, Z-axis
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x8C);// Set tap threshold 8C
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);// Set Duration, Quiet and Shock time windows 7F
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);// Single & double-tap enabled (SINGLE_DOUBLE_TAP = 1)
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);// Double-tap interrupt driven to INT1 pin
}



void vibration_found() {
  vibration_count++;
  Serial.println(vibration_count);
  if(vibration_count>=5){
    vibration_detected=true;
    waking_timer=millis();
    vibration_timer=millis();
}
}
void timer_found() {
    waking_timer=millis();
}


void goToPowerOff() {
  // setLedRGB(false, false, false);
  digitalWrite(SHT_SIM, LOW); // Set D2 high
//   Serial.println("Going to System OFF after 10 sec");
//  delay(10000);
  NRF_P1->PIN_CNF[13] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos) |  // Wake-up on LOW
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |  // Enable pull-up
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Connect input
                          (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);  // Set as input

  NRF_P0->PIN_CNF[3] = (GPIO_PIN_CNF_SENSE_High  << GPIO_PIN_CNF_SENSE_Pos) |  // Wake-up on LOW
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |  // Enable pull-up
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Connect input
                          (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);  // Set as input   configure_gpio_wakeup();

  NRF_P0->PIN_CNF[2] = (GPIO_PIN_CNF_SENSE_High  << GPIO_PIN_CNF_SENSE_Pos) |  // Wake-up on LOW
                          (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos) |  // Enable pull-up
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Connect input
                          (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);  // Set as input   configure_gpio_wakeup();  
                          
  NRF_P0->PIN_CNF[11] = (GPIO_PIN_CNF_SENSE_High  << GPIO_PIN_CNF_SENSE_Pos) |  // Wake-up on LOW
                          (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos) |  // Enable pull-up
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Connect input
                          (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);  // Set as input   configure_gpio_wakeup(); 
  // nrf_gpio_cfg_sense_input(digitalPinToInterrupt(int2Pin), NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);                                                 
    // Clear wake-up events
    NRF_POWER->RESETREAS = 0xFFFFFFFF;  // Clear reset reason to detect wake-up source later

    // Enter deep sleep (System OFF)

  NRF_POWER->SYSTEMOFF = 1;
}


void measure_temp(){
     if (!sht30.begin(0x44)) {
        Serial.println("Failed to find SHT30 sensor!");
    } else {
         temperature = sht30.readTemperature();
         humidity = sht30.readHumidity();

        if (!isnan(temperature) && !isnan(humidity)) {
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.print(" °C, Humidity: ");
            Serial.print(humidity);
            Serial.println(" %");
            // sendPostRequest("Temperature",String(temperature));
            // sendPostRequest("Humidity",String(humidity));
            publishData("Temperature",String(temperature));
            publishData("Humidity",String(humidity));
        } else {
            Serial.println("Failed to read SHT30 sensor data.");
        }
}

}


void sendPostRequest(String key, String value) {
    Serial.println("\""+key+"\",\""+value+"\"");
    // Check if the modem is connected
  int i=0;
  int signalQuality = modem.getSignalQuality();
  if(signalQuality<=31 && signalQuality>10){
    
    if(!modem.isGprsConnected()){
      while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          // Wait for 5 seconds before retrying
        Serial.println("Failed to connect to network, retrying in 2 seconds...");
        delay(2000);
        i++;
        if (i>5){
          // sendPostRequest_wifi(key,value);
          return;
          break;
        }
      }
    }
    // else{
    //   sendPostRequest_wifi(key,value);
    // }
  }else{
    // sendPostRequest_wifi(key,value);
    return;
  }

    
    // The ThingsBoard API URL to post attributes
    const char* serverName = "demo.thingsboard.io";
    String url = "/api/v1/nd1oln2mqaujvrix33sy/telemetry";

    
    // Create the JSON payload
    String jsonPayload = "{\""+key+"\": \"" + value + "\"}";

    // Connect to the server
    if (client4G.connect(serverName, 80)) {  // Connect to port 80 (HTTP)
      Serial.println("Connected to server");

      // Make the HTTP POST request manually
      client4G.print(String("POST ") + url + " HTTP/1.1\r\n");
      client4G.print(String("Host: ") + serverName + "\r\n");
      client4G.print("Content-Type: application/json\r\n");
      client4G.print("Content-Length: " + String(jsonPayload.length()) + "\r\n");
      client4G.print("Connection: close\r\n\r\n");
      client4G.print(jsonPayload);  // Send the JSON payload

      // Wait for the server to respond
      while (client4G.connected() || client4G.available()) {
        if (client4G.available()) {
          String line = client4G.readStringUntil('\n');
          Serial.println(line);  // Print the server response line by line
        }
      }

      // Close the connection
      client4G.stop();
      Serial.println("Connection closed");
    } else {      
      Serial.println("Failed to connect to server");
      // sendPostRequest_wifi(key,value);
     }
    delay(5000);
}




void connectToMQTT() {
  int i=0;
  int signalQuality = modem.getSignalQuality();
  if(signalQuality<=31 && signalQuality>10){
      if(!modem.isGprsConnected()){
      while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          // Wait for 5 seconds before retrying
        Serial.println("Failed to connect to network, retrying in 2 seconds...");
        delay(2000);
        i++;
        if (i>5){
          // sendPostRequest_wifi(key,value);
          return;
          break;
        }
      }
    }
  }
  while (!client4G_mqtt.connected()) {
    Serial.print("Connecting to MQTT broker... ");
    if (client4G_mqtt.connect(client_id.c_str(), mqtt_username.c_str(), mqtt_password.c_str())) {
      Serial.println("Connected!");
      modeminitializedd=true;
    } else {
      Serial.print("Failed. Retrying in 5 seconds, state: ");
      Serial.println(client4G_mqtt.state());
      delay(5000);
    }
  }

}
void publishData(String Key1,String value1) {

  String payload = "{\""+Key1+"\":\""+value1+"\"}";
  
  if (client4G_mqtt.publish(Topic_to_publish.c_str(), payload.c_str())) {
    Serial.println("Published: "+ payload);
  } else {
    Serial.println("Publish failed!");
  }
  delay(1000);
}

String checkWakeupPin() {
  uint32_t latchedPinsP0 = NRF_P0->LATCH;
  uint32_t latchedPinsP1 = NRF_P1->LATCH;

  String wakeupPin = "";

  // Check Port 0 pins
  for (int i = 0; i < 32; i++) {
    if (latchedPinsP0 & (1 << i)) {
      wakeupPin = "P0." + String(i);
      break;
    }
  }

  // Check Port 1 pins if none found in Port 0
  if (wakeupPin == "") {
    for (int i = 0; i < 16; i++) {  // P1 has up to 16 pins (P1.00 - P1.15)
      if (latchedPinsP1 & (1 << i)) {
        wakeupPin = "P1." + String(i);
        break;
      }
    }
  }

  // Clear the LATCH registers
  NRF_P0->LATCH = latchedPinsP0;
  NRF_P1->LATCH = latchedPinsP1;

  return wakeupPin;
}


