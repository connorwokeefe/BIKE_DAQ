/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include "SD.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_ADS1015.h>

// ADS Setup
Adafruit_ADS1115 ads(0x48);

const int potPin = 34;
int16_t adc0;
float v_ADC = 0.0;

// Define LED pins
#define PIN_GREEN2    13 // GPIO23
#define PIN_RED2  12 // GPIO22
#define PIN_BLUE2  14 // GPIO21

#define PIN_RED1    25 // GPIO23
#define PIN_GREEN1  26 // GPIO22
#define PIN_BLUE1   27 // GPIO21

// GPS Setup
#define GPS_RX 16  // Connect GPS module TX to this pin
#define GPS_TX 17  // Connect GPS module RX to this pin
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

TinyGPSPlus gps;

float gpsHr, gpsMin, gpsSec, gpsLat, gpsLong, gpsSpeed, gpsSats;

// Board ID Number
//////////////////////////////
// CHANGE THIS WITH EACH BOARD!!
//////////////////////////////
int BOARD_ID = 4;

// Define MPU6050
Adafruit_MPU6050 mpu;

// Time variable
unsigned long t = 0;

// FLOATS TO STORE OFFSET VALUES FROM MPU BEFORE STORAGE
float aX, aY, aZ, gX, gY, gZ;

// FLOATS TO STORE MPU CALIBRATION VALUES
float aXSum = 0.0;
float aYSum = 0.0;
float aZSum = 0.0;

float avgAX = 0.0;
float avgAY = 0.0;
float avgAZ = 0.0;

// MPU ACCELEROMETER + GRYO RANGE & FILTER BANDWIDTH
#define mpuAccelRange MPU6050_RANGE_8_G         // 2, 4, 8, or 16 [G]
#define mpuGyroRange MPU6050_RANGE_500_DEG      // 250, 500, 1000, or 2000 [deg/s]
#define mpuFilterBandwidth MPU6050_BAND_94_HZ   // 5, 10, 21, 44, 94, 184, or 260 [Hz]
String accelRange, gyroRange, filterBandwidth;

// CALIBRATION ITERATIONS
int iCal = 1000;

// Define SD card array
char dOut[100];         // String used to print data

// FILE PATHS
String calFilePath = "/calDataMTB_1.csv";
String dataFilePath = "/testDataMTB_1.csv";
File calData;
File testData;

// MMOTHER BOARD GROUP1
uint8_t broadcastAddress0[] = {0x94, 0xB5, 0x55, 0xFD, 0x5D, 0x5C}; // Board 0


// MOTHER BOARD GROUP 2
// uint8_t broadcastAddress9[] = {0x94, 0xB5, 0x55, 0xFD, 0x5D, 0x5C}; // Board 9


// Variable to store if sending data was successful
String success;

// Variable for communication checks
bool isReady = false;
bool isReadySent = false;
bool isCalibrationReady = false;
bool isWaitingCalibrationSent = false;
bool isGoing = false;
bool isGoingSent = false;
bool enoughSats = false;

int attempts = 0;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
  int id;
  String talk;
} struct_message;

struct_message heard;

struct_message outgoingTalk;

esp_now_peer_info_t peerInfo;


/* WAIT FUNCTION */
inline void wait(unsigned long const& t) {
   unsigned long sTime = millis();
   while (millis() - sTime <= t);
}

/* REBOOT FUNCTION */
void reboot(){
  glow(255,0,0,1);
  glow(0,0,0,2);
  wait(1000);
  glow(0,0,0,1);
  Serial.println("Restarting in 5...");
  wait(1000);
  Serial.println("4...");
  glow(255,0,0,1);
  wait(1000);
  Serial.println("3...");
  glow(0,0,0,1);
  wait(1000);
  Serial.println("2...");
  glow(255,0,0,1);
  wait(1000);
  Serial.println("1...");
  glow(0,0,0,1);
  wait(1000);
  ESP.restart();
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&heard, incomingData, sizeof(heard));
  if(heard.talk == "r"){
    Serial.println(heard.talk);
    isReady = true;
  }
  if(isReady && heard.talk == "c"){
    isCalibrationReady = true;
  }
  if(isReady && isCalibrationReady && heard.talk == "g"){
    isGoing = true;
  }
  if(heard.talk == "reboot" + String(outgoingTalk.id) || heard.talk == "rebootall"){
    outgoingTalk.talk = "B" + String(BOARD_ID) + " - Rebooting now...";
    outgoingTalk.id = BOARD_ID;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress9, (uint8_t *) &outgoingTalk, sizeof(outgoingTalk));
    if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
    else {
      Serial.println("Error sending the data");
    }
    reboot();
  }
  if(heard.talk == "sleep"){
    outgoingTalk.talk = "B" + String(BOARD_ID) + " - Sleeping now...";
    outgoingTalk.id = BOARD_ID;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress9, (uint8_t *) &outgoingTalk, sizeof(outgoingTalk));
    if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
    else {
      Serial.println("Error sending the data");
    }
  }
  if(heard.talk == "save"){
    testData.close();
    testData = SD.open(dataFilePath, FILE_APPEND);
    sendMessage("B" + String(BOARD_ID) + " - Saved", BOARD_ID);
  }
  if(heard.talk == "send" + String(outgoingTalk.id)){
    testData.close();
    wait(100);
    testData = SD.open(dataFilePath);
    while(testData.available()){
      sendMessage(String(testData.read()), BOARD_ID);
    }
    testData.close();
    testData = SD.open(dataFilePath, FILE_APPEND);
    
  }
}



/* REG LED FUNCTION*/
void glow(int red, int green, int blue, int led){
  if(led == 1){
    analogWrite(PIN_RED1,   red);
    analogWrite(PIN_GREEN1, green);
    analogWrite(PIN_BLUE1, blue);
  }
  else if(led == 2){
    analogWrite(PIN_RED2,   red);
    analogWrite(PIN_GREEN2, green);
    analogWrite(PIN_BLUE2,  blue);
  } 
}

/* START TALK BT FUNCTION*/
void beginTalk() {
  outgoingTalk.id = BOARD_ID;
  outgoingTalk.talk = "B" + String(BOARD_ID) + " - Ready?"; // Send board number and request start permission. (DOES NOT START DAQ)
  if(!isReadySent){
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress9, (uint8_t *) &outgoingTalk, sizeof(outgoingTalk));
    if (result == ESP_OK) {
        Serial.println("Sent with success");
        isReadySent = true;
      }
    else {
      Serial.println("Error sending the data");
    }
  }
  glow(255, 140, 0, 1);
  wait(3000);
}

void calibrationRequest() {
  if(!isWaitingCalibrationSent){
    sendMessage("B" + String(BOARD_ID) + " - Cal?", BOARD_ID); // Send board number and request calibration permission.
    isWaitingCalibrationSent = true;
    
  }
  glow(255, 140, 0, 1);
  glow(0, 0, 0, 2);
  wait(3000);
}

void goRequest() {
  if(!isGoingSent){
    sendMessage("B" + String(BOARD_ID) + " - GO?", BOARD_ID); // Send board number and request go permission. (DOES START DAQ)
    isGoingSent = true;
  }
  glow(255, 140, 0, 1);
  glow(0, 0, 0, 2);
  wait(3000);
}


// FUNCTION TO SEND MESSAGE VIA BT
void sendMessage(String message, int id){
  outgoingTalk.talk = message;
  outgoingTalk.id = id;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress9, (uint8_t *) &outgoingTalk, sizeof(outgoingTalk));
  if (result == ESP_OK) {
      Serial.println("Sent with success");
      outgoingTalk.talk = "";
    }
  else {
    Serial.println("Error sending the data");
  }
}

// FUNCTION TO WAIT FOR SATS GREATER THAN 3 TO ENSURE ENOUGH GPS DATA CAN BE RECEIVED
void satWait(){
  while (gpsSerial.available() > 0){
    if (gps.encode(gpsSerial.read())){
      if(gps.satellites.value() > 3){
        enoughSats = true;
      }
    }
  }
}

/* CONNECT TO MPU */
void mpuConnect(){
   if (!mpu.begin()) {
      //Serial.println("Failed to find MPU6050 chip!");
      sendMessage(String(BOARD_ID) + " - Failed to find MPU6050 chip!", BOARD_ID);
      glow(255,0,0,1);
      reboot();
   } else{
      glow(0,255,0,1);
      wait(3000);
      //Serial.println("Connected to MPU6050 :D");
      glow(0,0,0,1);
   }
}

/* CONNECT TO SD CARD */
void initSDCard(){
      if (!SD.begin()) {
      //Serial.println("Card mount failed... restarting!");
      glow(255,0,0,2);
      sendMessage(String(BOARD_ID) + " - Card mount failed... restarting!", BOARD_ID);
      reboot();
      return;
   }
   uint8_t cardType = SD.cardType();

   if(cardType == CARD_NONE){
      //Serial.println("No SD card attached... restarting!");
      glow(255,0,0,2);
      sendMessage(String(BOARD_ID) + " - No SD card attached... restarting!", BOARD_ID);
      reboot();
      return;
   }
//   Serial.print("SD Card Type: ");
//   if(cardType == CARD_MMC){
//      Serial.println("MMC");
//   } else if(cardType == CARD_SD){
//      Serial.println("SDSC");
//   } else if(cardType == CARD_SDHC){
//      Serial.println("SDHC");
//   } else {
//      Serial.println("UNKNOWN");
//   }
   uint64_t cardSize = SD.cardSize() / (1024 * 1024);
//   Serial.printf("SD Card Size: %lluMB\n", cardSize);

   bool fileLooping = true;
   int fileConstant = 1;

   /* WRITE CALIBRATION FILE TO SD CARD */
   while(fileLooping = true){
      if (!SD.exists(calFilePath)) {                    // CHECK IF CSV FILE ALREADY EXISTS OR NOT
         calData = SD.open(calFilePath, FILE_WRITE);  // WRITE CSV FILE TO SD CARD
         wait(1000);
         if (!calData) {
            Serial.println("!!!!!! FAILED TO WRITE " + calFilePath + " TO SD CARD !!!!!");
            calData.close();  //CLOSE FILE
            return;
         } else {
            Serial.println(calFilePath + " SUCCESSFULLY WRITTEN TO SD CARD.");
            calData.close();  //CLOSE FILE
            fileLooping = false;
            break;
         }
      } else {
         Serial.println(calFilePath + " ALREADY EXISTS ON SD CARD. TRYING AGAIN WITH DIFFERENT FILE NAME.");
         fileConstant++;      
         calFilePath = "/calDataMTB_" + String(fileConstant) + ".csv";         // Try again with different file name until file name is unique on SD card
      }
   }
   sendMessage("B" + String(BOARD_ID) + ":C" + String(fileConstant), BOARD_ID); // SEND BOARD NUMBER AND CALIBRATION FILE NUMBER
   
   fileLooping = true;
   fileConstant = 1;

   /* WRITE DATA FILE TO SD CARD */
   while(fileLooping = true){
      if (!SD.exists(dataFilePath)) {                    // CHECK IF CSV FILE ALREADY EXISTS OR NOT
         testData = SD.open(dataFilePath, FILE_WRITE);  // WRITE CSV FILE TO SD CARD
         wait(1000);
         if (!testData) {
            Serial.println("!!!!!! FAILED TO WRITE " + dataFilePath + " TO SD CARD !!!!!");
            testData.close();  //CLOSE FILE
            return;
         } else {
            Serial.println(dataFilePath + " SUCCESSFULLY WRITTEN TO SD CARD.");
            testData.close();  //CLOSE FILE
            fileLooping = false;
            break;
         }
      } else {
         Serial.println(dataFilePath + " ALREADY EXISTS ON SD CARD. TRYING AGAIN WITH DIFFERENT FILE NAME.");
         fileConstant++;      
         dataFilePath = "/testDataMTB_" + String(fileConstant) + ".csv";         // Try again with different file name until file name is unique on SD card
      }
   }
  sendMessage("B" + String(BOARD_ID) + ":T" + String(fileConstant), BOARD_ID); // SEND BOARD NUMBER AND TEST FILE NUMBER
}

/* MPU SETUP */
void mpuSetup(){
   mpu.setAccelerometerRange(mpuAccelRange);
   //Serial.print("MPU accelerometer range set to: ");
   switch (mpu.getAccelerometerRange()) {
   case MPU6050_RANGE_2_G:
      accelRange = "+-2G";
      break;
   case MPU6050_RANGE_4_G:
      accelRange = "+-4G";
      break;
   case MPU6050_RANGE_8_G:
      accelRange = "+-8G";
      break;
   case MPU6050_RANGE_16_G:
      accelRange = "+-16G";
      break;
   }
   //Serial.println(accelRange);
   mpu.setGyroRange(mpuGyroRange);
   //Serial.print("MPU gyro range set to: ");
   switch (mpu.getGyroRange()) {
   case MPU6050_RANGE_250_DEG:
      gyroRange = "+- 250 deg/s";
      break;
   case MPU6050_RANGE_500_DEG:
      gyroRange = "+- 500 deg/s";
      break;
   case MPU6050_RANGE_1000_DEG:
      gyroRange = "+- 1000 deg/s";
      break;
   case MPU6050_RANGE_2000_DEG:
      gyroRange = "+- 2000 deg/s";
      break;
   }
   //Serial.println(gyroRange);
   mpu.setFilterBandwidth(mpuFilterBandwidth);
   //Serial.print("MPU filter bandwidth set to: ");
   switch (mpu.getFilterBandwidth()) {
   case MPU6050_BAND_260_HZ:
      filterBandwidth = "260 Hz";
      break;
   case MPU6050_BAND_184_HZ:
      filterBandwidth = "184 Hz";
      break;
   case MPU6050_BAND_94_HZ:
      filterBandwidth = "94 Hz";
      break;
   case MPU6050_BAND_44_HZ:
      filterBandwidth = "44 Hz";
      break;
   case MPU6050_BAND_21_HZ:
      filterBandwidth = "21 Hz";
      break;
   case MPU6050_BAND_10_HZ:
      filterBandwidth = "10 Hz";
      break;
   case MPU6050_BAND_5_HZ:
      filterBandwidth = "5 Hz";
      break;
   }
   //Serial.println(filterBandwidth);
}

/* CALIBRATE MPU ON BIKE */
void mpuCalibration(){
 glow(0,0,0,2);
 aXSum = 0.0;
 aYSum = 0.0;
 aZSum = 0.0;
 
 float gXSum = 0.0;
 float gYSum = 0.0;
 float gZSum = 0.0;
 iCal = iCal*3;
 for(int i=0;i<iCal;i++){
  if(i%10 == 0){
    glow(0,0,255,1);
  }
  else{
    glow(0,0,0,1);
  }
  //Serial.println("Calibrating, hold still...");
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);    
  gXSum += float(g.gyro.x);
  gYSum += float(g.gyro.y);
  gZSum += float(g.gyro.z);   
  aXSum += float(a.acceleration.x);
  aYSum += float(a.acceleration.y);
  aZSum += float(a.acceleration.z);
  wait(10);                        
 }
 float gAX = gXSum/iCal;    
 float gAY = gYSum/iCal;
 float gAZ = gZSum/iCal;
 avgAX = aXSum/iCal;
 avgAY = aYSum/iCal;
 avgAZ = aZSum/iCal;
 if (!SD.exists(calFilePath)) {
    Serial.println("!!!!! UNABLE TO OPEN " + calFilePath + " TO PRINT DATA POINT !!!!!");
 } else {
    calData = SD.open(calFilePath, FILE_APPEND); // OPEN THE FILE
    if (calData) {
       calData.println("ON-THE-BIKE GYRO AND GRAVITATIONAL ACCELERATION CALIBRATION");
       calData.println("X-GIRO,Y-GIRO,Z-GIRO");
       sprintf(dOut,"%f,%f,%f",gAX,gAY,gAZ);
       calData.println(dOut);
       calData.println("");
       calData.println("");
       Serial.println("X-GIRO,Y-GIRO,Z-GIRO");
       Serial.println(dOut);
       calData.println("X-ACCEL,Y-ACCEL,Z-ACCEL");
       sprintf(dOut,"%f,%f,%f",avgAX,avgAY,avgAZ);
       calData.println(dOut);
       calData.close();                             // CLOSE THE FILE
       Serial.println("X-ACCEL,Y-ACCEL,Z-ACCEL");
       Serial.println(dOut);
    } else {
       Serial.println("!!!!! ERROR PRINTING DATA TO " + calFilePath + " !!!!!");
    }
 }

 Serial.println("-------------------------------");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  gpsSerial.begin(9600);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();

  

  // Init LED pins
  pinMode(PIN_RED1,   OUTPUT);
  pinMode(PIN_GREEN1, OUTPUT);
  pinMode(PIN_BLUE1,  OUTPUT);
  pinMode(PIN_RED2,   OUTPUT);
  pinMode(PIN_GREEN2, OUTPUT);
  pinMode(PIN_BLUE2,  OUTPUT);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress9, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  delay(5000);

  mpuConnect();

  initSDCard();

  mpuSetup();

  while(!isReady){
    beginTalk();
  }
  sendMessage("1", BOARD_ID); // Begin communication with mother board
  
  glow(0, 0, 255, 1); // Turn on first LED to blue
  glow(0, 0, 255, 2); // Turn on second LED to blue
  
  if(!enoughSats){
    glow(0,0,255,1);
    glow(0,255,0,2);
    sendMessage("B" + String(BOARD_ID) + " - NES", BOARD_ID); // NOT ENOUGH SATS (NES)
   }
   while(!enoughSats){
    satWait();
    if(heard.talk == "g") goto skipSat; // SKIP SAT CHECK TO AVOID DELAYS TO TESTING
   }
   sendMessage("B" + String(BOARD_ID) + " - ES", BOARD_ID); // ENOUGH SATS (ES)
   skipSat:
   glow(0,0,255,1);
   glow(0,0,255,2);
  
  while(!isCalibrationReady){
    calibrationRequest(); // WAIT FOR CALIBRATION CONFIRMATION
  }
  mpuCalibration();
  sendMessage("c", BOARD_ID); // SEND CALIBRATION COMPLETE CODE

  glow(0,255,0,1);
  glow(0,255,0,2);
  
  if (!SD.exists(dataFilePath)) {
      //Serial.println("!!!!! UNABLE TO OPEN " + dataFilePath + " TO PRINT DATA POINT !!!!!");
      sendMessage("Can't open file", BOARD_ID);
   } else {
      testData = SD.open(dataFilePath, FILE_APPEND); // OPEN THE FILE
      if (testData) {
         sprintf(dOut,"THIS DATA CORRESPONDS TO CALIBRATION FILE %s",calFilePath);
         testData.println(dOut);
         testData.println("");
         testData.println("");
         // USE FOR GPS DATA
         //testData.println("Hours,Minutes,Seconds,Latitude,Longitude,Speed (kmph),Satelites,X-Accel,Y-Accel,Z-Accel,X-Gyro,Y-Gyro,Z-Gyro, Potentiometer (V)"); // CLOSE THE FILE

         // USE FOR ON CHIP TIME
         testData.println("millis(),X-Accel,Y-Accel,Z-Accel,X-Gyro,Y-Gyro,Z-Gyro");                             // CLOSE THE FILE
      } else {
         //Serial.println("!!!!! ERROR PRINTING DATA TO " + dataFilePath + " !!!!!");
      }
   }
   

   while(!isGoing){
    goRequest(); // WAIT FOR GO REQUEST
   }
   sendMessage("g", BOARD_ID);
   sendMessage("B" + String(BOARD_ID) + " - ON", BOARD_ID); // SEND MESSAGE THAT SAYS BOARD IS RUNNING
   glow(0,255,0,1);
   glow(0,255,0,2);
   t = millis();
}
 
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);     // Get the current acceleration & gyro data
  aX = float(a.acceleration.x);
  aY = float(a.acceleration.y);
  aZ = float(a.acceleration.z);
  gX = float(g.gyro.x);
  gY = float(g.gyro.y);
  gZ = float(g.gyro.z);
  
  while (gpsSerial.available() > 0) { // Get the current GPS data
    if (gps.encode(gpsSerial.read())) {
      gpsHr = gps.time.hour();
      gpsMin = gps.time.minute();
      gpsSec = gps.time.second();
      gpsSats = gps.satellites.value();
      gpsLat = gps.location.lat();
      gpsLong = gps.location.lng();
      gpsSpeed = gps.speed.kmph();
    }
  }

  adc0 = ads.readADC_SingleEnded(0); // Get current potentiometer readings
  v_ADC = (adc0 * 0.1875)/1000;
  
  // USE FOR GPS DATA
  //sprintf(dOut,"%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",data_time,gpsHr,gpsMin,gpsSec,gpsLat,gpsLong,gpsSpeed,gpsSats,aX,aY,aZ,gX,gY,gZ,v_ADC);

  // USE FOR ON CHIP TIME
  sprintf(dOut,"%lu,%f,%f,%f,%f,%f,%f",millis(),aX,aY,aZ,gX,gY,gZ);

  testData.println(dOut);
  // SAVE DATA EVERY TEN SECONDS
  if(millis() - t >= 10000){ 
    testData.close();
    testData = SD.open(dataFilePath, FILE_APPEND);
    t = millis();
  }
}
