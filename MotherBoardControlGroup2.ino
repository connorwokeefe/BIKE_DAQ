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




MOTHER BOARD GROUP1
uint8_t broadcastAddress0[] = {0xC4, 0xDE, 0xE2, 0x17, 0x01, 0xAC}; // Board 0 - REPLACE WITH THE MAC Address of your sender

DAUGHTER GROUP 1
uint8_t broadcastAddress1[] = {0x08, 0xB6, 0x1F, 0x9C, 0xCE, 0x84}; // Board 1
uint8_t broadcastAddress2[] = {0x08, 0xB6, 0x1F, 0x9C, 0xB3, 0xD4}; // Board 2
uint8_t broadcastAddress3[] = {0x08, 0xB6, 0x1F, 0x9C, 0xD1, 0x9C}; // Board 3
uint8_t broadcastAddress4[] = {0x08, 0xB6, 0x1F, 0x9C, 0xCE, 0xE8}; // Board 4

// MOTHER BOARD GROUP 2
//uint8_t broadcastAddress9[] = {0x94, 0xB5, 0x55, 0xFD, 0x5D, 0x5C}; // Board 9

// DAUGHTER GROUP 2
// uint8_t broadcastAddress5[] = {0x08, 0xB6, 0x1F, 0x9C, 0xB4, 0x34}; // Board 5
// uint8_t broadcastAddress6[] = {0x08, 0xB6, 0x1F, 0x9C, 0xCE, 0xB8}; // Board 6
// uint8_t broadcastAddress7[] = {0x08, 0xB6, 0x1F, 0x9C, 0xCE, 0xE8}; // Board 7
// uint8_t broadcastAddress8[] = {0x08, 0xB6, 0x1F, 0x9C, 0xD1, 0x80}; // Board 8




// Variable to store if sending data was successful
String success;

// Variables for communication checks
bool isReady = false;
bool isCalibrated = false;
bool isGoing = false;
int attempts = 0;

// Variable to store serial input
String input = "";

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
  int id;
  String talk;
} struct_message;

struct_message myData;

// Variables to store recieved messages from each board.
struct_message heard1;
struct_message heard2;
struct_message heard3;
struct_message heard4;

struct_message boardsStruct[4] = {heard1, heard2, heard3, heard4};

// Variable to send message
struct_message outgoingTalk;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[24];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[24];
  //Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].talk = myData.talk;
  Serial.print(boardsStruct[myData.id-1].talk);
  Serial.println();
}

// Call this to empty outgoing message
void emptyTalk(){
  for(int i = 0; i < 5; i++){
    boardsStruct[i].talk = "";
  }
}

// Function used to check for board communication and functionality via bluetooth "talk"
void boardCheck(){
//  if(boardsStruct[0].talk == "1" & boardsStruct[1].talk == "1" & boardsStruct[2].talk == "1" & boardsStruct[3].talk == "1"){
//    isReady = true;  
//    emptyTalk();
//    Serial.println("ALL BOARDS ARE COMMUNICATING");
//  }
  if(boardsStruct[1].talk == "l"){
    isReady = true;  
    emptyTalk();
    Serial.println("ALL BOARDS ARE COMMUNICATING");
  }
//  if(boardsStruct[0].talk == "c" & boardsStruct[1].talk == "c" & boardsStruct[2].talk == "c" & boardsStruct[3].talk == "c"){
//    isCalibrated = true; 
//    emptyTalk(); 
//    Serial.println("CALIBRATION IS COMPLETE");
//  }
  if(boardsStruct[1].talk == "c"){
    isCalibrated = true; 
    emptyTalk(); 
    Serial.println("CALIBRATION IS COMPLETE");
  }
//  if(boardsStruct[0].talk == "g" & boardsStruct[1].talk == "g" & boardsStruct[2].talk == "g" & boardsStruct[3].talk == "g"){
//    isGoing = true;  
//    emptyTalk();
//    Serial.println("WE ARE READY TO GO");
//  }
  if(boardsStruct[1].talk == "g"){
    isGoing = true; 
    emptyTalk(); 
    Serial.println("WE ARE COLLECTING DATA");
  }
}

// Function to send message to daughter boards through serial comm
void communicationRequest(){
  outgoingTalk.talk = Serial.readStringUntil('\n'); //Wait for input
  if(outgoingTalk.talk != ""){
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(0, (uint8_t *) &outgoingTalk, sizeof(outgoingTalk));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      outgoingTalk.talk = "";
    }
    else {
      Serial.println("Error sending the data");
    }
  }
  boardCheck();
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress5, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress6, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
  /// register third peer
  memcpy(peerInfo.peer_addr, broadcastAddress7, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
  /// register fourth peer
  memcpy(peerInfo.peer_addr, broadcastAddress8, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Waiting for board communication requests...");
  outgoingTalk.talk = "";
  while(!isReady){
    communicationRequest();
  }

  Serial.println("ALL BOARDS ARE COMMUNICATING");
  delay(1000);
  
  while(!isCalibrated)
    communicationRequest();
  }

  while(!isGoing){
    communicationRequest();
  }
}
 
void loop() {
  communicationRequest();

}
