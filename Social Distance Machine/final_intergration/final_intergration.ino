/*
  Read the temperature pixels from the MLX90640 IR array
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14769

  This example initializes the MLX90640 and outputs the 768 temperature values
  from the 768 pixels.

  This example will work with a Teensy 3.1 and above. The MLX90640 requires some
  hefty calculations and larger arrays. You will need a microcontroller with 20,000
  bytes or more of RAM.

  This relies on the driver written by Melexis and can be found at:
  https://github.com/melexis/mlx90640-library

  Hardware Connections:
  Connect the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  to the Qwiic board
  Connect the male pins to the Teensy. The pinouts can be found here: https://www.pjrc.com/teensy/pinout.html
  Open the serial monitor at 9600 baud to see the output
*/
//------------for ESP-NOW communication------------
#include <esp_now.h>
#include <WiFi.h>
//A4:CF:12:73:93:0C
uint8_t broadcastAddress[] = {0xA4, 0xCF, 0x12, 0x73, 0x4B, 0x48};
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  String d;
  bool e = false;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

bool detected = false;

// -----------------temperature sensor----------------

#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

static float mlx90640To[768];
paramsMLX90640 mlx90640;

// -----------------Ultra sensor----------------

#define PWM0A 18
#define AIN02 16 // 控制輸入A01
#define AIN01 17 // 控制輸入A02
#define STBY0 19
//設定HC-SR04接腳位置
//int trigPin = 12;  // Trigger pin
//int echoPin = 14;  // Echo pin
//long duration1, duration2, duration3, duration4, disCM1, disCM2, disCM3, disCM4;
long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor;
//int ledredPin = 13;  // LED pin
const int buzzer = 2; //27

// Ultrasensor Pin
int trigPin1 = 12;  // Trigger pin
int echoPin1 = 14;  // Echo pin
int trigPin2 = 27;  // Trigger pin
int echoPin2 = 26;  // Echo pin
int trigPin3 = 33;  // Trigger pin
int echoPin3 = 32;  // Echo pin
int trigPin4 = 19;  // Trigger pin
int echoPin4 = 18;  // Echo pin


#include "pitches.h"
#define Do  523
#define Re  587
#define Mi  659
#define Fa  698
#define So  784
#define La  880
#define Si  988
int melody[7] = {Do, Re, Mi, Fa, So, La, Si};
 
// Motor 1模態設定
//Tb6612fng motor11(STBY0, AIN01, AIN02, PWM0A);
int playing = 0;
void tone(byte pin, int freq) {
  ledcSetup(0, 2000, 8); // setup beeper
  ledcAttachPin(pin, 0); // attach beeper
  ledcWriteTone(0, freq); // play tone
  playing = pin; // store pin
  delay(50);
}
void noTone() {
  tone(playing, 0);
}

void SonarSensor(int trigPin,int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
}



void setup()
{
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("MLX90640 IR Array Example");

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

  //Once params are extracted, we can release eeMLX90640 array

  // Ultra sensor

  //Serial.begin (115200);
  //定義Pin腳模式
  pinMode(buzzer, OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);

  digitalWrite(buzzer, LOW);

  //for ESP-NOW communication
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
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop()
{
  //for ESP-NOW communication
  detected = false;
  
  Serial.println("Hello world");

  SonarSensor(trigPin1, echoPin1);
  FrontSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  RightSensor = distance;
  SonarSensor(trigPin3, echoPin3);
  BackSensor = distance;
  SonarSensor(trigPin4, echoPin4);
  LeftSensor = distance;
  
  Serial.print("FrontSensor");
  Serial.print(FrontSensor);
  Serial.print("cm");
  Serial.println();
  //delay(100);

  if (FrontSensor<=50) {
    //digitalWrite(ledredPin, HIGH); // turn LED on
    //tone(buzzer, 988); // turn LED on
    //digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    detected = true;
    //motor11.brake();
  } 
  else {
    //digitalWrite(ledredPin, LOW); // turn LED off
    //tone(buzzer, 256); // turn LED off
    noTone();
    //delay(100);
  }

  Serial.print("RightSensor");
  Serial.print(RightSensor);
  Serial.print("cm");
  Serial.println();
  //delay(100);

  if (RightSensor<=50) {
    //digitalWrite(ledredPin, HIGH); // turn LED on
    //tone(buzzer, 988); // turn LED on
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    detected = true;
    //delay(100);
    //motor11.brake();
  } 
  else {
    //digitalWrite(ledredPin, LOW); // turn LED off
    //tone(buzzer, 256); // turn LED off
    noTone();
    //delay(100);
  }

  
  Serial.print("BackSensor");
  Serial.print(BackSensor);
  Serial.print("cm");
  Serial.println();
  //delay(100);

  if (BackSensor<=50) {
    //digitalWrite(ledredPin, HIGH); // turn LED on
    //tone(buzzer, 988); // turn LED on
    //digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    detected = true;
    //delay(100);
    //motor11.brake();
  } 
  else {
    //digitalWrite(ledredPin, LOW); // turn LED off
    //tone(buzzer, 256); // turn LED off
    noTone();
    //delay(100);
  }

  Serial.print("LeftSensor");
  Serial.print(LeftSensor);
  Serial.print("cm");
  Serial.println();
  //delay(100);

  // 判斷LED與馬達停止條件disCM<=10
  if (LeftSensor<=50) {
    //digitalWrite(ledredPin, HIGH); // turn LED on
    //tone(buzzer, 988); // turn LED on
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    detected = true;
    //delay(100);
    //motor11.brake();
  } 
  else {
    //digitalWrite(ledredPin, LOW); // turn LED off
    noTone(); // turn LED off
    //delay(100);
  }

// -----------------temperature sensor----------------
  
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  for (int x = 0 ; x < 10 ; x++)
  {
    Serial.print("Pixel ");
    Serial.print(x);
    Serial.print(": ");
    Serial.print(mlx90640To[x], 2);
    Serial.print("C");
    Serial.println();
  }

  delay(1000);

  //----------for ESP-NOW communication---------
  if(detected == true)
  {
    // Set values to send
    strcpy(myData.a, "THIS IS A CHAR");
    myData.b = random(1,20);
    myData.c = 1.5;
    myData.d = "People detected";
    myData.e = true;
  
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
     
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    //delay(2000);
  }
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
