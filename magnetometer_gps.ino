#include <Wire.h>                   //Wire for GPS
#include <TinyGPS++.h>              //GPS module
#include "SSD1306Ascii.h"           //OLED
#include "SSD1306AsciiAvrI2c.h"     //OLED
#include <SPI.h>                    //SPI for SD card
#include <SD.h>                     //SD card
#include <Adafruit_Sensor.h>        //Adafruit sensors
#include <Adafruit_HMC5883_U.h>     //GY-271

//#include <SoftwareSerial.h>         //GPS module for testing


//DEFINITIONS 
#define I2C_ADDRESS 0x3C            //OLED address
#define GPSBaud 9600                //GPS Serial Baud
#define CS 4                        //SD card

//#define RXPin 3                     //SoftwareSerial RX Pin
//#define TXPin 2                     //SoftwareSerial TX Pin





void(* resetFunc) (void) = 0; // RESET FUNCTION
static void p_gps(String &latitude, String &longitude); //PRINT GPS DATA
static void p_mag(String &mag_x,String &mag_y,String &mag_z,String &mag); //PRINT MAGNETOMETR DATA
static void vol(); // VOLTAGE PRINT
static void p_date(String &datedate, String &timetime); //DATE PRINT


//OBJECTS
SSD1306AsciiAvrI2c OLED;            //OLED object OLED
TinyGPSPlus GPS;                    //GPS object GPS
File MyFile;                        //File to create on SD crad MyFile
Adafruit_HMC5883_Unified MagSensor = Adafruit_HMC5883_Unified(12345); //GY-271 object MagSensor

//SoftwareSerial SoftSerial(RXPin, TXPin); //SoftwareSerial object

unsigned long time = 0;                      //time
unsigned long count = 0;                     //counts of loop
//Setup 
void setup() {

  OLED.begin(&Adafruit128x32, I2C_ADDRESS); //OLED start
  OLED.setFont(Arial14);    
  OLED.set2X();
  OLED.clear();
  
  Serial.begin(GPSBaud); // DEBUG
  while(!Serial){
    ;
    }
    
  
  //SoftSerial.begin(GPSBaud);                // DEBUG Serial GPS
  MagSensor.begin();                        //GY-271 start


  //SD card 
  if(!SD.begin(CS)){
    OLED.println("SD FAILED");
    delay(3000);
    OLED.clear();
    OLED.println("...");
    delay(3000);
    OLED.clear();
    resetFunc();
    }
   
  OLED.println("DONE!");
  delay(3000);
  OLED.clear();
  for(int i = 20; i > 0; --i){
    OLED.clear();
    OLED.println("WAIT " + String(i) + " s");
    delay(1000);
    }
    
  OLED.clear();
  OLED.println("READY");
  delay(3000);
  OLED.clear();
  
}

void loop() {

  OLED.clear();
  sensors_event_t EVENT; 
  MagSensor.getEvent(&EVENT);
  MyFile = SD.open("dane.txt", FILE_WRITE);

  
  
  String latitude = String(GPS.location.lat(),6);
  String longitude = String(GPS.location.lng(),6);
  String datedate = String(GPS.date.day()) +"/"+ String(GPS.date.month()) +"/"+ String(GPS.date.year());
  String timetime = String(GPS.time.hour()) +":"+ String(GPS.time.minute()) +":"+ String(GPS.time.second());
  String mag_x = String(EVENT.magnetic.x);
  String mag_y = String(EVENT.magnetic.z);
  String mag_z = String(EVENT.magnetic.y);
  String mag = String(sqrt(EVENT.magnetic.x*EVENT.magnetic.x+EVENT.magnetic.y*EVENT.magnetic.y+EVENT.magnetic.z*EVENT.magnetic.z));

  MyFile.print(datedate);
  MyFile.print("\t");
  MyFile.print(timetime);
  MyFile.print("\t");
  MyFile.print(latitude);
  MyFile.print("\t");
  MyFile.print(longitude);
  MyFile.print("\t");
  MyFile.print(mag_x);
  MyFile.print("\t");
  MyFile.print(mag_y);
  MyFile.print("\t");
  MyFile.print(mag_z);
  MyFile.print("\t");
  MyFile.print(mag);
  MyFile.println();
  MyFile.close();

  switch(count%6){
    case 0:
    p_gps(latitude,longitude);
    break;
    case 1:
    p_gps(latitude,longitude);
    break;
    case 2:
    p_mag(mag_x,mag_y,mag_z,mag);
    break;
    case 3:
    p_mag(mag_x,mag_y,mag_z,mag);
    break;
    case 4:
    vol();
    break;
    case 5:
    p_date(datedate,timetime);
    break;

    
  }
  count +=1;
  SDelay(1000);
}


static void SDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial.available())
      GPS.encode(Serial.read());
  } while (millis() - start < ms);
}


static void p_gps(String &latitude, String &longitude){

    
    OLED.set1X();
    OLED.print("Lat : ");
    OLED.print(latitude);
    OLED.println();
    OLED.print("Long : ");
    OLED.println(longitude);
}

static void p_date(String &datedate, String &timetime){
    OLED.set1X();
    OLED.print("DATE ");
    OLED.print(datedate);
    OLED.println();
    OLED.print("TIME ");
    OLED.print(timetime);
  }

static void p_mag(String &mag_x,String &mag_y,String &mag_z,String &mag){
    OLED.set1X();
    OLED.print("Bx ");
    OLED.print(mag_x);
    OLED.print("uT | ");
    OLED.print("By ");
    OLED.print(mag_y);
    OLED.println("uT");
    OLED.print("Bz ");
    OLED.print(mag_z);
    OLED.print("uT | ");
    OLED.print("B ");
    OLED.print(mag);
    OLED.println("uT");
  }
static void vol(){
    OLED.set2X();
    int wartosc = analogRead(A1);
    float napiecie = 5*wartosc/1024.0;
    OLED.print("U = ");
    OLED.print(" ");
    OLED.print(napiecie);
    OLED.println(" V");
  }
