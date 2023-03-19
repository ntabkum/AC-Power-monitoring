// Created by Nava Tabkum  5/30/2020 for AMV PZEM_004T v.3 with Temp
// with V21-v27 for additional sensors
//

#include <BlynkSimpleEsp8266.h>
#include <DNSServer.h>
#include "config.h"
#include <Wire.h>

//#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>

//#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C

#include <SoftwareSerial.h>  //  ( NODEMCU ESP8266 )
SoftwareSerial pzem(D5,D6);  // (RX,TX) connect to TX,RX of PZEM for NodeMCU IO12 
//SoftwareSerial pzem(D7,D8);  // (RX,TX) connect to TX,RX of PZEM
#include <ModbusMaster.h>

char auth1[] = "ePAgYuHJtuLIFmBXcn1Y7E1sJY4HrvqU"; // home VM n.tabkum@gmail.com
char auth2[] = "LYmwQCoZvUlSXvhmyBsm-ldEK8B-dyIk";//

float t, p, a, h;
bool auth0 = false;

#define vPIN_VOLTAGE                V21   
#define vPIN_FREQUENCY              V22
#define vPIN_CURRENT_USAGE          V23
#define vPIN_Temp                   V24
#define vPIN_Pressure               V25
#define vPIN_Altitude               V26
#define vPIN_Humidity               V27

WidgetLED led1(V28);
/*
#define vPIN_ACTIVE_POWER           V2
#define vPIN_ACTIVE_ENERGY          V3
#define vPIN_POWER_FACTOR           V5
#define vPIN_OVER_POWER_ALARM       V6
*/
ModbusMaster node;

BlynkTimer T0, T1;

// int timerTask1;
double U_PR, I_PR,  P_PR,  PPR, PR_F, PR_PF, PR_alarm;
uint8_t result;  
// uint16_t data[6];
//uint16_t timeslice = millis();

void setup(){
    
  Serial.begin(115200);
  // pinMode(2, OUTPUT); // D4 onboard LED
  pinMode(5, INPUT_PULLUP); //
  pinMode(4, OUTPUT); //
  pinMode(0, INPUT); //
  pinMode(16, OUTPUT);//
  
  digitalWrite(4, 1);
  digitalWrite(16, 0);
  
  delay(3000);//
  if (!digitalRead(5)) {
    WiFi.disconnect(true); 
    delay(2000); 
  } 
   if (!digitalRead(0)) {
    auth0 = true;
    digitalWrite(16, 1);
   }
   
 
  io.connect();
  if(auth0) {
  Blynk.config(auth2, "navcomm.cc", 55555);}
  else { 
  Blynk.config(auth1, "navcomm.cc", 55555);}
  
   //Serial.println(WiFi.SSID());
   //Serial.println(WiFi.psk());
     
  //Serial.println("Start serial"); 
  pzem.begin(9600); 
  //Serial.println("Start PZEM serial");
  node.begin(1, pzem);  
  //Serial.println("Start PZEM"); // 1 = ID MODBUS

  Wire.begin(2, 13); // or sda = gpio2, scl = gpio13
  Wire.setClock(100000); // clock F = 100Khz
  
  T1.setInterval(5000L, Temp);
    if (!bme.begin()) {
    
    //Serial.println("Could not find a valid BME280 sensor...");
    while (1);
    } 
    T0.setInterval(1000L, LED4);
    //Serial.println("BME280 Connected");
    
  
}

bool flag = true;

void updateBlynk() {
  
  Blynk.virtualWrite(vPIN_VOLTAGE,               U_PR);
  Blynk.virtualWrite(vPIN_FREQUENCY,             PR_F);
  Blynk.virtualWrite(vPIN_CURRENT_USAGE,         I_PR);
  if (flag) {
    led1.off();flag = false;
    //Serial.println("off");
   } else {
    led1.on();flag = true;
    //Serial.println("on");   
   }
   
   // digitalWrite(2, !digitalRead(2));
  //Blynk.virtualWrite(vPIN_ACTIVE_POWER,          P_PR);
  //Blynk.virtualWrite(vPIN_ACTIVE_ENERGY,         PPR);
  //Blynk.virtualWrite(vPIN_POWER_FACTOR,          PR_PF);
  //Blynk.virtualWrite(vPIN_OVER_POWER_ALARM,      PR_alarm);
}
void LED4() { 
  digitalWrite(4, !digitalRead(4));  //LED flip-flop
}

void Temp() {
 
    t = bme.readTemperature();
    t = t*1.8+23.0;
    p = bme.readPressure()/100.0F; //hPa
    a = bme.readAltitude(1013.25); //meter
    a = a * 3.28084;// in feets
    h = bme.readHumidity();//
    Blynk.virtualWrite(vPIN_Temp, t);
    Blynk.virtualWrite(vPIN_Pressure, p);
    Blynk.virtualWrite(vPIN_Altitude, a);
    Blynk.virtualWrite(vPIN_Humidity, h);
}
void loop(){
Blynk.run();
T0.run();
T1.run();
result = node.readInputRegisters(0x0000, 10);
  if (result == node.ku8MBSuccess)  {
U_PR      = (node.getResponseBuffer(0x00)/10.0f);
//I_PR      = (node.getResponseBuffer(0x01)/1000.000f);
//P_PR      = (node.getResponseBuffer(0x03)/10.0f);
//PPR       = (node.getResponseBuffer(0x05)/1000.0f);
PR_F      = (node.getResponseBuffer(0x07)/10.0f);
//PR_PF     = (node.getResponseBuffer(0x08)/100.0f);
//PR_alarm  = (node.getResponseBuffer(0x09));  
 } else { 
U_PR      = 0;
I_PR      = 0;
//P_PR      = 0;
//PPR       = 0;
PR_F      = 0;
//PR_PF     = 0;
//PR_alarm  = 0;
 }

    /*Serial.print("U_PR:     "); Serial.println(U_PR);   // V
    Serial.print("I_PR:     "); Serial.println(I_PR,3);   //  A
    Serial.print("P_PR:     "); Serial.println(P_PR);   //  W 
    Serial.print("PPR:      "); Serial.println(PPR,3);   // kWh
    Serial.print("PR_F:     "); Serial.println(PR_F);    // Hz
    Serial.print("PR_PF:    "); Serial.println(PR_PF);  
    Serial.print("PR_alarm: "); Serial.println(PR_alarm,0);
Serial.println("====================================================");*/

updateBlynk();
  // delay(50);

}
