#include "Arduino.h"
#include "Servo.h"
#include "SimpleTimer.h"

#include "Adafruit_NeoPixel.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_MPR121.h"

#include <RtcDS3231.h>
//
//TODO _> I2c DS1302 or DS3231
//#define TIME_24_HOUR      false
//RTC_DS1307 rtc = RTC_DS1307();

int hours = 0;
int minutes = 0;
int seconds = 0;

bool blinkColon = false;

int16_t ax, ay, az, gx, gy, gz;
const int MPU_addr=0x68;  // I2C address of the MPU-6050

#define OLED_RESET D1 // Not Connected, software just requires it
#define PIXEL_COUNT 60 // need to count
#define defaultBrightness 100 // should I reserve energy?


#define PIX_PIN D1 
#define SWITCH_PIN D7

#define STAMPERL_PIN D6
#define COMPENSATION_PIN D5
//D3,D4 - i2c
#define FRONT_PIN D5

#define BACK_PIN D7
#define STEP_PIN D8
#include <EEPROM.h>

SimpleTimer timer;
Servo stamperl;
Servo compensation;
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_MPR121 cap = Adafruit_MPR121();


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

RtcDS3231<TwoWire> Rtc(Wire);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIX_PIN, NEO_GRB + NEO_KHZ800);


const char *ssid = "Anton-0tcsbiez6m"; // "Berta-8axlgha56g"
const char *password = "spoon";

volatile int dancedTonight = 0;
volatile int currentActivity = 4; // unfortunately inverted...
volatile unsigned long sum = 0;
volatile unsigned long meanValue = 0;
int sumCount = 0;
int sampleRate = 5;
long meanActivitySum = 0;
int  meanActivityCounter = 0;
int meanActivitySampleRate = 10;

void setup() {
  EEPROM.begin(512);

  storeVal(3,0);
  
  Serial.begin(9600);
  Wire.begin(D2,D3);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  initDisplay();
  welcome();
  initMPU();
  

  strip.begin();
  strip.setBrightness(defaultBrightness); // should be stored in EEPROM
  strip.show();
  delay(1000);

  Rtc.Begin();
  RtcDateTime now = Rtc.GetDateTime();
  // day month

  int lastDancedMonth = restoreVal(1);
  int lastDancedDay = restoreVal(2);

  if(now.Month() != lastDancedMonth || now.Day() != lastDancedDay){
    storeVal(1, now.Month());
    storeVal(2, now.Day());
    storeVal(3,dancedTonight);
    storeVal(4,currentActivity);
  }else{
    dancedTonight = restoreVal(3);
    currentActivity = restoreVal(4);
  }

  updateDanced(); // will be called by readyFordanced

  timer.setInterval(50, animateLed);
  timer.setInterval(100, readActivity);
  timer.setInterval(1000, readyForDanced); // at first to fake it just press
  timer.setInterval(1000, updateOLED);
  
  pinMode(SWITCH_PIN, INPUT_PULLUP);
}

//----------------------------------------------------------------------
void loop() { // main loop does not need to be changed
//----------------------------------------------------------------------

  //Serial.println(analogRead(A0)); // microphone
  //Serial.println(digitalRead(SWITCH_PIN)); // push button

  timer.run();
} 

void showClock(){
  RtcDateTime now = Rtc.GetDateTime();
  display.clearDisplay();
  showBigAtLineWith(0, String(now.Month()) + "/" + String(now.Day()));
  showBigAtLineWith(1, String(now.Hour()) + ":" + String(now.Minute()) + ":" + String(now.Second()) );
  display.display();
}

void updateDanced(){
  strip.clear();
  for( int i = 0; i < dancedTonight; i++){
    strip.setPixelColor(i, 0xffffff); 
  }
  strip.show();
}


void readActivity(){
  readMPU();
  if(sumCount >= sampleRate){
    meanValue = sum/(sampleRate + 1);
    sum = 0;
    sumCount = 0;
    //meanActivitySum = map();
    meanActivitySum += map(meanValue, 1600, 1800, 0, 4);
    meanActivityCounter++;
    if(meanActivityCounter >= meanActivitySampleRate){
      if(currentActivity > meanActivitySum/(meanActivitySampleRate + 1)){
        if(currentActivity > 0){
          currentActivity--;
        }
      }else if(currentActivity < meanActivitySum/(meanActivitySampleRate + 1)){
        if(currentActivity < 4){
          currentActivity++;
        }
      }
      storeVal(4,currentActivity);
      meanActivitySum = 0;
      meanActivitySampleRate = 0;
    }
  }else{
    sum += abs(ax)/10 + abs(ay)/10 + abs(az)/10;
    sumCount++;
  }
}

void updateOLED(){
  showOneLine(String(currentActivity));
}


int ledState = HIGH;  
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;

void readyForDanced(){
  int reading = digitalRead(SWITCH_PIN);
  if(reading == LOW){
    if(dancedTonight < 8){
      dancedTonight++;
      storeVal(3,dancedTonight);
      updateDanced();
    } else {
      dancedTonight = 0;
      storeVal(3,dancedTonight);
      updateDanced();
    }
  }
}
//----------------------------------------------------------------------
void showOneLine(String message){
//----------------------------------------------------------------------
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(message);
  display.display();
}

//----------------------------------------------------------------------
void showAtLineWith(int line, String message){
//----------------------------------------------------------------------
  //display.clearDisplay(); // we want to be able to cascadate
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,line*8);
  display.println(message);
  //display.display(); // -> ugly updating
}

//----------------------------------------------------------------------
void showBigAtLineWith(int line, String message){
//----------------------------------------------------------------------
  //display.clearDisplay(); // we want to be able to cascadate
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,line*16);
  display.println(message);
  //display.display(); // -> ugly updating
}

//----------------------------------------------------------------------
void initDisplay(){
//----------------------------------------------------------------------
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.clearDisplay();
  display.display();
}

//----------------------------------------------------------------------
void welcome(){
//----------------------------------------------------------------------
    display.clearDisplay();
    
    showBigAtLineWith(0, "FLIRTY");
    showBigAtLineWith(1, "DANCING <3");
    display.display();
}

void initMPU(){
   Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.write("Initiating MPU");
}
void readMPU(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  gx=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  gy=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  gz=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  int temperature=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  ax=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  ay=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  az=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Serial.print("AcX = "); Serial.print(ax);
  Serial.print(" | AcY = "); Serial.print(ay);
  Serial.print(" | AcZ = "); Serial.print(az);
  Serial.println();
  //Serial.print(" | Tmp = "); Serial.print(temperature/340.00+36.53);  //equation for temperature in degrees C from datasheet

  Serial.print(" | GyX = "); Serial.print(gx);
  Serial.print(" | GyY = "); Serial.print(gy);
  Serial.print(" | GyZ = "); Serial.println(gz);
 
  //gx -> pitch
  //gy -> roll
  // gz -> az
  //ax -> roll acc
  //ay -> pitch acc
  //az -> yaw acc
}
//----------------------------------------------------------------------
void storeVal(int addr, int val){
//----------------------------------------------------------------------
  EEPROM.write(addr, val);
  EEPROM.commit();
}

//----------------------------------------------------------------------
int restoreVal(int addr){
//----------------------------------------------------------------------
  return EEPROM.read(addr);
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void animateLed() {
  uint16_t minimum, maximum;
  switch (currentActivity) {
  case 4: // violet
    minimum = 210;
    maximum = 230;
    break;
  case 3: // blue
    minimum = 120;
    maximum = 160;
    break;
  case 2: // green
     minimum = 50;
    maximum = 70;
    break;
  case 1: // yellow
    minimum = 28;
    maximum = 40;
    break;
  default: // red
    minimum = 0;
    maximum = 14;
  }
  
  for(int i=8; i < 16; i++) {
        strip.setPixelColor(i, Wheel(random(minimum, maximum)));
  }
  strip.show();
}
