#include <Wire.h>
#include <USaber.h>

#define SMALL_TRESHOLD   5
#define MEDIUM_TRESHOLD  9
#define LARGE_TRESHOLD  18

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#define LED_0 3
#define LED_1 5
#define LED_2 6
#define LED_3 9
#define LED_4 10
#define LED_5 11

#define DFPlayer_RX 8
#define DFPlayer_TX 7

#define INTTRRUPT_PIN 2

#define STATE_TURNING_ON 0
#define STATE_TURNED_ON 1
#define STATE_TURNING_OFF 2
#define STATE_TURNED_OFF 3

//DFPlayer
SoftwareSerial mySoftwareSerial(DFPlayer_RX, DFPlayer_TX); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

//LED state
int led_light[6] = { 255 ,255 ,255 ,255 ,255 ,255 }; // 
volatile int state = STATE_TURNED_OFF;

//debounce
long debouncing_time = 100; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

//Tolerance threshold data for MPU6050 motion manager
MPU6050TolData gToleranceData;
//Calibration data for MPU6050 motion manager
MPU6050CalibrationData gMpuCalData;

AMotionManager* apMotion;

void setup() {
  // put your setup code here, to run once:
  
  //init DFPlayer 
  mySoftwareSerial.begin(9600);
  myDFPlayer.begin(mySoftwareSerial);
  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30

  //init interrupt pin
  //pinMode(INTTRRUPT_PIN, INPUT_PULLUP);
  apMotion = new Mpu6050MotionManager(&gToleranceData, &gMpuCalData);
  apMotion->Init();
  
  attachInterrupt(digitalPinToInterrupt(INTTRRUPT_PIN), change_state, RISING); //assign int0
  light_off();


  //Set MPU6050 swing tolerance threasholds here.
  //Tweak as needed to adjust sensitivity
  gToleranceData.mSwingLarge  = LARGE_TRESHOLD;
  gToleranceData.mSwingMedium = MEDIUM_TRESHOLD;
  gToleranceData.mSwingSmall  = SMALL_TRESHOLD;

  //Set MPU6050 calibration data here.
  //Tweak as needed to improve accuracy
  gMpuCalData.mAccelXOffset = -1848;
  gMpuCalData.mAccelYOffset = -5409;
  gMpuCalData.mAccelZOffset = 1640;
  gMpuCalData.mGyXOffset = -5;
  gMpuCalData.mGyYOffset = 48;
  gMpuCalData.mGyZOffset = 23;

  Serial.begin(9600);
  Serial.println("Start");
  
}

void loop() {
  
  // put your main code here, to run repeatedl
  switch(state){
    case STATE_TURNING_ON:
    Serial.println("STATE_TURNING_ON");
      smooth_turn_on_the_light();
      break;
    case STATE_TURNED_ON:
    Serial.println("STATE_TURNED_ON");
      breath_light();
      break;
    case STATE_TURNING_OFF:
      Serial.println("STATE_TURNING_OFF");
      smooth_turn_off_the_light();
      break;
    case STATE_TURNED_OFF:
      Serial.println("STATE_TURNED_OFF");
      light_off();
      break;
  }
  
}

void change_state() {
   if(state==STATE_TURNED_ON){
     state = STATE_TURNING_OFF;
   } else if(state==STATE_TURNED_OFF){
     state = STATE_TURNING_ON;
   }

}

void set_light() {
  if(led_light[0]==255){
    digitalWrite(LED_0, HIGH);
    pinMode(LED_0, INPUT);
  }else{
    analogWrite(LED_0, led_light[0]);
    pinMode(LED_0, OUTPUT);
  }
  if(led_light[1]==255){
    digitalWrite(LED_1, HIGH);
    pinMode(LED_1, INPUT);
  }else{
    analogWrite(LED_1, led_light[1]);
    pinMode(LED_1, OUTPUT);
  }
  if(led_light[2]==255){
    digitalWrite(LED_2, HIGH);
    pinMode(LED_2, INPUT);
  }else{
    analogWrite(LED_2, led_light[2]);
    pinMode(LED_2, OUTPUT);
  }
   if(led_light[3]==255){
    digitalWrite(LED_3, HIGH);
    pinMode(LED_3, INPUT);
  }else{
    analogWrite(LED_3, led_light[3]);
    pinMode(LED_3, OUTPUT);
  }
  if(led_light[4]==255){
    digitalWrite(LED_4, HIGH);
    pinMode(LED_4, INPUT);
  }else{
    analogWrite(LED_4, led_light[4]);
    pinMode(LED_4, OUTPUT);
  }
  if(led_light[5]==255){
    digitalWrite(LED_5, HIGH);
    pinMode(LED_5, INPUT);
  }else{
    analogWrite(LED_5, led_light[5]);
    pinMode(LED_5, OUTPUT);
  }
}

void breath_light() {
  int fade_amount = 10;
  for(int i = 0 ; i < 6 ; i++ ){
    led_light[i] = 0;
  }
  //Serial.println(myDFPlayer.readState());
  myDFPlayer.loop(1);
  unsigned long lastTime = millis();
  int swing_count = 0;
  int clash_count = 0;
  while( state == STATE_TURNED_ON ){
    Serial.println("Xxxx");
    apMotion->Update();
    if((millis() - lastTime >= 200)){
      lastTime = millis();
      Serial.println("reset count");
      Serial.println(swing_count);
      Serial.println(clash_count);
      swing_count = 0;
      clash_count = 0;
      }
      
    if(apMotion->IsClash())
    {
      Serial.print("Clash detected:");
      switch(apMotion->GetClashMagnitude())
      {
      case eeSmall:
        Serial.println("Small");
        break;
      case eeMedium:
        Serial.println("Medium");
        break;
      case eeLarge:
        Serial.println("Large");
        if(clash_count == 0){
          clash_count++;
          lastTime = millis();
          myDFPlayer.play(random(3)+7); 
          for(int i = 0 ; i < 6 ; i++ ){
            led_light[i] = 255;
          }
          set_light();
          delay(30);
          for(int i = 0 ; i < 6 ; i++ ){
            led_light[i] = 0;
          }
          fade_amount = 10;
          set_light();
          delay(350);
          
          myDFPlayer.loop(1);
        }
        clash_count++;
        break;
      default:
        Serial.println("Unknown");
        break;
      }
    }else if(apMotion->IsSwing())
    {
      swing_count++;
      if(swing_count==3 && clash_count==0){
        Serial.println(swing_count);
        Serial.println(clash_count);
        lastTime = millis();
        swing_count++;
        myDFPlayer.play(random(3)+4);
        delay(500);
        myDFPlayer.loop(1);
      }
      Serial.print("Swing detected:");
      switch(apMotion->GetSwingMagnitude())
      {
      case eeSmall:
        Serial.println("Small");
        break;
      case eeMedium:
        Serial.println("Medium");
        break;
      case eeLarge:
        Serial.println("Large");
        break;
      default:
        Serial.println("Unknown");
        break;
      }
    }


    
    led_light[0] += fade_amount;
    led_light[1] += fade_amount;
    led_light[2] += fade_amount;
    led_light[3] += fade_amount;
    led_light[4] += fade_amount;
    led_light[5] += fade_amount;
    if(led_light[0]==0 || led_light[0]>=150){
      fade_amount = -fade_amount;
    }
    set_light();
    delay(50);
  }
}

void smooth_turn_on_the_light() {
  myDFPlayer.play(2); 
  light_off();
  while( led_light[0]>0 ||led_light[1]>0 ||led_light[2]>0 ||led_light[3]>0 ||led_light[4]>0 ||led_light[5]>0 ){
    if(led_light[4]<255){
      led_light[5] = increase_light_value(led_light[5]);
    }
    if(led_light[3]<255){
      led_light[4] = increase_light_value(led_light[4]);
    }
    if(led_light[2]<255){
      led_light[3] = increase_light_value(led_light[3]);
    }
    if(led_light[1]<255){
      led_light[2] = increase_light_value(led_light[2]);
    }
    if(led_light[0]<255){
      led_light[1] = increase_light_value(led_light[1]);
    }
    led_light[0] = increase_light_value(led_light[0]);
    set_light();
    delay(50);
  }
  if(state == STATE_TURNING_ON){
    state = STATE_TURNED_ON;
  }
}

void smooth_turn_off_the_light() {
  myDFPlayer.play(3); 
  light_on();
  while( led_light[0]<255 | led_light[1]<255 || led_light[2]<255 || led_light[3]<255 || led_light[4]<255 || led_light[5]<255  ){
    led_light[5] = decrease_light_value(led_light[5]);
    if(led_light[1]>0){
      led_light[0] = decrease_light_value(led_light[0]);
    }
    if(led_light[2]>0){
      led_light[1] = decrease_light_value(led_light[2]);
    }
     if(led_light[3]>0){
      led_light[2] = decrease_light_value(led_light[2]);
    }
    if(led_light[4]>0){
      led_light[3] = decrease_light_value(led_light[3]);
    }
    if(led_light[5]>0){
      led_light[4] = decrease_light_value(led_light[4]);
    }
    set_light();
    delay(50);
  }
  if(state == STATE_TURNING_OFF){
    state = STATE_TURNED_OFF;
    light_off();
  }
}
int increase_light_value(int value) {
  if(value == 0 ){
    return 0;
  } else if(value == 255) {
    return value - 55;
  } else {
    return value - 50;
  }
}

int decrease_light_value(int value) {
  if(value == 255 ){
    return 255;
  } else if(value == 0) {
    return value + 55;
  } else {
    return value + 50;
  }
}


void light_off() {
  for(int i = 0 ; i < 6 ; i++ ){
    led_light[i] = 255;
  }
  set_light();
}

void light_on() {
  for(int i = 0 ; i < 6 ; i++ ){
    led_light[i] = 0;
  }
  set_light();
}
