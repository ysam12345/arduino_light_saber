#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "Timer.h"

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

//MPU 6050
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars                                                                                                                                                                                                                                                                                                                                                                                                       
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//LED state
int led_light[6] = { 255 ,255 ,255 ,255 ,255 ,255 }; // 
volatile int state = STATE_TURNED_OFF;

//debounce
long debouncing_time = 100; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

volatile bool mpuInterrupt = true;
void dmpDataReady() {
  mpuInterrupt = true;
  //detachInterrupt(0);
}

void setup() {
  // put your setup code here, to run once:
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // 載入與設定 DMP
  Serial.println("Initializing DMP...\n");
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("DMP failed, code: " + devStatus);
  }
  
  //init DFPlayer 
  mySoftwareSerial.begin(9600);
  myDFPlayer.begin(mySoftwareSerial);
  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30

  //init interrupt pin
  //pinMode(INTTRRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTTRRUPT_PIN), change_state, RISING); //assign int0
  light_off();
 
  Serial.println("Start");
}

void loop() {
  
  // put your main code here, to run repeatedl
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println("FIFO overflow!");
    } else if (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      Serial.println("getFIFOBytes");
      Serial.println(fifoCount);
      Serial.println(packetSize);
      //fifoCount -= packetSize;
      

      // 實際的加速度（去除重力）
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);

      /*
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);*/
    } else {
      Serial.println("dmp is not Ready");
    }
  
  
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
  Serial.println(myDFPlayer.readState());
  while(myDFPlayer.readState() == 513);
  myDFPlayer.loop(3);
  while( state == STATE_TURNED_ON ){
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
  myDFPlayer.play(1); 
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
  myDFPlayer.play(2); 
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
