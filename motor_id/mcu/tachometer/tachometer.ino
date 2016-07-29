/*
  file: tachometer.ino
  date: 20160618
  description : read motor speed

*/


#include <ros.h>
#include <std_msgs/Float64.h>

#define encoderPinA 2
#define encoderPinB 3
#define LOOPTIME 40
#define CPR 28
#define gear_ratio 65.5


ros::NodeHandle nh;
std_msgs::Float64 motor_speed;
ros::Publisher pub_speed("motor_speed",&motor_speed);

double rad_tick=(double)6.28/32767; //resolution
double degree_tick=(double)360/32767; //resolution

//ENC STATE
int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;

volatile int lastEncoded = 0;
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastSend = 0;                    // send timing
long dT = 0;

volatile long encoderPos = 0;
volatile long unknownvalue = 0;
double omega_actual = 0;


void setup() { 
       
    pinMode(encoderPinA, INPUT); 
    digitalWrite(encoderPinA, HIGH); // turn on pullup resistor
    pinMode(encoderPinB, INPUT); 
    digitalWrite(encoderPinB, HIGH); // turn on pullup resistor

    attachInterrupt(0, doEncoder, CHANGE); // encoder pin on interrupt 0 - pin 2
    attachInterrupt(1, doEncoder, CHANGE);
    nh.initNode();
    nh.advertise(pub_speed);

    Serial.begin (57600);

} 

void loop(){

  
  if((millis()-lastMilli) >= LOOPTIME){
    dT = millis()-lastMilli;
    lastMilli = millis();
    getMotorData();
    motor_speed.data=omega_actual;
    pub_speed.publish(&motor_speed);
    printTemple();
    nh.spinOnce();
  } 

}
void getMotorData()
{
  static long encoderPosPre = 0;
  omega_actual = ((encoderPos - encoderPosPre)*(1000/dT))*2*PI/(CPR*gear_ratio);  //ticks/s to rad/s
  encoderPosPre = encoderPos;

}



void doEncoder() {
    // encoderPos++;
    pinAState = digitalRead(2);
    pinBState = digitalRead(3);

    if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
    encoderPos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
    encoderPos --;
    if (pinAStateOld == 1 && pinBStateOld == 1) // unknown
    unknownvalue ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // unknown
    unknownvalue ++;
    }
    if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
    encoderPos ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
    encoderPos --;
    if (pinAStateOld == 1 && pinBStateOld == 0) // unknown
    unknownvalue ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // unknown
    unknownvalue ++;
    }
    if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
    encoderPos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
    encoderPos --;
    if (pinAStateOld == 0 && pinBStateOld == 0) // unknown
    unknownvalue ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // unknown
    unknownvalue ++;
    }

    if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
    encoderPos ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
    encoderPos --;
    if (pinAStateOld == 0 && pinBStateOld == 1) // unknown
    unknownvalue ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // unknown
    unknownvalue ++;
    }
    pinAStateOld = pinAState;
    pinBStateOld = pinBState;
}



void printTemple(){

        Serial.print(" speed: ");Serial.print(omega_actual);
        Serial.println("");     
  
  }

