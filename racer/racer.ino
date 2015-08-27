//import Shapr IR sensor library and motor library located in arduino library folder
#include <SharpIR.h>
#include <DualVNH5019MotorShield.h>
#include "PinChangeInt.h"            // Library for encoders interrupts

//For sensors - pins and other variables
#define sensor1Pin A0
#define sensor2Pin A1
#define sensor3Pin A2
#define sensor4Pin A3
#define sensor5Pin A4
#define shortRange 1080
#define longRange 20150


//For motors - pins and other variables
//Motor 1 Encoder Pins
//Yellow is encoder A, connected to arduino board pin 3
//White is encoder  B, connected to arduino board pin 5
#define m1_inA 3
#define m1_inB 5

//Motor 2 Encoder Pins
//Yellow is encoder A, connected to arduino board pin 11
//White is encoder  B, connected to arduino board pin 13
#define m2_inA 11
#define m2_inB 13

//For reading encoder values
//volatile int motor1A=0, motor1B=0, motor2A=0, motor2B=0;
static int motor1Aold = 0;
static int motor1Bnew = 0;
static int motor2Aold = 0;
static int motor2Bnew = 0;

int count = 0;
//SharpIR sharpIR1(sensor1Pin,20,93,shortRange);
DualVNH5019MotorShield md;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while (1);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // pinMode(sensor1Pin,INPUT);
  md.init();
  //Setting up of motors encoder pins
  pinMode(m1_inA, INPUT);
  pinMode(m1_inB, INPUT);
  pinMode(m2_inA, INPUT);
  pinMode(m2_inB, INPUT);
  digitalWrite(m1_inA, HIGH);
  digitalWrite(m1_inB, HIGH);
  digitalWrite(m2_inA, HIGH);
  digitalWrite(m2_inB, HIGH);
  
  PCintPort::attachInterrupt(m1_inA, notifyOnChangeM1A, CHANGE);  // attachInterrupt(interrupt pin, ISR, mode)
  PCintPort::attachInterrupt(m1_inB, notifyOnChangeM1B, CHANGE);
  PCintPort::attachInterrupt(m2_inA, notifyOnChangeM2A, CHANGE);
  PCintPort::attachInterrupt(m2_inB, notifyOnChangeM2B, CHANGE);
  Serial.println("Motor Starts!");
}


void loop() {
    md.setM1Speed(300);
}


// Motor Encoder Pin Interrupt Functions
// Assume pattern is 00 01 11 10
// Increase counter when both pins are different

void notifyOnChangeM1A(){
  motor1Aold = digitalRead(m1_inA);
  Serial.print("A : ");
  Serial.println(motor1Aold);
}

void notifyOnChangeM1B(){
   motor1Bnew = digitalRead(m1_inB);
  Serial.print("B : ");
  Serial.println(motor1Bnew);
}

void notifyOnChangeM2A(){
}

void notifyOnChangeM2B(){

}
void left() {
  md.setM1Speed(250);
  md.setM2Speed(50);
  delay(1500);
  md.setM1Speed(0);
  md.setM2Speed(0);
}

void right() {
  md.setM1Speed(40);
  md.setM2Speed(250);
  delay(1500);
  md.setM1Speed(0);
  md.setM2Speed(0);
}

void forward()
{
  md.setM1Speed(200);
  md.setM2Speed(200);
  delay(1500);
  md.setM1Speed(0);
  md.setM2Speed(0);
}

void back()
{
  md.setM1Speed(-200);
  md.setM2Speed(-200);
  delay(1500);
  md.setM1Speed(0);
  md.setM2Speed(0);
}

void communicateWithPi(){
    /* Rpi bluetooth to arduino */
  int index = 0;
  if (Serial.available())
  {
    char message[10] = {Serial.read()};

    switch (message[0])
    {
      //turn left
      case 'l':
        left();
        break;

      //turn right
      case 'r':
        right();
        break;

      //move forward
      case 'f':
        forward();
        break;

      //move backwards
      case 'b':
        back();
        break;

      //do nothing
      default:
        break;
    }
  }
  }
