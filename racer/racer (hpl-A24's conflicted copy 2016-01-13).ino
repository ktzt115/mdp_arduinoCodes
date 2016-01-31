//import Shapr IR sensor library and motor library located in arduino library folder
#include <SharpIR.h>
#include <DualVNH5019MotorShield.h>
#include "PinChangeInt.h"            // Library for encoders interrupts
#include "Motors.h"
#include "math.h"

//For sensors - pins and other variables
#define sensor1Pin A0
#define sensor2Pin A1
#define sensor3Pin A2
#define sensor4Pin A3
#define sensor5Pin A4
#define shortRange 1080
#define longRange 20150


//Robot settings
bool isDemo = false;

int count = 0;
//SharpIR sharpIR1(sensor1Pin,20,93,shortRange);
DualVNH5019MotorShield md;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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

  if (isDemo) { //For checklist clearing demo
  } else {
    while(count < 9000){
      md.setSpeeds(200,200);
    Serial.print("Encoder 1 count : ");
    Serial.println(encoder1);
    Serial.print("Encoder 2 count : ");
    Serial.println(encoder2);
    count += 1;
    }
    
  }


  //    motor1A = digitalRead(m1_inA);
  //    Serial.print(motor1A);
  //    Serial.print(" : ");
  //    motor1B = digitalRead(m1_inB);
  //    Serial.println(motor1B);

}


// Motor Encoder Pin Interrupt Functions
// Assume pattern is 00 01 11 10
// Increase counter when both pins are different

void notifyOnChangeM1A() {
  if (motor1Bnew ^ motor1Aold) {
    encoder1++;
  } else {
    encoder1--;
  }
  motor1Aold = digitalRead(m1_inA);
}

void notifyOnChangeM1B() {
  motor1Bnew = digitalRead(m1_inB);
  if (motor1Bnew ^ motor1Aold) {
    encoder1++;
  } else {
    encoder1--;
  }
}

//Opposite with motor 1
void notifyOnChangeM2A() {
  if (motor2Bnew ^ motor2Aold) {
    encoder2--;
  } else {
    encoder2++;
  }
  motor2Aold = digitalRead(m2_inA);
}

void notifyOnChangeM2B() {
  motor2Bnew = digitalRead(m2_inB);
  if (motor2Bnew ^ motor2Aold) {
    encoder2--;
  } else {
    encoder2++;
  }
}

//void updateDirection(int directionFlag) {
//  if (directionFlag > 0) {
//    forward = true;
//    backward = false;
//  } else {
//    forward = false;
//    backward = true;
//  }
//}
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

void communicateWithPi() {
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
