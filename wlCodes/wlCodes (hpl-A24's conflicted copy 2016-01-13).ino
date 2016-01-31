//---------------Test for obstacle movement------------------//

//    while (1){
//  if ((calculateDistance (A2) <15) | (calculateDistance (A1 ) <15)|(calculateDistance (A3) <15))
//   {
//     turnRight(90);
//     moveForward(20);
//     turnLeft(90);
//     moveForward(45);
//     turnLeft(90);  
//     moveForward(20);
//     turnRight(90); 
//   }
//  else
//    moveForward(10);  
//  }


//----------------Beyond basics Turning----------------------//
//for (int i=0; i<35; i++){
//  Serial.println("Loop");
//  md.setSpeeds(200,300);
//  delay(100);
//}
//for (int i=0; i<39; i++){
//  Serial.println("Loop");
//  md.setSpeeds(300,200);
//  delay(100);
//}
// md.setBrakes(400,400);
//  
//

///////
#include "Motor.h"                   // Motor pins definition
#include "PinChangeInt.h"            // Library for encoders interrupts
#include "DualVNH5019MotorShield.h"  // Library for the Pololu Dual VNH5019 Motor Driver Shield
#include "math.h"

// Sensor definition (starting from left)
#define sensor_Left          A0    // Left short IR sensor
#define sensor_LeftFront     A1    // Left front short IR sensor
#define sensor_Center        A2    // Center short IR sensor
#define sensor_RightFront    A3    // Right front short IR sensor
#define sensor_Right         A5    // Right long IR sensor

#define pi 3.142


static int counter = 0;

static int alignCounter = 3;
const int checkInterval = 3;

static int olderLeftSensor = 0;
static int oldLeftSensor = 0;
static int currentLeftSensor = 0;

DualVNH5019MotorShield md;  // Declaration of motor driver

////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Up
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){

  md.init();  // Initialization of motor driver
  Serial.begin(115200);  // Initialization of serial port

  // Setting up of interrupt pins
  pinMode(motor1_A, INPUT); 
  digitalWrite(motor1_A, HIGH);
  pinMode(motor1_B, INPUT); 
  digitalWrite(motor1_B, HIGH); 

  pinMode(motor2_A, INPUT); 
  digitalWrite(motor2_A, HIGH);
  pinMode(motor2_B, INPUT); 
  digitalWrite(motor2_B, HIGH); 

  PCintPort::attachInterrupt(motor1_A, Motor1EncoderA, CHANGE);  // attachInterrupt(interrupt pin, ISR, mode)
  PCintPort::attachInterrupt(motor1_B, Motor1EncoderB, CHANGE);
  PCintPort::attachInterrupt(motor2_A, Motor2EncoderA, CHANGE);
  PCintPort::attachInterrupt(motor2_B, Motor2EncoderB, CHANGE);

}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Motor Pin Interrupt Functions
// Assume pattern is 00 01 11 10
// Increase counter when both pins are different
////////////////////////////////////////////////////////////////////////////////////////////////////

void Motor1EncoderA(){
  motor1_Bnew ^ motor1_Aold ? motor1_encoder++ : motor1_encoder--;
  motor1_Aold = digitalRead(motor1_A);
}

void Motor1EncoderB(){
  motor1_Bnew = digitalRead(motor1_B);
  motor1_Bnew ^ motor1_Aold ? motor1_encoder++ : motor1_encoder--;
}

void Motor2EncoderA(){
  motor2_Bnew ^ motor2_Aold ? motor2_encoder++ : motor2_encoder--;
  motor2_Aold = digitalRead(motor2_A);
}

void Motor2EncoderB(){
  motor2_Bnew = digitalRead(motor2_B);
  motor2_Bnew ^ motor2_Aold ? motor2_encoder++ : motor2_encoder--;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Random Test Functions
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Loop
////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(){

//    while(1) {
//      for(int i=0;i<10;i++)
//      {
//      moveForward10(10);
//      }
//      delay(100000);
//    }

//while (1){
//  turnRight(90);
//  delay(1500);
//}
  
  delay(100);
  char command[10] = {NULL};
  int index = 0;
  char newChar = '!';
  int param = 0;
  int terminatorFound = 0;
 
    
  while (1){
    if (Serial.available()){
      newChar = Serial.read();

      if ((newChar != '!' && newChar != '\n' && newChar != '\r') && !terminatorFound){
        command[index] = newChar;
        index++;
      } else if (newChar == '\n') {
        break;
      }
    }  
  }

  index = 1;
  
  while (command[index] != NULL){
    Serial.print(command[index]);
    param = param * 10 + command[index] - 48;
    index++;
  }
 
  
  char movement = command[0];

  delay(100);
  
  switch ( movement ) {
    case 'w':
    {
      if (param == 0){
        moveForward10(10);
        distanceAlignment();
      }
      else {
        moveForward(param);  // for fastest path
      }

        
      break;
    }
    
    case 's':
    {
      if (param == 0){
        moveBackwardSlow(10);
      }
      else {
        moveBackwardSlow(param);
        delay(50);
      }

      break;
    }
      
    case 'a':
    {
      if (param == 0){
        turnLeft(90);
      }
      else {
        turnLeft(param);
        delay(50);
      }

    
      break;
    }
    
    case 'd':
    {
      if (param == 0){
        turnRight(90);
      }
      else {
        turnRight(param);
        delay(50);
       }


      break;
    }
      
    case 'x':
    {      
      break;
    }
    
    case 't': 
    {
      distanceAlignment();
      break;
    }
    
    case 'f':
    {
      if (param == 0){
        moveForward(10);
 
      }
      else {
        moveForward(param);
        //delay(50);
      }

      break;
    }
    
    default:
    {
      //memset(command,0,sizeof(command));
      break;
    }
  
    memset(command,0,sizeof(command));
  }
  
  delay(50);
  exportSensors();
  delay(50);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Motor Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

// Motor 1 negative - count up, motor 2 (right) negative - count down
void moveForward (float distance) {
  
  //Serial.println("Enter moveForward");
  
  int output;
  float target_distance = 0;
  
  motor1_encoder = 0;
  motor2_encoder = 0; 

// 19cm = 6753
// formula = distance / (6*pi) * 2238
// 10cm = 

  // target_distance = (distance / (6*pi) * 2238) - 250; 
  // target_distance = (distance / (6*pi) * 2238);
  target_distance = (distance * 120.5) - 145;  //170 // 195 // 205 overshot // 220 // 250 // 270 correct at first but fall short // 260 fall short // 240 overshot // 250 slightly short
  // target_distance = 1213 - 180; 
  // one grid about 1262 ticks
    
//  if (counterForward == 1) {
//    target_distance = target_distance -20; //17
//    counterForward = 0;
//  }

  
  while (1) {
    
    output = pidControlForward(motor1_encoder, motor2_encoder);
    md.setSpeeds(341, 333-output);  // correct right wheel (set pwm as 150 for default testing) // use slow pid // 300 right // 340 left // 335 right // 338 right // 339 right
    //md.setSpeeds(249, 242);  // use normal pid
    delay(50);
    
    if(motor1_encoder >= target_distance){
      md.setBrakes(400, 400);
      delay(500);
      //md.setBrakes(0, 0);
      
      //counterForward++;
      
      break;
    }
  }
}

int pidControlForward(int motor1_encoder, int motor2_encoder){
  
  /* from pidcontrol250
  int error =0, prev_error, pwmCorrection;
  float integral, derivative, output;
  float Kp = 1.2; // :D (Slightly towards right  // 0.75  // 0.175 (arena in sce)  // 0.3 or 0.25 better (arena swlab2) // 0.8 // need to test 1.25 for accuracy
  float Kd = 0.2;  // 1.65 // 0.2 // 0.15
  float Ki = 0;  // 0.75
  */

  int error =0, prev_error, pwmCorrection;
  float integral, derivative, output;
  float Kp = 0.5; // :D (Slightly towards right  // 0.75  // 0.175 (arena in sce)  // 0.3 or 0.25 better (arena swlab2) // 0.9 original
  float Kd = 0;  // 1.65 // 0.2 // 0.15
  float Ki = 0;  // 0.75

  motor2_encoder = abs(65536 - motor2_encoder);

  error = motor2_encoder - motor1_encoder;
  integral += error;
  derivative = error - prev_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prev_error = error;

  pwmCorrection = output;
  
  return pwmCorrection;

}

void moveForward10 (float distance) {
  
  //Serial.println("Enter moveForward");
  
  int output;
  float target_distance = 0;
  
  motor1_encoder = 0;
  motor2_encoder = 0; 

// 19cm = 6753
// formula = distance / (6*pi) * 2238
// 10cm = 

  // target_distance = (distance / (6*pi) * 2238) - 250; 
  // target_distance = (distance / (6*pi) * 2238);
  //target_distance = (distance * 120.5) - 130; //145 //170 // 195 // 205 overshot // 220 // 250 // 270 correct at first but fall short // 260 fall short // 240 overshot // 250 slightly short
  // target_distance = 1213 - 180; 
  // one grid about 1262 ticks
  target_distance = (distance * 120.5) - 109;
  // error 165 with kp 0.47 very random
    
//  if (counterForward == 1) {
//    target_distance = target_distance -20; //17
//    counterForward = 0;
//  }

  
  while (1) {
    
    md.setSpeeds(341, 333-output);
    output = pidControlForward10(motor1_encoder, motor2_encoder);
      // correct right wheel (set pwm as 150 for default testing) // use slow pid // 300 right // 340 left // 335 right // 338 right // 339 right
    //md.setSpeeds(249, 242);  // use normal pid
    //delay(50);
    
    if(motor1_encoder >= target_distance){
      md.setBrakes(400, 400);
      delay(500);
      //md.setBrakes(0, 0);
      
      //counterForward++;
      
      break;
    }
  }
}

int pidControlForward10(int motor1_encoder, int motor2_encoder){
  
  /* from pidcontrol250
  int error =0, prev_error, pwmCorrection;
  float integral, derivative, output;
  float Kp = 1.2; // :D (Slightly towards right  // 0.75  // 0.175 (arena in sce)  // 0.3 or 0.25 better (arena swlab2) // 0.8 // need to test 1.25 for accuracy
  float Kd = 0.2;  // 1.65 // 0.2 // 0.15
  float Ki = 0;  // 0.75
  */

  int error =0, prev_error, pwmCorrection;
  float integral, derivative, output;
  float Kp = 0.3; // :D (Slightly towards right  // 0.75  // 0.175 (arena in sce)  // 0.3 or 0.25 better (arena swlab2) // 0.9 original
  //kp 0.87 with error 160 slightly jerking
  float Kd = 0;  // 1.65 // 0.2 // 0.15
  float Ki = 0;  // 0.75

  motor2_encoder = abs(65536 - motor2_encoder);

  error = motor2_encoder - motor1_encoder;
  integral += error;
  derivative = error - prev_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prev_error = error;

  pwmCorrection = output;
  
  return pwmCorrection;

}



void moveForwardSlow(float distance) {
  
  int output;
  int target_distance = 0;
  
  motor1_encoder=0;
  motor2_encoder=0; 


  target_distance = (distance * 126) - 130; 

  
  while (1) {
    
    output = pidControlForwardSlow(motor1_encoder, motor2_encoder);
    md.setSpeeds(150, 150-output);
    delay(50);
    
    if(motor1_encoder >= target_distance){
      md.setBrakes(400, 400);
      delay(50);
      md.setBrakes(0, 0);
     
      break;
    }
  }
}

void moveForwardAlign(float distance) {
  
  int output;
  int target_distance = 0;
  
  motor1_encoder=0;
  motor2_encoder=0; 

 target_distance = (distance * 10) ; 

  
  while (1) {
    
    output = pidControlForwardSlow(motor1_encoder, motor2_encoder);
    md.setSpeeds(250, 250-output);//150
    delay(50);
    
    if(motor1_encoder >= target_distance){
      md.setBrakes(400, 400);
      delay(50);
      md.setBrakes(0, 0);

      break;
    }
  }
}


void moveBackwardSlow(float distance) {
  
  int output;
  int target_distance = 0;
  
  motor1_encoder=0;
  motor2_encoder=0; 


  target_distance = (distance * 126) - 130; 
   
  while (1) {
    
    output = pidControlBackwardSlow(motor1_encoder, motor2_encoder);
    md.setSpeeds(-150, -(150-output));
    delay(50);
    
    if(motor1_encoder >= target_distance){
      md.setBrakes(400, 400);
      delay(50);
      md.setBrakes(0, 0);
      
      break;
    }
  }
}


void moveBackwardAlign(float distance) {
  
  int output;
  int target_distance = 0;
  
  motor1_encoder=0;
  motor2_encoder=0; 

 target_distance = (distance * 10); 
  
  while (1) {
    
    output = pidControlBackwardSlow(motor1_encoder, motor2_encoder);
    md.setSpeeds(-250, -(250-output));// 150 then 200
    delay(50);
    
    if(motor1_encoder >= target_distance){
      md.setBrakes(400, 400);
      delay(50);
      md.setBrakes(0, 0);
      
      //counterForward++;
      
      break;
    }
  }
}
// Motor 1 (left) postive - count up, motor 2 (right) negative - count up
// Right wheel = 2251 (1 full revolution)
// 360 turn = 3 x 2251 = 6753

void turnRight (float angle) {
   
  int output, error = 165;
  float target;
  int rev_right = 2251 * 3;
  int LeftPosition,RightPosition;
  float test_angle;
  
  motor1_encoder=0;
  motor2_encoder=0; 
  

  target = (angle/360) * 6753 - 127;
  
  
  if (target<0){
    target =10;
  }
  
  while (1) {

    
    output = pidTurnRight(motor1_encoder, motor2_encoder);
    md.setSpeeds(249, -(243-output));
    

    if(motor1_encoder >= target){
      md.setBrakes(400, 400);

      delay(50);

      break;
    }
  }
}


void turnLeft(float angle) {
  
  int output;
  int error = 146; // 160 for lab floor  //165
  float target;
  int rev_left = 2251 * 3; // 2251
  int LeftPosition,RightPosition;
  
  motor1_encoder=0;
  motor2_encoder=0; 
 
  target = (angle/360 *  6753)- 130;// accurate for 26th March
 
  if (target<0){
    target =20;

  }
  while (1) {
    
    LeftPosition =  abs(65536-motor1_encoder); 
    RightPosition =  abs(65536-motor2_encoder); 
    
    output = pidTurnRight(LeftPosition, RightPosition);

    md.setSpeeds(-249, (242-output));  //249, 243-output (before 26th march )    
    

    
    if(LeftPosition >= target){

      md.setBrakes(400, 400);
      delay(100);
      md.setBrakes(0, 0);

      
      counter++;
  
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// PID Control Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

int pidControlForwardSlow(int motor1_encoder, int motor2_encoder){  // for speed 200

  int error =0, prev_error, pwmCorrection;
  float integral, derivative, output;
// lesser slanted left, higher slanted right
  float Kp = 0.778; // :D (Slightly towards right  // 0.75  // 0.175 (arena in sce)  // 0.3 or 0.25 better (arena swlab2) // 0.8 // need to test 1.25 for accuracy
  float Kd = 0.2;  // 1.65 // 0.2 // 0.15
  float Ki = 0;  // 0.75

  motor2_encoder = abs(65536 - motor2_encoder);

  error = motor2_encoder - motor1_encoder;
  integral += error;
  derivative = error - prev_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prev_error = error;

  pwmCorrection = output;
  
  return pwmCorrection;

}

int pidControlBackwardSlow(int motor1_encoder, int motor2_encoder){  // for speed 200

  int error =0, prev_error, pwmCorrection;
  float integral, derivative, output;
// lesser slanted left, higher slanted right
  float Kp = 0.778; // :D (Slightly towards right  // 0.75  // 0.175 (arena in sce)  // 0.3 or 0.25 better (arena swlab2) // 0.8 // need to test 1.25 for accuracy
  float Kd = 0.2;  // 1.65 // 0.2 // 0.15
  float Ki = 0;  // 0.75

  motor2_encoder = abs(65536 - motor2_encoder);

  error = motor2_encoder - motor1_encoder;
  integral += error;
  derivative = error - prev_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prev_error = error;

  pwmCorrection = output;
  
  return pwmCorrection;

}

int pidTest(int motor1_encoder, int motor2_encoder){
  
  int error, prev_error, pwmCorrection;
  float integral, derivative, output;
  float Kp = 1.25;  // 0.75  //0.95
  float Kd = 1.5;  // 1.65 // 0.15 // larger kd = lesser diff
  float Ki = 0;  // 0.75
  
  error = ((motor2_encoder/2) - motor1_encoder);
  integral += error;
  derivative = error - prev_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prev_error = error;

  pwmCorrection = output;
  return pwmCorrection;

}

int pidTurnRight(int motor1_encoder, int motor2_encoder){

  int error, prev_error, pwmCorrection;
  float integral, derivative, output;
  
  float Kp = 1;  // 0.75 // 0.45 // 0.85 // 1.35
  float Kd = 0;  // 1.65 //0.25(newer)
  float Ki = 0;  // 0.75

  error = motor2_encoder - motor1_encoder;
  integral += error;
  derivative = error - prev_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prev_error = error;

  pwmCorrection = output;
  return pwmCorrection;

}

int pidTurnLeft(int motor1_encoder, int motor2_encoder){

  int error, prev_error, pwmCorrection;
  float integral, derivative, output;
  float Kp = 1.5;  // 0.75  //0.95
  float Kd = 0;  // 1.65 // 0.15 // larger kd = lesser diff
  float Ki = 0;  // 0.75
  
  error = motor2_encoder - motor1_encoder;
  integral += error;
  derivative = error - prev_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prev_error = error;

  pwmCorrection = output;
  return pwmCorrection;

}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

// Read value from sensor
int SensorRead(int sensorPin) {  
  return analogRead(sensorPin);
}

// Print value from sensor
void SensorPrint(int sensorValue) {   
  Serial.println(sensorValue);
}

int calculateDistance(int sensorIndex){

  int adc = averageFeedback(100,50,sensorIndex);  // 30,15
  float distance, distance1;

  switch (sensorIndex){
    case A0: // Left
    distance = 6088.0 / (adc  + 7)-1;//most accurate
    break;
    
    case A1:  // Left Front 
    distance = 6088.0 / (adc + 7) - 2; //most accurate
      
    break;
    
    case A2:   // Center
    distance = 6088.0 / (adc +7) - 2;//most accurate
    break;
    
    case A3:   // Right Front
    distance = 6170 / (adc +7) - 2; // -1
 
    break;
    
    case A5:   // Right (Long)
    distance = 5790 / (adc +7) - 2; // -1 // 4100  // 5770
    break;
  }
  return distance;
}

int averageFeedback(int in, int out, int pin){
  int x[in];
  int i;
  int sum = 0;
  int start = (in - out)/2;
  int average;
  for(i=0;i<in;i++){
    x[i] = SensorRead(pin);
  }
  quickSort(x, 0, in-1);
  for(i = start; i < start+out; i++){
    sum = sum + x[i];
  }
  average = sum/out;
  return average;
}

void quickSort(int x[],int first,int last){
  int pivot,j,temp,i;
  if(first<last){
    pivot=first;
    i=first;
    j=last;

    while(i<j){
      while(x[i]<=x[pivot]&&i<last)
        i++;
      while(x[j]>x[pivot])
        j--;
      if(i<j){
        temp=x[i];
        x[i]=x[j];
        x[j]=temp;
      }
    }

    temp=x[pivot];
    x[pivot]=x[j];
    x[j]=temp;
    quickSort(x,first,j-1);
    quickSort(x,j+1,last);

  }
}

void exportSensors(){
   Serial.print("!");
   //Serial.println("Print Left A0 distance");
   Serial.print(calculateDistance(A0));
   Serial.print(",");
   //Serial.println("Print LeftFront A1 distance");
   Serial.print(calculateDistance(A1));  
   Serial.print(",");
   //Serial.println("Print Front A2 distance");
   Serial.print(calculateDistance(A2));
   Serial.print(",");
  //Serial.println("Print RightFront A3 distance");
  Serial.print(calculateDistance(A3));
  Serial.print(",");
  //Serial.println("Print RightLong A5 distance");
  Serial.println(calculateDistance(A5));
  
  // delay(100);
  // Serial.println("|");
}

void exportSensorsFloat(){
   //Serial.println("Print Left A0 distance");
   Serial.print(calculateDistanceFloat(A0));
   Serial.print(",");
   //Serial.println("Print LeftFront A1 distance");
   Serial.print(calculateDistanceFloat(A1));  
   Serial.print(",");
   //Serial.println("Print Front A2 distance");
   Serial.print(calculateDistanceFloat(A2));
   Serial.print(",");
  //Serial.println("Print RightFront A3 distance");
  Serial.print(calculateDistanceFloat(A3));
  Serial.print(",");
  //Serial.println("Print RightLong A5 distance");
  Serial.println(calculateDistanceFloat(A5));
  
  // delay(100);
  // Serial.println("|");
}

float calculateDistanceFloat(int sensorIndex){
  //int numLoop = 5;
  int adc = averageFeedback(100,50,sensorIndex);  // 30,15

  float distance, distance1;

  switch(sensorIndex){
    case A0: // Left

    distance = 6088.0 / (adc  + 7)-1; // at swl2 =1 , earlier = 2
    break;
    
    case A1:  // Left Front 

    distance = 6088.0 / (adc + 7) - 2; //most accurate
    break;
    
    case A2:   // Center
    distance = 6088.0 / (adc +7) - 2;//most accurate

    break;
    
    case A3:   // Right Front
    distance = 6170.0 / (adc +7) - 2; // -1
    
    break;
    
    case A5:   // Right (Long)
    distance = 5790.0 / (adc +7) - 2;

    break;
  }
  return distance;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Alignment Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void distanceAlignment() {
  
  int M = (calculateDistanceFloat(A2));  
  int L = (calculateDistanceFloat(A1)); 
  int R = (calculateDistanceFloat(A3)); 
  
  boolean Fflag= true;
  boolean Sflag= true;
  alignCounter = alignCounter - 1;

  if (alignCounter < 0) {
    alignCounter = 0;
  }
  if (alignCounter == 0) {
    
    if (M < 13) {
      if ((R<13) && (L<13)){
      Fflag = frontAlignment();
      }
    }
      Sflag= sideAlignment();
    if ((Sflag ==false)|(Fflag = false)){
      alignCounter=0;
    }
    else 
      alignCounter = checkInterval;
  }
}

void angleAlignment() {

  float dif = 0;
  int breakCount = 0;
     
    float L = (calculateDistanceFloat(A1));
    float M = (calculateDistanceFloat(A2));
    float R = (calculateDistanceFloat(A3));

  
  if ((M < 14) && (M > 5)) {
    dif = L - R;
    delay(50);
    if ((abs(dif)<5)){ 
      if ((L-M) < 5){
        if ((R-M) < 5) {
        //both sides has obstacles // 5
    
    
    if (dif > 0) {// too much left
      while (dif > 0.5) { //calibrate to an accuracy of 0.5
        L = (calculateDistanceFloat(A1));
        M = (calculateDistanceFloat(A2));
        R = (calculateDistanceFloat(A3));
        dif = L - R;

        
        if (dif >=  0.5) {
          turnRight(1);

        } 
        else {
          breakCount++;
          if (breakCount == 5) {
            break;
          }
        }
      }
    }
    else
    {
      while (dif < - 0.5) { //too much right
        L = (calculateDistanceFloat(A1));
        M = (calculateDistanceFloat(A2));
        R = (calculateDistanceFloat(A3));
        dif = L - R;
        
        if (dif <= - 0.5) {
          turnLeft(1);

        } 
        else {
          breakCount++;
          if (breakCount == 5) {
            break;
          }
        }
      }
    }
  }
  } 
  }
  }
}

boolean frontAlignment() {
  
  int safeDistance = 12;//14
  int actualDistance = 10;
  float M = (calculateDistanceFloat(A2));
  boolean frontflag = false;
  
  angleAlignment();
  
  while (M < (safeDistance)) {
    frontflag = true;
    moveBackwardAlign(1);
    M = (calculateDistanceFloat(A2));
  }
  
  while (M > (actualDistance)) {
    frontflag = true;
    moveForwardAlign(1);
    M = (calculateDistanceFloat(A2));
  }
  
  // straighten
  angleAlignment();
  return frontflag;
}

boolean sideAlignment() {
  
  int safeDistance = 12;  // 12
  int actualDistance = 10;  // 13
  float LL = (calculateDistance(A0));
  float RR = (calculateDistanceFloat(A5));
  float MM = (calculateDistanceFloat(A2));
  boolean sideflag = false;
  
  // goal = 11 cm

  if ((LL >= 7) && (LL <= 15)) {
    sideflag = true;

    turnLeft(90);
    delay(50);
    

    angleAlignment();
    
    MM = (calculateDistanceFloat(A2));
    
    // maintain distance from wall (move backward)
    while (MM < safeDistance) {
     
      moveBackwardAlign(1);
      MM = (calculateDistanceFloat(A2));
    }
    
    // maintain distance from wall (move forward)
    while (MM > actualDistance) {
      
      moveForwardAlign(1);
      MM = (calculateDistanceFloat(A2));
    }
    
    // straighten
    angleAlignment();
    turnRight(90);
  }
  return sideflag;
}



void moveForwardFast(float distance) {  // Need to calibrate speed

  int output;
  int target_distance = 0;
  
  motor1_encoder = 0;
  motor2_encoder = 0; 

  target_distance = (distance * 126) - 455;  //170 // 195 // 205 overshot // 220 // 250 // 270 correct at first but fall short // 260 fall short // 240 overshot // 250 slightly short // 245

  
  while (1) {
    
    output = pidControlForwardFast(motor1_encoder, motor2_encoder);
    md.setSpeeds(350, 339-output);  // correct right wheel (set pwm as 150 for default testing) // use slow pid // 300 right // 340 left // 335 right // 338 right // 339 right
    delay(50);
    
    if(motor1_encoder >= target_distance){
      md.setBrakes(400, 400);
      
      break;
    }
  }
}

int pidControlForwardFast(int motor1_encoder, int motor2_encoder){


  int error =0, prev_error, pwmCorrection;
  float integral, derivative, output;
  float Kp = 0.9; // :D (Slightly towards right  // 0.75  // 0.175 (arena in sce)  // 0.3 or 0.25 better (arena swlab2) // 0.9 original
  float Kd = 0.2;  // 1.65 // 0.2 // 0.15
  float Ki = 0;  // 0.75

  motor2_encoder = abs(65536 - motor2_encoder);

  error = motor2_encoder - motor1_encoder;
  integral += error;
  derivative = error - prev_error;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prev_error = error;

  pwmCorrection = output;
  
  return pwmCorrection;

}
