/*
Motor Header File
(For variable declarations)
*/

#include <Arduino.h>

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


//For reading encoder values and
//controlling motors
static int motor1Aold = 0;
static int motor1Bnew = 0;
static int motor2Aold = 0;
static int motor2Bnew = 0;

static unsigned int encoder1 = 0;
static unsigned int encoder2 = 0;

//flag to know whether the motor is moving forward or backward
//static bool forward = false;
//static bool backward = false;


const float wheelRadius = 3.00; //Motor Wheel Radius
