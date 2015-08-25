#include <SharpIR.h>
#include <DualVNH5019MotorShield.h>
#define sensor1Pin A0
#define sensor2Pin A1
#define sensor3Pin A2
#define sensor4Pin A3
#define sensor5Pin A4
#define shortRange 1080
#define longRange 20150


//Rotary Encoder
#define motor1_A 11
#define motor1_B 5
#define motor2_A 3
#define motor2_B 13

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
  Serial.begin(9600);
  // pinMode(sensor1Pin,INPUT);
  md.init();

  pinMode(motor1_A, INPUT);
  pinMode(motor1_B, INPUT);
  pinMode(motor2_A, INPUT);
  pinMode(motor2_B, INPUT);

  digitalWrite(motor1_A, HIGH);
  digitalWrite(motor1_B, HIGH);
  digitalWrite(motor2_A, HIGH);
  digitalWrite(motor2_B, HIGH);

  
}


void loop() {
  //For Sensor Reading


  //  unsigned long startTime=millis();  // takes the time before the loop on the library begins (to be removed later)
  // //Sensor 1 Reading
  //  int dis=sharpIR1.distance();  // this returns the distance to the object you're measuring
  //
  //  Serial.print("Mean distance: ");  // returns it to the serial monitor
  //  Serial.println(dis);
  //
  //  unsigned long endTime=millis()-startTime;  // the following gives you the time taken to get the measurement
  //  Serial.print("Time taken (ms): ");
  //  Serial.println(endTime);


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

  //delay(10000);
  //md.setM1Speed(250);
  //forward();

  
  Serial.print("M1 A: ");
  Serial.println(analogRead(motor1_A));
  Serial.print("M2 A: ");
  Serial.println(analogRead(motor2_A));

  md.setM1Speed(209);
  md.setM2Speed(207);
  
  delay(1000);

  md.setM1Brake(400);
  md.setM2Brake(400);

  delay(1000);
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
