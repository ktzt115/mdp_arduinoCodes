/*
Sensors Header File
(For variable declarations)
*/
#include <Arduino.h>
//For sensors - pins and other variables
#define sensor1Pin A0
#define sensor2Pin A1
#define sensor3Pin A2
#define sensor4Pin A3
#define sensor5Pin A5

class Sensors
{
  public:  
    // CONSTRUCTORS
    Sensors(); // Default pin selection.
    
    // PUBLIC METHODS
    //Read sensor value and return distance
    float getDistance(int sensorID);
    float readFromS1();
    float readFromS2();
    float readFromS3();
    float readFromS4();
    float readFromS5();
    float readFromS6();
    
  private:
    int _S1Pin;
    int _S2Pin;
    int _S3Pin;
    int _S4Pin;
    int _S5Pin;
    int _S6Pin;
    float _S1Offset;
    float _S2Offset;
    float _S3Offset;
    float _S4Offset;
    float _S5Offset;
    float _S6Offset;
    int readAverageValue(int pinNo,int inLen, int outLen);
    void quickSort(int x[],int first, int last);
};
