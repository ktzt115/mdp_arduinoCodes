#include "Arduino.h"
#include "Sensors.h"
#include "math.h"
Sensors::Sensors() {
  _S1Pin = A0;
  _S2Pin = A1;
  _S3Pin = A2;
  _S4Pin = A3;
  _S5Pin = A4;
  _S6Pin = A5;
  _S1Offset = 5.8; //2cm away from the board // 5.6
  _S2Offset = 6; //0.5cm away from the board
  _S3Offset = 5; //2cm away from the board
  _S4Offset = 5.5;//2cm away from the board
  _S5Offset = 9.6; //2cm away from the board
  _S6Offset = 0; //2cm away from the board
  analogReference(DEFAULT);
}

float Sensors::getDistance(int sensorID) {
  float sdistance = 0;
  float offset = 0;
  switch (sensorID) {
    case 1:
      //Set A short range
      sdistance = readFromS1();
      offset = _S1Offset;
      break;
    case 2:
      //Set B short range
      sdistance = readFromS2();
      offset = _S2Offset;
      break;
    case 3:
      //Set C short range
      sdistance = readFromS3();
      offset = _S3Offset;
      break;
    case 4:
      sdistance = readFromS4();
      offset = _S4Offset;
      break;
    case 5:
      sdistance = readFromS5();
      offset = _S5Offset;
      break;
    case 6:
      sdistance = readFromS6();
      offset = _S6Offset;
      break;
    default:
      break;
  }
//  if (sdistance > offset) {
//    sdistance = ceil(sdistance - offset);
//  }
  if (sdistance > offset) {
    sdistance = sdistance - offset;
  }
  return sdistance;
}

float Sensors::readFromS1() {
  //Set F
  int voltage = readAverageValue(_S1Pin, 50, 25);
  float sdistance = 0;
  if (voltage > 80) {
    sdistance = 4418.91 / (voltage - 26.429); //k = 0 equation
    //sdistance = ceil(sdistance);
  }
  return sdistance;

}

float Sensors::readFromS2() {
  //Set C
  int voltage = readAverageValue(_S2Pin, 50, 25);
    //Serial.println(voltage);
  float sdistance = 0;
  if (voltage > 80) {
    sdistance = (6297.23 / (voltage  + 10.302)) - 3; //k = 3 equation
    //sdistance = ceil(sdistance);
      
//    if (sdistance > 50 && sdistance < 57) {
//      sdistance -= 1;
//    } else if (sdistance > 60 && sdistance < 65) {
//      sdistance -= 2;
//    } else if (sdistance > 75 && sdistance < 80) {
//      sdistance += 3;
//    }
  }
  return sdistance;

}
float Sensors::readFromS3() {
  //Set A
  int voltage = readAverageValue(_S3Pin, 50, 25);
  float sdistance = 0;
  if (voltage > 75) {
    sdistance = (6605.01 / (voltage  + 20.57)) - 4; //k = 4 equation
    //sdistance = ceil(sdistance);
  }
  return sdistance;
}

float Sensors::readFromS4() {
  //Set B
  int voltage = readAverageValue(_S4Pin, 50, 25);
  float sdistance = 0;
  if (voltage > 120) {
    sdistance = 4063.38 / (voltage  - 40.63); //k = 0 equation
    //sdistance = ceil(sdistance);
  }
  return sdistance;
}

float Sensors::readFromS5() {
  //Set G
//  int sum = 0;
//  int p = 0;
//  int i;
//  float sdistance, previous;
  int voltage = readAverageValue(_S5Pin, 50, 25);
  float sdistance =  (31645.56 / (voltage + 168.67)) - 26;
//  //float sdistance = (33090.67 / (voltage + 156.62)) - 29; //k = 29 equation
//  previous = (31645.56 / (voltage + 168.67)) - 26; //k = 26 equation
//  for (i = 0; i < 20; i++) {
//    voltage = readAverageValue(_S2Pin, 100, 50);
//    sdistance = (31645.56 / (voltage + 168.67)) - 26;
//    if (sdistance >= (previous - 5) || sdistance <= (previous + 5)) {
//      sum += sdistance;
//      p++;
//      previous = sdistance;
//    }
//  }
//  sdistance = ceil(sum / p);
  return sdistance;

}

//float Sensors::readFromS6() {
//
//  //Set F
//  int voltage = readAverageValue(_S6Pin, 30, 5);
//  float sdistance = 0;
//  if (voltage > 70) {
//    sdistance = 4418.91 / (voltage - 26.429); //k =0 equation
//    //float sdistance = (5341.88 / (voltage  - 11.805)) - 2; //k = 2 equation
//    sdistance = ceil(sdistance);
//  }
//  return sdistance;
//}

float Sensors::readFromS6() {
  //Set D (long range)
  int voltage = readAverageValue(_S6Pin, 50, 25);
  float sdistance = 0;
  if (voltage > 60) {
    sdistance = (31367.63 / (voltage + 148.964)) - 29;
    //sdistance = ceil(sdistance);
  }
  return sdistance;
}

//int Sensors::readAverageValue(int pinNo, int interval, int tolerance) {
//  long sum = 0;
//  int p = 0;
//  int raw = 0;
//  int previous = 0;
//  for (int i = 0; i < interval; i++) {
//    raw = analogRead(pinNo);
//    if ( raw >= previous - 5 || raw <= previous + 5) {
//      sum = sum + raw;
//      p += 1;
//      previous = raw;
//    }
//  }
//  int averageValue = (sum / p ) ;
//  return averageValue;
//}

int Sensors::readAverageValue(int pinNo, int inLen, int outLen) {
  int x[inLen];
  int i;
  int sum = 0;
  int start = (inLen - outLen) / 2;
  int average;
  for (i = 0; i < inLen; i++) {
    x[i] = analogRead(pinNo);
  }
  quickSort(x, 0, inLen - 1);
  for (i = start; i < start + outLen; i++) {
    sum = sum + x[i];
  }
  average = sum / outLen;
  //Serial.println(average);
  return average;
}

void Sensors::quickSort(int x[], int first, int last) {
  int pivot, j, temp, i;
  if (first < last) {
    pivot = first;
    i = first;
    j = last;

    while (i < j) {
      while (x[i] <= x[pivot] && i < last)
        i++;
      while (x[j] > x[pivot])
        j--;
      if (i < j) {
        temp = x[i];
        x[i] = x[j];
        x[j] = temp;
      }
    }

    temp = x[pivot];
    x[pivot] = x[j];
    x[j] = temp;
    quickSort(x, first, j - 1);
    quickSort(x, j + 1, last);

  }
}
