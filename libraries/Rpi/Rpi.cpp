#include "Arduino.h"
#include "Rpi.h"

char* Rpi::getRpiMsg(char *inData) {
  memset(inData, 0, sizeof(inData));
  Serial.readBytes(inData, 100);
  return inData;
}
