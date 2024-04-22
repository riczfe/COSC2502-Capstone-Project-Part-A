#include <Arduino.h>
unsigned long time_prev = 0;

void Init_Serial()
{
  Serial.begin(115200);
  while (!Serial)
    ;
}
