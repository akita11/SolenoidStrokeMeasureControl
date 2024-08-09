#include <Arduino.h>

// PA0 : UPDI       / D5
// PA1 : ADC (AIN1) / D2
// PA2 : PWM (=WO2) / D3
// PA3 : SW         / D4
// PA6 : TXD        / D0
// PA7 : RXD        / D1

void setup() {
  pinMode(0, OUTPUT);
}

void loop() {
  digitalWrite(0, 1); delay(500); digitalWrite(0, 0); delay(500);
}
