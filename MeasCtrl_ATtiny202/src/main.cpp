#include <Arduino.h>

// PA0(p6) : UPDI       / D5
// PA1(P4) : ADC (AIN1) / D2
// PA2(p5) : PWM (=WO2) / D3
// PA3(p7) : SW         / D4
// PA6(p2) : TXD        / D0
// PA7(p3) : RXD        / D1

void setup() {
  pinMode(0, OUTPUT);
}

void loop() {
  digitalWrite(0, 1); delay(200); digitalWrite(0, 0); delay(200);
}
