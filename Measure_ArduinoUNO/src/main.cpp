#include <Arduino.h>

#define PIN_FLAG1 2
#define PIN_FLAG2 3
#define PIN_ADC   A0
#define PIN_PWM   9
#define PIN_SW    4
#define PIN_LED   13

uint32_t v0, v1;
uint8_t n = 0;
//uint8_t N = 128;
uint8_t N = 32;

uint16_t Ton = 1000;
uint16_t Delay = 100;

void setup()
{
	pinMode(PIN_PWM, OUTPUT);
	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, LOW);
	pinMode(PIN_FLAG2, OUTPUT); digitalWrite(PIN_FLAG2, LOW);
	pinMode(PIN_SW, INPUT_PULLUP);
	pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
	Serial.begin(115200);
	// setup Timer1
	TCCR1A = _BV(COM1A1) | _BV(WGM11);			  // Ch.A: non-inverting, WGM11=1 (mode14, Fast PWM)
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // WGM13:12=11, CS11=1 (16MHz/8 = 2MHz)

	ICR1 = 19999; // 2MHz/20000=100Hz(10ms) / TOP
	OCR1A = Ton * 2 - 1; // PWM Duty Cycle
	OCR1B = 999; // 2MHz/1000=2kHz / 0.5ms

	// enable interrupts
	TIMSK1 |= _BV(TOIE1);  // enable Timer1 OVF interrupt (=PWM ON)
	TIMSK1 |= _BV(OCIE1B); // enable Timer1 COMPB interrupt 
//	TIMSK1 |= _BV(OCIE1A); // enable Timer1 COMPA interrupt (=PWM OFF)
	sei(); // enable global interrupt
}

// Timer1 のオーバーフロー割り込み (=PWM ON)
ISR(TIMER1_OVF_vect)
{
	delayMicroseconds(Delay);
	PORTD |= _BV(PD2);
	v0 += analogRead(PIN_ADC);
	PORTD &= ~(_BV(PD2));
}

// Timer1 のCompareMatchB割り込み
ISR(TIMER1_COMPB_vect)
{
	PORTD |= _BV(PD3);
	v1 += analogRead(PIN_ADC);
	PORTD &= ~(_BV(PD3));
	n++;
	if (n == N) {
		n = 0;
		v0 /= N;
		v1 /= N;
//		Serial.print(v0*5.0/1024.0); Serial.print(","); Serial.println(v1*5.0/1024.0); 
		Serial.print(v0); Serial.print(","); Serial.print(v1); Serial.print(","); Serial.println(v1 - v0);
	}
}

// Timer1 のCompareMatchA割り込み(=PWM OFF)
ISR(TIMER1_COMPA_vect)
{
	PORTD |= _BV(PD3);
	v1 += analogRead(PIN_ADC);
	PORTD &= ~(_BV(PD3));
	n++;
	if (n == N) {
		n = 0;
		v0 /= N;
		v1 /= N;
		Serial.print(v0*5.0/1024.0); Serial.print(","); Serial.println(v1*5.0/1024.0); 
	}
}

uint8_t st = 0;
void loop()
{
	if (digitalRead(PIN_SW) == LOW) {
/*
		st = (st + 1) % 18;
		switch(st){
			case 0 : Delay = 50; Ton = 1000; break;
			case 2 : Delay = 50; Ton = 2000; break;
			case 4 : Delay = 50; Ton = 3000; break;
			case 6 : Delay = 50; Ton = 4000; break;
			case 8 : Delay = 50; Ton = 5000; break;
			case 10 : Delay = 50; Ton = 6000; break;
			case 12 : Delay = 50; Ton = 7000; break;
			case 14 : Delay = 50; Ton = 8000; break;
			case 16 : Delay = 50; Ton = 9000; break;
			case 1 : Delay = 100; Ton = 1000; break;
			case 3 : Delay = 100; Ton = 2000; break;
			case 5 : Delay = 100; Ton = 3000; break;
			case 7 : Delay = 100; Ton = 4000; break;
			case 9 : Delay = 100; Ton = 5000; break;
			case 11 : Delay = 100; Ton = 6000; break;
			case 13 : Delay = 100; Ton = 7000; break;
			case 15 : Delay = 100; Ton = 8000; break;
			case 17 : Delay = 100; Ton = 9000; break;
		}
		if (st % 2 == 1) digitalWrite(PIN_LED, HIGH);
		else digitalWrite(PIN_LED, LOW);
*/
		st = (st + 1) % 9;
		switch(st){
			case 0 : Delay = 100; Ton = 1000; break;
			case 1 : Delay = 100; Ton = 2000; break;
			case 2 : Delay = 100; Ton = 3000; break;
			case 3 : Delay = 100; Ton = 4000; break;
			case 4 : Delay = 100; Ton = 5000; break;
			case 5 : Delay = 100; Ton = 6000; break;
			case 6 : Delay = 100; Ton = 7000; break;
			case 7 : Delay = 100; Ton = 8000; break;
			case 8 : Delay = 100; Ton = 9000; break;
		}
		OCR1A = Ton * 2 - 1; // PWM Duty Cycle
		while(digitalRead(PIN_SW) == LOW) delay(10);
	}
}
