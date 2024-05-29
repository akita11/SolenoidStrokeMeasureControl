#include <Arduino.h>

#define PIN_FLAG1 2
#define PIN_FLAG2 3
#define PIN_ADC A0
#define PIN_PWM 9
#define PIN_SW 4
#define PIN_LED 13

uint32_t v0s, v1s;
uint16_t v0, v1;
uint8_t n = 0;
//uint8_t N = 128;
//uint8_t N = 1;
uint8_t N = 4;

uint16_t Ton = 1000;
uint16_t Delay = 100;

void setup()
{
	pinMode(PIN_PWM, OUTPUT);
	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, LOW);
	pinMode(PIN_FLAG2, OUTPUT); digitalWrite(PIN_FLAG2, LOW);
	pinMode(PIN_SW, INPUT_PULLUP);
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);
	Serial.begin(115200);
	// setup Timer1
	TCCR1A = _BV(COM1A1) | _BV(WGM11);			  // Ch.A: non-inverting, WGM11=1 (mode14, Fast PWM)
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // WGM13:12=11, CS11=1 (16MHz/8 = 2MHz)

	ICR1 = 19999;		 // 2MHz/20000=100Hz(10ms) / TOP
	OCR1A = Ton * 2 - 1; // PWM Duty Cycle
	OCR1B = 999;		 // 2MHz/1000=2kHz / 0.5ms
//	OCR1B = 1799;		 // 2MHz/1800=1.111kHz / 0.9ms

	// enable interrupts
	TIMSK1 |= _BV(TOIE1);  // enable Timer1 OVF interrupt (=PWM ON)
	TIMSK1 |= _BV(OCIE1B); // enable Timer1 COMPB interrupt
    //TIMSK1 |= _BV(OCIE1A); // enable Timer1 COMPA interrupt (=PWM OFF)
	sei();				   // enable global interrupt
}

// Timer1 のオーバーフロー割り込み (=PWM ON)
ISR(TIMER1_OVF_vect)
{
	delayMicroseconds(Delay);
//	PORTD |= _BV(PD2);
	v0s += analogRead(PIN_ADC);
//	PORTD &= ~(_BV(PD2));
}

// Timer1 のCompareMatchB割り込み
ISR(TIMER1_COMPB_vect)
{
//	PORTD |= _BV(PD3);
	v1s += analogRead(PIN_ADC);
//	PORTD &= ~(_BV(PD3));
	n++;
	if (n == N)
	{
		n = 0;
		v0 = v0s / N; v1 = v1s / N;
//		Serial.print(v0); Serial.print(","); Serial.print(v1); Serial.print(","); Serial.println(v1 - v0);
		v0s = 0; v1s = 0;
	}
}

// Timer1 のCompareMatchA割り込み(=PWM OFF)
ISR(TIMER1_COMPA_vect)
{
	PORTD |= _BV(PD3);
	v1 += analogRead(PIN_ADC);
	PORTD &= ~(_BV(PD3));
	n++;
	if (n == N)
	{
		n = 0;
		v0 /= N;
		v1 /= N;
/*
		Serial.print(v0 * 5.0 / 1024.0);
		Serial.print(",");
		Serial.println(v1 * 5.0 / 1024.0);
*/
	}
}

uint8_t st = 0;

uint16_t tm = 0;

uint8_t fMeasure = 0;

void loop()
{
	if (digitalRead(PIN_SW) == LOW)
	{
/*
		st = (st + 1) % 9;
		Ton = 1000 + 1000 * st;
		OCR1A = Ton * 2 - 1; // PWM Duty Cycle
		while (digitalRead(PIN_SW) == LOW) delay(10);
		tm = 0;
*/
		fMeasure = 1 - fMeasure;
	}
	
	if (fMeasure == 1){
		digitalWrite(13, HIGH);
		for (Ton = 1000; Ton <= 9000; Ton += 1000){
			Serial.print(Ton);
			uint16_t v1s = 0;
			OCR1A = Ton * 2 - 1; // PWM Duty Cycle
			delay(2000);
//			Serial.print(','); Serial.print(v0);
//			Serial.print(','); Serial.print(v1);
//			Serial.print(','); Serial.println(v1-v0);
			for (Delay = 100; Delay < 800; Delay += 200){
				OCR1A = Ton * 2 - 1; // PWM Duty Cycle
				OCR1B = Ton * 2 - 201;
				delay(1000);
				Serial.print(','); Serial.print(v0);
				v1s += v1;
			}
			Serial.print(','); Serial.println(v1s / 4);
		}
		fMeasure = 0;
		OCR1A = 2000 - 1; // PWM Duty Cycle
		OCR1B = 999;
		digitalWrite(13, LOW);
	}

}

