#include <Arduino.h>
#include <SPI.h>

#define PIN_FLAG1 2
#define PIN_FLAG2 3
#define PIN_ADC A0
#define PIN_PWM 9
#define PIN_SW 4
#define PIN_LED 13

#define PIN_CALIPER 8

#define MEASURE_PARAM

// SCK=13(p3), MOSI=11(p2), GND(p1), /REQ(p5)

uint8_t n = 0;
uint8_t N = 8; 
uint16_t v0s, v1s;
uint16_t v0, v1;

uint16_t Ton = 1000;
uint16_t Delay = 100;

uint8_t spiBuf[32];
uint8_t p_spiBuf = 0;

#define getLowerChar(c) ('0' + (c & 0x0f))
#define getHigherChar(c) ('0' + (c >> 4))

#ifdef MEASURE_PARAM
#else
ISR(SPI_STC_vect)
{
	spiBuf[p_spiBuf] = SPDR;
	p_spiBuf++;
	if (p_spiBuf == 32) p_spiBuf = 0;
	digitalWrite(PIN_FLAG1, 1 - digitalRead(PIN_FLAG1));

/*
	char datBuf[8];
	if (p_spiBuf > 6){
		sprintf(datBuf, "%c%c%c%c%c.%c%c",
			getLowerChar(spiBuf[2]), getHigherChar(spiBuf[2]),		
			getLowerChar(spiBuf[3]), getHigherChar(spiBuf[3]),		
			getLowerChar(spiBuf[4]), getHigherChar(spiBuf[4]),		
			getLowerChar(spiBuf[5]));		
		s = atof(datBuf);
		p_spiBuf = 0;
	}
*/
//	if (p_spiBuf == 7){
//		p_spiBuf = 0;
//		SPCR &= ~bit(SPE); SPCR |= bit(SPE);
//	}
}
#endif

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
//	OCR1B = 999;		 // 0.5us(2MHz)*1000 = 0.5ms
	OCR1B = 1399;		 // 0.5us(2MHz)*1400 = 0.7ms

	// enable interrupts
	TIMSK1 |= _BV(TOIE1);  // enable Timer1 OVF interrupt (=PWM ON)
	TIMSK1 |= _BV(OCIE1B); // enable Timer1 COMPB interrupt
    //TIMSK1 |= _BV(OCIE1A); // enable Timer1 COMPA interrupt (=PWM OFF)
	sei();				   // enable global interrupt

#ifdef MEASURE_PARAM
#else
	pinMode(PIN_CALIPER, OUTPUT); digitalWrite(PIN_CALIPER, HIGH);
	// setup SPI
	SPCR |= bit(SPE); // enabel SPI
	SPCR |= bit(DORD); // LSB first
	SPCR |= bit(CPOL); // clock polarity
	pinMode(MISO, OUTPUT);
	pinMode(SCK, INPUT_PULLUP);
	pinMode(MOSI, INPUT_PULLUP);
//	pinMode(SS, INPUT_PULLUP);
	SPI.attachInterrupt();
#endif
}

/*
// for multiple sampling
uint16_t v0s = 0, v1s = 0, v2s = 0, v3s = 0, v4s = 0;
uint16_t v0, v1, v2, v3, v4;
*/


// Timer1 のオーバーフロー割り込み (=PWM ON)
ISR(TIMER1_OVF_vect)
{
	delayMicroseconds(Delay);
//	PORTD |= _BV(PD2);
	v0s += analogRead(PIN_ADC);
//	PORTD &= ~(_BV(PD2));
/*
	delayMicroseconds(100);
	PORTD |= _BV(PD2);
	v0s += analogRead(PIN_ADC);
	delayMicroseconds(100);
	PORTD &= ~(_BV(PD2));
	v1s += analogRead(PIN_ADC);
	delayMicroseconds(100);
	PORTD |= _BV(PD2);
	v2s += analogRead(PIN_ADC);
	delayMicroseconds(100);
	PORTD &= ~(_BV(PD2));
	v3s += analogRead(PIN_ADC);
	delayMicroseconds(100);
	PORTD |= _BV(PD2);
	v4s += analogRead(PIN_ADC);
	PORTD &= ~(_BV(PD2));
	n++;
	if (n == N){
		n = 0;
		v0 = v0s / N;
		v1 = v1s / N;
		v2 = v2s / N;
		v3 = v3s / N;
		v4 = v4s / N;
	}
*/
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
		v0 = v0s / N; v0s = 0;
		v1 = v1s / N; v1s = 0;
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
#ifdef MEASURE_PARAM
#else		
		if (fMeasure == 1){
			SPCR &= ~bit(SPE); delay(1); SPCR |= bit(SPE);
		}
		digitalWrite(PIN_CALIPER, 1 - fMeasure);
#endif
		delay(500);
	}
	
	if (fMeasure == 1){
#ifdef MEASURE_PARAM
		int v[9][5];
		digitalWrite(13, HIGH);
		uint8_t iTon, iDelay;;
/*
		for (iDelay = 0; iDelay < 5; iDelay++){
			Delay = 100 + iDelay * 200;
			for (iTon = 0; iTon < 9; iTon++){
				Ton = iTon * 1000 + 1000;
				OCR1A = Ton * 2 - 1; // PWM Duty Cycle
				OCR1B = Ton * 2 - 201;
				v0s = 0; v1s = 0; n = 0;
				delay(2000);
				v[iTon][iDelay] = v0;
			}
		}
		for (iTon = 0; iTon < 9; iTon++){
			Serial.print(iTon+1);
			for (iDelay = 0; iDelay < 5; iDelay++){
				 Serial.print(","); Serial.print(v[iTon][iDelay]);
			}
			Serial.println();
		}
*/
/*
		for (iTon = 0; iTon < 9; iTon++){
			Ton = iTon * 1000 + 1000;
			OCR1A = Ton * 2 - 1; // PWM Duty Cycle
//			for (iDelay = 0; iDelay < 5; iDelay++){
//				OCR1B = 199 + iDelay * 400; // 0.5us*X [ms], 100us=200, 300us=600, 500us=1000, ...
//				v0s = 0; v1s = 0; n = 0;
//				delay(1000);
//				v[iTon][iDelay] = v1;
//			}
			delay(1000);
			v[iTon][0] = v0;
			v[iTon][1] = v1;
			v[iTon][2] = v2;
			v[iTon][3] = v3;
			v[iTon][4] = v4;
		}
		for (iTon = 0; iTon < 9; iTon++){
			Serial.print(iTon+1);
			for (iDelay = 0; iDelay < 5; iDelay++){
				 Serial.print(","); Serial.print(v[iTon][iDelay]);
			}
			Serial.println();
		}
*/
		for (iTon = 0; iTon < 9; iTon++){
			Ton = iTon * 1000 + 1000;
			OCR1A = Ton * 2 - 1; // PWM Duty Cycle
			v0s = 0; v1s = 0; n = 0;
			delay(1000);
			v[iTon][0] = v0;
			v[iTon][1] = v1;
		}
		OCR1A = 1999; // PWM Duty Cycle
		for (iTon = 0; iTon < 9; iTon++){
			Serial.print(iTon+1);
			for (iDelay = 0; iDelay < 2; iDelay++){
				 Serial.print(","); Serial.print(v[iTon][iDelay]);
			}
			Serial.println();
		}
		digitalWrite(13, LOW);
		fMeasure = 0;
#else
		char datBuf[8];
		if (p_spiBuf >= 6){
			if (spiBuf[0] == 0xff && spiBuf[1] == 0xff){ // check data header
				sprintf(datBuf, "%c%c%c%c%c.%c%c",
						getLowerChar(spiBuf[2]), getHigherChar(spiBuf[2]),		
						getLowerChar(spiBuf[3]), getHigherChar(spiBuf[3]),		
						getLowerChar(spiBuf[4]), getHigherChar(spiBuf[4]),		
						getLowerChar(spiBuf[5]));		
				float s = atof(datBuf);
				Serial.print(s);
				Serial.println(" ");
			}
			for (uint8_t i = 0; i < p_spiBuf; i++){
				Serial.print(spiBuf[i], HEX); Serial.print(' ');
			}
			p_spiBuf = 0;
			SPCR &= ~bit(SPE); delay(1); SPCR |= bit(SPE);
		}
#endif
	}
}

