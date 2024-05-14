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
// uint8_t N = 128;
uint8_t N = 4;

uint16_t Ton = 1000;
uint16_t Delay = 100;

#define X 9
#define Y 5
uint16_t ADCvalue[X][Y] = {
	{75, 82, 101, 116, 144}, // for Ton=1, L[0], L[1], ...
	{73, 81, 101, 116, 143}, // for Ton=2, L[0], L[1], ...
	{68, 79, 100, 113, 143},
	{63, 76, 97, 109, 142},
	{59, 72, 93, 105, 142},
	{56, 67, 88, 100, 140},
	{51, 62, 81, 91, 132},
	{47, 53, 70, 80, 116},
	{38, 42, 53, 61, 87}
};

// Ton = {1, 2, 3, 4, 5, 6, 7, 8, 9}
float L[] = {123.3, 111.5, 94.43, 86.33, 65.3};

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

float calc_pos(int Ton, int ADCval)
{
	uint8_t x, y;
	x = 0; while(x < X - 1){
	  if ((uint16_t)(x+1) * 1000 <= Ton && Ton < (uint16_t)(x+2)*1000) break;
	  x++;
	}
	float x0 = (float)(x + 1);
	float t = ((float)Ton / 1000.0 - x0);
	float s;
      
	y = 0; while(y < Y - 1){
	  float y01 = (1.0 - t) * (float)ADCvalue[x][y] + t * (float)ADCvalue[x+1][y];
	  float y23 = (1.0 - t) * (float)ADCvalue[x][y+1] + t * (float)ADCvalue[x+1][y+1];
	  s = ((float)ADCval - y01) / (y23 - y01);
	  if (0.0 <= s && s <= 1.0) break;
	  y++;
	}
    float Lint;
    float S;
	if (ADCval < ADCvalue[x][0]) Lint = L[0];
	else if (y < Y - 1){
	  Lint = (1 - s) * L[y] + s * L[y+1];
	  // L[mH] = -11.861 x S[mm] + 114.53
	}
	else{
 	  Lint = L[Y - 1];
	}
    S = (114.53 - Lint) / 11.861;
/*
	Serial.print(x);
	Serial.print(' ');
	Serial.print(t);
	Serial.print(' ');
	Serial.print(y);
	Serial.print(' ');
	Serial.print(s);
	Serial.print(' ');
	Serial.println(S);
*/
	return(S);
}

#define LEN_LINE 64
char buf[LEN_LINE];
uint8_t pBuf = 0;
float St = 3.0;

void loop()
{
	if (digitalRead(PIN_SW) == LOW)
	{
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
		switch (st) 
		{
			case 0:	Ton = 1000;	break;
			case 1:	Ton = 2000;	break;
			case 2:	Ton = 3000;	break;
			case 3:	Ton = 4000;	break;
			case 4:	Ton = 5000;	break;
			case 5:	Ton = 6000;	break;
			case 6:	Ton = 7000;	break;
			case 7:	Ton = 8000;	break;
			case 8:	Ton = 9000;	break;
		}
		OCR1A = Ton * 2 - 1; // PWM Duty Cycle
		while (digitalRead(PIN_SW) == LOW) delay(10);
		tm = 0;
	}
	
	uint16_t ADC0 = v1 - v0;
/*
	Serial.print(">ADC0:"); Serial.println(ADC0);
	Serial.print(">Lint:"); Serial.println(Lint);
	Serial.print(">S:"); Serial.println(S);
	Serial.print(tm++);
	Serial.print(' ');
	Serial.print(Ton);
	Serial.print(' ');
	Serial.print(ADC0);
	Serial.print(' ');
	Serial.println(calc_pos(Ton, ADC0));
*/

	// get target position from serial [mm]
	while(Serial.available() > 0 && pBuf < LEN_LINE){
		char c = Serial.read();
		if (c == '\r'){
	 		buf[pBuf] = '\0';
			Serial.println(buf);
			pBuf = 0;
			St = atof(buf);
//			tm = 0;
		}
		buf[pBuf++] = c;
	}

	float S = calc_pos(Ton, ADC0);
	// S : Neutral = 4.0(PWM=1.0), Pushed = 0.0(PWM=9.9)
/*
	// preliminary simple up/down control
	if (S > St){
		if (Ton < 9000) Ton += 10;
	}
	else {
		if (Ton > 600) Ton -= 10;
	}
	*/
#define Kp 4.0
	int16_t dTon = (uint16_t)((S - St) * Kp);
	int16_t Ton_t = Ton + dTon;
#define Ton_MAX 990
#define Ton_MIN 100
	if (Ton_t < Ton_MAX) Ton = Ton_MAX;
	else if (Ton_t < Ton_MIN) Ton = Ton_MIN;
	else Ton = Ton_t;
	OCR1A = Ton * 2 - 1; // update PWM Duty Cycle
	Serial.print(tm++);
	Serial.print(' ');
	Serial.print(St);
	Serial.print(' ');
	Serial.print(S);
	Serial.print(' ');
	Serial.print(dTon);
	Serial.print(' ');
	Serial.println(Ton);
}

