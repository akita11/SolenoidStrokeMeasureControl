#include <Arduino.h>

#define PIN_FLAG1 2
#define PIN_FLAG2 3
#define PIN_ADC A0
#define PIN_PWM 9
#define PIN_SW 4
#define PIN_LED 13

uint16_t v0, v0_, v1;

uint16_t Ton = 1000;
uint16_t Delay = 100;

// for CDS0730
/*
// (1st)
#define X 9
#define Y 6
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
float L[] = {123.3, 111.5, 94.43, 86.33, 65.3};
	// for CDS043 (1st)
    // L[mH] = -11.861 * S[mm] + 114.53
    //S = (114.53 - Lint) / 11.861;
*/

// (2nd)
#define X 9
#define Y 6
uint16_t ADCvalue[X][Y] = {
{73, 85, 99, 113, 133, 149},
{72, 84, 99, 113, 133, 148},
{69, 82, 98, 112, 132, 147},
{65, 78, 95, 112, 133, 147},
{61, 74, 91, 108, 132, 147},
{58, 69, 86, 103, 128, 146},
{53, 63, 78, 94, 120, 139},
{47, 55, 67, 82, 105, 123},
{39, 43, 50, 60, 78, 94}
};

float Pos[] = {0, 1.03, 2.13, 3.02, 4.1, 4.87};

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
	sei();				   // enable global interrupt
}

// Timer1 のオーバーフロー割り込み (=PWM ON)
ISR(TIMER1_OVF_vect)
{
	delayMicroseconds(Delay);
//	PORTD |= _BV(PD2);
	v0_ = analogRead(PIN_ADC);
//	PORTD &= ~(_BV(PD2));
}

// Timer1 のCompareMatchB割り込み
ISR(TIMER1_COMPB_vect)
{
//	PORTD |= _BV(PD3);
	v1 = analogRead(PIN_ADC);
	v0 = v0_
//	PORTD &= ~(_BV(PD3));
}

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
	if (t < 0.0) t = 0.0;
	else if (t > 1.0) t = 1.0;
	float s;
    y = 0; while(y < Y - 1){
		float y01 = (1.0 - t) * (float)ADCvalue[x][y] + t * (float)ADCvalue[x+1][y];
		float y23 = (1.0 - t) * (float)ADCvalue[x][y+1] + t * (float)ADCvalue[x+1][y+1];
		s = ((float)ADCval - y01) / (y23 - y01);
		if (0.0 <= s && s <= 1.0) break;
		y++;
	}
/*
	// for CDS0730, 1st
	float Lint;
    float S;
	if (ADCval < ADCvalue[x][0]) Lint = L[0];
	else if (y < Y - 1){
	  Lint = (1 - s) * L[y] + s * L[y+1];
	}
	else{
 	  Lint = L[Y - 1];
	}
	// for CDS0730, 1st
    // L[mH] = -11.861 * S[mm] + 114.53
    //S = (114.53 - Lint) / 11.861;
	// for LongStroke
	// L[mH] = -0.823 * S[mm] + 118.3
    S = (118.3 - Lint) / 0.823;
*/
	float Pos_int;
	if (ADCval < ADCvalue[x][0]) Pos_int = Pos[0];
	else if (y < Y - 1){
	  Pos_int = (1 - s) * Pos[y] + s * Pos[y+1];
	}
	else{
 	  Pos_int = Pos[Y - 1];
	}

	Serial.print(x);
	Serial.print(' ');
	Serial.print(t);
	Serial.print(' ');
	Serial.print(y);
	Serial.print(' ');
	Serial.print(s);
	Serial.print(' ');
	Serial.println(Pos_int);
	return(Pos_int);
}

#define LEN_LINE 64
char buf[LEN_LINE];
uint8_t pBuf = 0;
float St = 3.0;

uint8_t fMeasure = 0;

void loop()
{
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

	// Position Control
	float S = calc_pos(Ton, v1 - v0);
#define Kp 4.0
	int16_t dTon = (uint16_t)((S - St) * Kp);
	int16_t Ton_t = Ton + dTon;
#define Ton_MAX 9900
#define Ton_MIN 1000
	if (Ton_t > Ton_MAX) Ton = Ton_MAX;
	else if (Ton_t < Ton_MIN) Ton = Ton_MIN;
	else Ton = Ton_t;
	OCR1A = Ton * 2 - 1; // update PWM Duty Cycle
	Serial.print(tm++);
	Serial.print(' ');
	Serial.print(v1 - v0);
	Serial.print(' ');
	Serial.print(St);
	Serial.print(' ');
	Serial.print(S);
	Serial.print(' ');
	Serial.println(Ton);
}

