#include <Arduino.h>
#include <SPI.h>

#define PIN_FLAG1 2
#define PIN_FLAG2 3
#define PIN_ADC A0
#define PIN_PWM 9
#define PIN_SW 4
#define PIN_LED 13
#define PIN_POT A1

uint16_t v0, v0_, v1;

uint16_t Ton = 1000;
uint16_t Delay = 100;

// for CDS0730140 (Vs=6V, Rf=30k)
#define X 9
#define Y 6
uint16_t ADCvalue[X][Y] = {
{43, 50, 59, 69, 80, 87},
{42, 50, 59, 68, 80, 88},
{42, 49, 59, 68, 79, 87},
{40, 48, 59, 68, 79, 87},
{38, 46, 58, 68, 80, 87},
{36, 43, 54, 65, 80, 87},
{34, 39, 49, 61, 76, 86},
{30, 34, 43, 53, 67, 77},
{25, 28, 33, 40, 50, 59}
};
float Pos[] = {0, 1, 2.08, 3.03, 3.96, 4.82};

void setup()
{
	pinMode(PIN_PWM, OUTPUT);
	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, LOW);
	pinMode(PIN_FLAG2, OUTPUT); digitalWrite(PIN_FLAG2, LOW);
	pinMode(PIN_SW, INPUT_PULLUP);
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);
//	Serial.begin(115200);
	Serial.begin(9600);
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
	v0_ = analogRead(PIN_ADC);
}

// Timer1 のCompareMatchB割り込み
ISR(TIMER1_COMPB_vect)
{
	v1 = analogRead(PIN_ADC);
	v0 = v0_;
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
	// for CDS0730058
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
	return(Pos_int);
}

#define LEN_LINE 64
char buf[LEN_LINE];
uint8_t pBuf = 0;
float St = 3.0;

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
//			Ton = atoi(buf);
//			OCR1A = Ton * 2 - 1; // update PWM Duty Cycle
//			tm = 0;
		}
		buf[pBuf++] = c;
	}

	// set St by potentiometer
	uint16_t St_ = analogRead(PIN_POT);
	// 0-676(3.3V) -> St: 2.5 - 4
	St = (float)St_ * 1.5 / 676 + 2.5;
	// Position Control
	float S = calc_pos(Ton, v1 - v0);
#define Kp 5.0
	int16_t dTon = (uint16_t)((S - St) * Kp);
	int16_t Ton_t = Ton + dTon;
#define Ton_MAX 9000
#define Ton_MIN 1000
	if (Ton_t > Ton_MAX) Ton = Ton_MAX;
	else if (Ton_t < Ton_MIN) Ton = Ton_MIN;
	else Ton = Ton_t;
	OCR1A = Ton * 2 - 1; // update PWM Duty Cycle
   	Serial.print(">Pos:"); Serial.println(S);
 	Serial.print(">PosT:"); Serial.println(St);
/*
	Serial.print(tm++);
	Serial.print(' ');
	Serial.print(v1 - v0);
	Serial.print(' ');
	Serial.print(St);
	Serial.print(' ');
	Serial.print(S);
	Serial.print(' ');
	Serial.println(Ton);
*/
}


