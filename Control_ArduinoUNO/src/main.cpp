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

#define X 9 // Ton
#define Y 12 // pos
#define Z 5 // I

// data from LongStrokeSolenoid, Rf=15k
int ADCvalue[X][Y][Z] = {
  // Ton=1000
  // I(01), I(03), I(05), I(07), I(09)
  {{ 33, 45, 52, 58, 64}, // for pos[0]
   { 32, 45, 53, 59, 64}, // for pos[1]
   { 32, 45, 53, 59, 65},
   { 32, 45, 53, 60, 66},
   { 31, 44, 53, 60, 67},
   { 30, 44, 53, 61, 68},
   { 29, 43, 53, 61, 69},
   { 29, 43, 54, 62, 70},
   { 28, 43, 54, 63, 72},
   { 27, 43, 54, 64, 74},
   { 26, 43, 55, 66, 76},
   { 25, 43, 57, 69, 80}},
  {{ 82, 94, 101, 107, 112},
   { 81, 93, 100, 106, 111},
   { 80, 92, 99, 105, 111},
   { 79, 91, 99, 105, 111},
   { 77, 90, 98, 104, 110},
   { 75, 88, 97, 104, 110},
   { 73, 87, 96, 103, 110},
   { 72, 85, 95, 103, 111},
   { 69, 84, 94, 103, 111},
   { 67, 82, 93, 102, 111},
   { 64, 80, 92, 102, 111},
   { 60, 78, 91, 102, 113}},
  {{ 136, 147, 153, 159, 164},
   { 134, 145, 152, 157, 162},
   { 132, 143, 150, 156, 161},
   { 130, 142, 149, 155, 160},
   { 128, 140, 148, 154, 160},
   { 125, 137, 145, 152, 158},
   { 122, 135, 143, 150, 157},
   { 120, 133, 142, 150, 156},
   { 117, 131, 140, 148, 156},
   { 113, 128, 137, 146, 154},
   { 108, 124, 135, 145, 153},
   { 103, 120, 132, 143, 152}},
  {{ 189, 200, 207, 212, 216},
   { 188, 198, 205, 210, 215},
   { 186, 197, 203, 208, 213},
   { 184, 195, 201, 207, 212},
   { 182, 193, 199, 206, 210},
   { 177, 189, 197, 203, 208},
   { 174, 186, 194, 200, 206},
   { 172, 184, 192, 199, 205},
   { 168, 181, 189, 197, 204},
   { 163, 176, 185, 193, 201},
   { 157, 172, 182, 191, 198},
   { 150, 166, 177, 187, 196}},
  {{ 243, 254, 261, 266, 270},
   { 243, 253, 259, 264, 268},
   { 241, 251, 257, 262, 266},
   { 239, 249, 256, 261, 265},
   { 236, 247, 253, 259, 263},
   { 232, 243, 250, 256, 261},
   { 229, 240, 247, 253, 258},
   { 226, 237, 244, 251, 256},
   { 222, 234, 242, 249, 255},
   { 215, 228, 237, 244, 251},
   { 210, 223, 233, 240, 247},
   { 201, 216, 227, 235, 243}},
  {{ 296, 307, 314, 319, 323},
   { 297, 307, 313, 318, 322},
   { 297, 307, 312, 317, 321},
   { 296, 306, 312, 316, 320},
   { 293, 303, 309, 315, 318},
   { 290, 300, 306, 311, 316},
   { 286, 297, 303, 308, 313},
   { 283, 294, 301, 306, 312},
   { 279, 290, 298, 304, 308},
   { 273, 284, 291, 298, 304},
   { 267, 280, 288, 294, 301},
   { 258, 271, 281, 289, 296}},
  {{ 348, 360, 366, 372, 376},
   { 351, 362, 368, 373, 377},
   { 352, 362, 368, 372, 376},
   { 353, 362, 368, 372, 376},
   { 351, 361, 367, 371, 375},
   { 349, 359, 365, 369, 373},
   { 346, 356, 361, 366, 370},
   { 344, 354, 360, 365, 369},
   { 340, 350, 357, 362, 366},
   { 334, 345, 351, 357, 362},
   { 329, 340, 347, 353, 358},
   { 321, 332, 340, 347, 352}},
  {{ 400, 414, 422, 428, 432},
   { 405, 417, 424, 429, 433},
   { 409, 419, 425, 430, 433},
   { 412, 421, 427, 431, 435},
   { 411, 420, 425, 430, 433},
   { 411, 420, 425, 428, 431},
   { 409, 418, 423, 427, 430},
   { 409, 418, 423, 427, 430},
   { 406, 415, 420, 424, 428},
   { 402, 411, 416, 421, 425},
   { 397, 406, 412, 417, 421},
   { 390, 401, 406, 411, 415}},
  {{ 469, 483, 490, 495, 498},
   { 471, 484, 491, 495, 499},
   { 474, 485, 492, 496, 499},
   { 478, 488, 493, 496, 499},
   { 478, 488, 493, 496, 499},
   { 480, 489, 493, 496, 498},
   { 479, 487, 491, 495, 498},
   { 480, 488, 493, 494, 497},
   { 478, 486, 491, 494, 496},
   { 475, 483, 487, 490, 493},
   { 472, 480, 485, 488, 491},
   { 467, 477, 481, 485, 488}}
};

float pos[Y] = {12.08, 14.3, 16.32, 18.51, 20.07, 22.48, 24.11, 26.11, 28.27, 30.63, 32.75, 35.34};


int vt[2];

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
	// OCR1B/2MHz*1000 = T[ms] ; OCR1B = T[ms]*2000
//	OCR1B = 999;		 // 0.5ms
//	OCR1B = 1399;		 // 0.7ms
	OCR1B = 1799;		 // 0.9ms

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
	v0 = v0_;
	vt[0] = v0_;
	vt[1] = v1;
//	PORTD &= ~(_BV(PD3));
}

uint16_t tm = 0;

#define Dpos (pos[Y-1] - pos[0])

float calcV(float Ton, int i_pos, int i_param)
{
  int i_Ton = Ton / 1000 - 1;
  float t = (float)(Ton - (i_Ton + 1) * 1000) / 1000.0;

  int v0 = ADCvalue[i_Ton][i_pos][i_param];
  int v1 = ADCvalue[i_Ton + 1][i_pos][i_param];

  float v = (float)v0 + t * (float)(v1 - v0);

  return(v);
}

float calcVt(float Ton, float s, int i_param){
  int k;
  float posT = pos[0] + s * Dpos;
  for (k = 0; k < Y - 1; k++){
    if (pos[k] <= posT && posT < pos[k+1]) break;
  }
  float u = (posT - (float)pos[k]) / (float)(pos[k+1] - pos[k]);
  float v0 = calcV(Ton, k, i_param);
  float v1 = calcV(Ton, k+1, i_param);
  float vt;
  vt = v0 + u * (v1 - v0);
  return(vt);
}

#define fabs(x) (((x)>0)?(x):(-(x)))

float calcE(float vt, float Ton, float s, int i_param){
  return(fabs(vt - calcVt(Ton, s, i_param)));
}

float calcPos(float s)
{
  return(pos[0] + s * Dpos);
}

//float calc_pos(int Ton, int ADCval)
float calc_pos(float Ton, int nParam, int *param, float *vt)
{
  float s;
  int i, j;

  float v0, v1;
  float t;
  float ss, sp;

  // calculate intial value of s
  ss = 0;
  for (i = 0; i < nParam; i++){
    for (j = 0; j < Y - 1; j++){
      v0 = calcV(Ton, j, param[i]);
      v1 = calcV(Ton, j + 1, param[i]);
      t = (vt[i] - v0) / (v1 - v0);
      if (0 <= t && t <= 1.0) break;
    }
    float ps = pos[j] + t * (pos[j+1] - pos[j]);
    sp = (ps - pos[0]) / (pos[Y-1] - pos[0]);
    //    printf("->%d %f %f %f : %f %f\n", j, t, pos[j], pos[j+1], ps, sp);
    ss += sp;
  }
  s = ss / nParam;
#define ds 0.001
#define MU 0.0001
#define EMAX  0.005
#define E0MAX  0.1
  int st;
  for (st = 0; st < 500; st++){
    float E1, E0;
    E1 = 0.0; E0 = 0.0;
    for (i = 0; i < nParam; i++){
      //      printf(" %d %d %d %f %f\n", st, i, param[i], vt[i], calcVt(Ton, s, i));
      E1 += calcE(vt[i], Ton, s+ds, param[i]);
      E0 += calcE(vt[i], Ton, s, param[i]);
    }
    float dEds = (E1 - E0) / ds;
    if (E0 < E0MAX || fabs(dEds) < EMAX) break;
    s = s - MU * dEds;
  }
  return(calcPos(s));
}

#define LEN_LINE 64
char buf[LEN_LINE];
uint8_t pBuf = 0;
float St = 3.0;
uint8_t st = 0;

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

	if (digitalRead(PIN_SW) == LOW)
	{
		st = (st + 1) % 9;
		Ton = 1000 + 1000 * st;
		OCR1A = Ton * 2 - 1; // PWM Duty Cycle
		while (digitalRead(PIN_SW) == LOW) delay(10);
		tm = 0;
		delay(500);
	}


/*
	// set St by potentiometer
	uint16_t St_ = analogRead(PIN_POT);
	// 0-1023 -> St:0-4
	St = (float)St_ * 4.0 / 1023.0;
*/

	// Position Control

//	float calc_pos(float Ton, int nParam, int *param, float *vt)
	int param[2] = {0, 4}; // I(0.1), I(0.9)
	float S = calc_pos(Ton, 2, param, (float *)vt);

	Serial.print(S);
	Serial.print(' ');
	Serial.println(Ton);
/*
#define Kp 5.0
	int16_t dTon = (uint16_t)((S - St) * Kp);
	int16_t Ton_t = Ton + dTon;
#define Ton_MAX 9000
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
*/
}


