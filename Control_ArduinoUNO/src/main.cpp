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
#define Y 9 // pos
#define Z 5 // I

// data from LongStrokeSolenoid, Rf=20k
int ADCvalue[X][Y][Z] = {
  {
    // I(01) I(03) I(05) I(07) I(-01)
    { 43, 58, 68, 75, 82 }, // I for Ton=1ms, pos[0]
    { 42, 58, 68, 76, 83 }, // I for Ton=1ms, pos[1]
    { 42, 58, 68, 76, 84 }, // I for Ton=1ms, pos[2]
    { 41, 58, 68, 77, 85 },
    { 40, 57, 69, 78, 86 },
    { 38, 56, 69, 80, 90 },
    { 36, 56, 70, 82, 93 },
    { 34, 56, 72, 86, 99 },
    { 32, 56, 74, 90, 104 }
  }, {
    { 108, 123, 132, 140, 174 }, // I for Ton=2ms, pos[0]
    { 107, 121, 131, 138, 173 }, // I for Ton=2ms, pos[1]
    { 105, 120, 130, 137, 175 }, // I for Ton=2ms, pos[2]
    { 103, 119, 129, 137, 176 },
    { 101, 117, 128, 136, 178 },
    { 96, 113, 125, 135, 184 },
    { 91, 110, 123, 134, 190 },
    { 85, 106, 121, 134, 199 },
    { 80, 102, 119, 134, 207 }
  }, {
    { 178, 192, 201, 208, 261 },
    { 176, 190, 199, 206, 261 },
    { 173, 188, 196, 204, 262 },
    { 171, 185, 195, 202, 264 },
    { 168, 183, 193, 200, 267 },
    { 161, 177, 188, 197, 274 },
    { 154, 172, 184, 194, 281 },
    { 145, 164, 178, 190, 292 },
    { 137, 158, 174, 188, 302 }
  }, {
    { 249, 263, 272, 278, 347 },
    { 246, 260, 268, 275, 346 },
    { 244, 258, 266, 273, 346 },
    { 241, 255, 264, 271, 348 },
    { 237, 251, 261, 268, 351 },
    { 229, 244, 255, 263, 358 },
    { 221, 237, 249, 258, 365 },
    { 209, 228, 241, 252, 377 },
    { 200, 220, 234, 247, 387 }
  }, {
    { 321, 335, 343, 350, 431 },
    { 319, 332, 340, 347, 427 },
    { 316, 330, 338, 344, 427 },
    { 313, 327, 335, 342, 428 },
    { 309, 323, 332, 339, 430 },
    { 301, 316, 325, 333, 436 },
    { 292, 308, 318, 327, 443 },
    { 280, 297, 308, 319, 454 },
    { 268, 287, 300, 312, 464 }
  }, {
    { 390, 404, 413, 419, 516 },
    { 390, 403, 411, 417, 510 },
    { 389, 402, 410, 416, 507 },
    { 388, 400, 408, 414, 506 },
    { 385, 397, 405, 411, 506 },
    { 377, 390, 399, 405, 511 },
    { 368, 382, 392, 399, 517 },
    { 355, 371, 382, 390, 526 },
    { 343, 361, 372, 382, 535 }
  }, {
    { 458, 473, 482, 489, 597 },
    { 460, 474, 482, 488, 589 },
    { 461, 474, 482, 488, 585 },
    { 462, 474, 481, 487, 582 },
    { 461, 473, 480, 486, 579 },
    { 456, 468, 476, 482, 581 },
    { 448, 461, 470, 476, 585 },
    { 437, 451, 460, 467, 592 },
    { 425, 441, 451, 459, 599 }
  }, {
    { 527, 544, 554, 562, 669 },
    { 532, 547, 556, 562, 662 },
    { 536, 550, 558, 564, 656 },
    { 539, 552, 559, 564, 652 },
    { 540, 552, 559, 564, 649 },
    { 540, 551, 558, 563, 647 },
    { 535, 546, 554, 559, 649 },
    { 526, 538, 546, 551, 653 },
    { 516, 530, 538, 545, 657 }
  }, {
    { 617, 635, 644, 651, 726 },
    { 619, 635, 644, 650, 721 },
    { 622, 636, 644, 650, 718 },
    { 626, 639, 646, 651, 714 },
    { 629, 641, 647, 651, 710 },
    { 632, 642, 648, 652, 708 },
    { 630, 640, 646, 650, 707 },
    { 625, 635, 641, 645, 709 },
    { 619, 630, 636, 640, 710 }
  }
};
float pos[Y] = {12.24, 14.25, 16.31, 18.2, 20.34, 24.36, 28.12, 32.31, 35.05};

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
	OCR1B = 1399;		 // 0.7ms

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
      if (0 <= t && t < 1.0) break;
    }
    float ps = pos[j] + t * (pos[j+1] - pos[j]);
    sp = (ps - pos[0]) / (pos[Y-1] - pos[0]);
    //    printf("->%d %f %f %f : %f %f\n", j, t, pos[j], pos[j+1], ps, sp);
    ss += sp;
  }
  s = ss / nParam;
#define ds 0.01
#define MU 0.0001
#define EMAX  0.005
  int st;
  for (st = 0; st < 500; st++){
    float dE = 0.0;
    for (i = 0; i < nParam; i++){
      //      printf(" %d %d %d %f %f\n", st, i, param[i], vt[i], calcVt(Ton, s, i));
      dE += calcE(vt[i], Ton, s+ds, param[i]) - calcE(vt[i], Ton, s, param[i]);
    }
    float dss = dE / ds;
    if (fabs(dE) < EMAX) break;
    s = s - MU * dss;
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


	// set St by potentiometer
	uint16_t St_ = analogRead(PIN_POT);
	// 0-1023 -> St:0-4
	St = (float)St_ * 4.0 / 1023.0;

	// Position Control

//	float calc_pos(float Ton, int nParam, int *param, float *vt)
	int param[2] = {0, 3}; // I(0.1), I(0.7)
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


