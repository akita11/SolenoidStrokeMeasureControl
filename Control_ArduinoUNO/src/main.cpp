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
#define Z 2 // I

// data from LongStrokeSolenoid, Rf=15k
int ADCvalue[X][Y][Z] = {
  // Ton=1000
  // I(01), I(07)
  {{33, 59}, // for pos[0]
   {32, 59}, // for pos[1]
   {32, 59},
   {31, 60},
   {29, 62},
   {29, 63},
   {28, 63},
   {27, 65},
   {25, 69}},
  {{83, 107},
   {82, 106},
   {80, 106},
   {78, 106},
   {75, 105},
   {74, 105},
   {71, 104},
   {69, 105},
   {63, 105}},
  {{136, 159},
   {135, 158},
   {133, 157},
   {129, 155},
   {126, 154},
   {124, 153},
   {120, 151},
   {118, 151},
   {108, 148}},
  {{189, 212},
   {189, 211},
   {187, 209},
   {183, 207},
   {179, 206},
   {177, 204},
   {172, 201},
   {170, 201},
   {158, 195}},
  {{244, 266},
   {244, 265},
   {242, 264},
   {238, 261},
   {233, 257},
   {232, 258},
   {227, 254},
   {225, 254},
   {209, 243}},
  {{298, 320},
   {299, 320},
   {300, 321},
   {298, 320},
   {292, 315},
   {291, 314},
   {286, 311},
   {284, 310},
   {270, 301}},
  {{352, 375},
   {356, 378},
   {357, 378},
   {359, 379},
   {356, 377},
   {354, 375},
   {352, 374},
   {347, 370},
   {335, 362}},
  {{407, 433},
   {412, 435},
   {416, 437},
   {420, 439},
   {421, 440},
   {421, 439},
   {419, 438},
   {417, 436},
   {407, 429}},
  {{475, 501},
   {479, 503},
   {483, 504},
   {490, 508},
   {494, 510},
   {494, 510},
   {495, 510},
   {494, 509},
   {489, 505}}
};

float pos[Y] = { 13.11, 15.08, 17.29, 21.11, 25.03, 27.16, 29.02, 31.18, 35.9};

int vt[Z];
int vmax[X][Z], vmin[X][Z];

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
	OCR1B = 1399;		 // 0.7ms for v1

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
//	PORTD &= ~(_BV(PD3));
	v0 = v0_;
	vt[0] = v0_;
	vt[1] = v1;
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
  uint8_t k;
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

float calc_pos(float Ton, int nParam, int *param, int *vt)
{
  float s;
  uint8_t i, j;

  float v0, v1;
  float t;
  float ss, sp;

  // round ADCvalue to calculated max & min boundary 
  for (i = 0; i < nParam; i++){
    int i_Ton = Ton / 1000 - 1;

    t = (float)(Ton - (i_Ton + 1) * 1000) / 1000.0;

    float vmax0, vmin0;
    int v0, v1;
    if (i_Ton < X - 1){
      v0 = vmax[i_Ton][param[i]];
      v1 = vmax[i_Ton + 1][param[i]];
      vmax0 = (float)v0 + t * (float)(v1 - v0);
      v0 = vmin[i_Ton][param[i]];
      v1 = vmin[i_Ton + 1][param[i]];
      vmin0 = (float)v0 + t * (float)(v1 - v0);
    }
    else{
      vmax0 = vmax[i_Ton][param[i]];
      vmin0 = vmin[i_Ton][param[i]];
    }
    if (vt[param[i]] > vmax0) vt[param[i]] = vmax0;
    if (vt[param[i]] < vmin0) vt[param[i]] = vmin0;
  }

  // calculate intial value of s
  ss = 0;
  for (i = 0; i < nParam; i++){
		Serial.print('*'); Serial.print(vt[i]); Serial.print(' ');
    for (j = 0; j < Y - 1; j++){
      v0 = calcV(Ton, j, param[i]);
      v1 = calcV(Ton, j + 1, param[i]);
      t = (float(vt[i]) - v0) / (v1 - v0);
//			Serial.print(i); Serial.print(' '); Serial.print(j); Serial.print(' ');
//			Serial.print(vt[i]); Serial.print(' ');
//			Serial.print(v0); Serial.print(' '); Serial.print(v1); Serial.print(' '); Serial.println(t); 
      if (0 <= t && t <= 1.0) break;
    }
//		Serial.print(i); Serial.print(' '); Serial.print(j); Serial.print(' ');
//		Serial.print(t); Serial.print(' ');
    float ps;
    if (j == Y - 1) ps = pos[Y-1];
    else ps = pos[j] + t * (pos[j+1] - pos[j]);
    sp = (ps - pos[0]) / (pos[Y-1] - pos[0]);
//		Serial.print(ps); Serial.print(' '); Serial.println(sp);
    //    printf("->%d %f %f %f : %f %f\n", j, t, pos[j], pos[j+1], ps, sp);
    ss += sp;
  }
  s = ss / nParam;
	Serial.print(s); Serial.print(' '); Serial.println(calcPos(s));
	// optimization
#define ds 0.01
#define MU 0.0001
#define dEdsMAX  0.005
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
//		Serial.print('*'); Serial.print(st); Serial.print(' '); Serial.println(E0);
    if (E0 < E0MAX || fabs(dEds) < dEdsMAX) break;
    s = s - MU * dEds;
  }
	Serial.print('#'); Serial.print(s); Serial.print(' '); Serial.println(calcPos(s));
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
		Ton = 1000 + 1000 * st; // for 100Hz
//		Ton = 100 + 100 * st; // for 1000Hz
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
//	int param[2] = {0, 4}; // I(0.1), I(0.9)
	int param[2] = {0, 2}; // I(0.1), I(0.5)

	float S = calc_pos(Ton, 2, param, vt);

//	Serial.print(vt[0]); Serial.print(' ');
//	Serial.print(vt[1]); Serial.print(' ');
//	Serial.print(S); Serial.print(' ');
//	Serial.println(calcPos(S));
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


