#include <Arduino.h>
#include <M5Unified.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// using MCPWM
// https://qiita.com/motorcontrolman/items/18abd9738860f6ba5620
// https://rt-net.jp/mobility/archives/10150

// ESP32 timer interrupt & WDT timeout
// https://qiita.com/GANTZ/items/892841dcdd6dbfab0baa

// MCPWM0, MCPWM1

// events
// UTEP : count-up, timer=period (PWM_TIMERx_PERIOD)
// UTEZ : count-up, timer=zero
// UTEA : count-up, timer=regA
// UTEB : count-up, timer=regB

//static const mcpwm_unit_t UNIT = MCPWM_UNIT_0;
//#define UNIT MCPWM_UNIT_0

#define BM_INT_TIMER0_TEZ (1 << 3)
#define BM_INT_OP0_TEA	(1 << 15)
#define BM_INT_OP0_TEB	(1 << 18)

// for PortA
#define PIN_PWM 32
#define PIN_ADC 33

// for PortB
#define PIN_POT 36

// for PortC
#define PIN_FLAG1 13
#define PIN_FLAG2 14

uint16_t v0, v0_, v1;
uint16_t Ton = 1000;

// for LongStrokeSolenoid (Vs=12V, Rf=15k)
#define X 9
#define Y 7
#define Z 2
uint16_t ADCvalue[X][Y][Z] = {
	// Ton=1
{{241,336}, // pos[0]
 {233,340}, // pos[1]
 {230,347},
 {226,352},
 {222,365},
 {215,376},
 {210,404}},
{{483,579},
 {464,572},
 {456,571},
 {445,569},
 {432,569},
 {415,571},
 {391,574}},
{{742,833},
 {717,817},
 {704,809},
 {689,806},
 {668,795},
 {643,784},
 {608,776}},
{{1010,1098},
 {981,1077},
 {963,1068},
 {950,1058},
 {920,1043},
 {888,1027},
 {840,1004}},
{{1266,1357},
 {1252,1340},
 {1238,1331},
 {1222,1321},
 {1191,1300},
 {1156,1278},
 {1103,1247}},
{{1517,1614},
 {1529,1612},
 {1518,1603},
 {1503,1592},
 {1476,1572},
 {1442,1550},
 {1384,1515}},
{{1756,1863},
 {1800,1883},
 {1800,1879},
 {1791,1876},
 {1770,1858},
 {1740,1837},
 {1689,1799}},
{{2015,2128},
 {2085,2163},
 {2091,2165},
 {2098,2170},
 {2087,2160},
 {2064,2142},
 {2025,2114}},
{{2358,2462},
 {2407,2486},
 {2426,2495},
 {2440,2502},
 {2443,2502},
 {2432,2494},
 {2408,2476}}
};
float pos[] = {13.08, 16.37, 19.03, 23.08, 27.15, 31.01, 35.58};

/*
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
*/
volatile SemaphoreHandle_t pwmSemaphore;
volatile uint8_t st_int = 0;

void IRAM_ATTR isr_handler(void *XX)
{
	if (MCPWM0.int_st.val & BM_INT_TIMER0_TEZ){
	  // interrupt of Timer0 == 0
		st_int = 1;
  	}
  	else if (MCPWM0.int_st.val & BM_INT_OP0_TEB){
		// interrupt of Timer0 == REGB
		st_int = 2;
	}
	// other interrupt may occur, so don't make st_int=0
	xSemaphoreGiveFromISR(pwmSemaphore, NULL);
	MCPWM0.int_clr.val = MCPWM0.int_st.val; // clear interrupt flags
}

void timer_task(void *pvParameters){
	while(1){
		if (xSemaphoreTake(pwmSemaphore, 0) == pdTRUE) {
			if (st_int == 1){
				st_int = 0;
				delayMicroseconds(100); // after 100us of PWM ON
//				digitalWrite(PIN_FLAG1, 1);
				v0_ = analogReadMilliVolts(PIN_ADC);
//				digitalWrite(PIN_FLAG1, 0);	
			}
			else if (st_int == 2){
				st_int = 0;
//				digitalWrite(PIN_FLAG2, 1);
				v1 = analogReadMilliVolts(PIN_ADC);
//				digitalWrite(PIN_FLAG2, 0);	
				v0 = v0_;
			}
		}
	}
}

int vt[Z];
int vmax[X][Z], vmin[X][Z];

void setup() {
	M5.begin();
	M5.Display.setTextSize(2);

	pwmSemaphore = xSemaphoreCreateBinary();

	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, 0);
	pinMode(PIN_FLAG2, OUTPUT); digitalWrite(PIN_FLAG2, 0);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_PWM);

	mcpwm_config_t pwm_config;
	pwm_config.frequency = 100; // 100Hz,
	pwm_config.cmpr_a = 0; // duty cycle for A
	pwm_config.cmpr_b = 0; // duty cycle for B
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // active high
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,  &pwm_config);

	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000); // 1ms, PWM ON
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 700); // 700us
	MCPWM0.int_ena.val |= BM_INT_TIMER0_TEZ;
	MCPWM0.int_ena.val |= BM_INT_OP0_TEB;
	ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));

  // calculate boundary of v
  for (uint8_t ton = 0; ton < X; ton++){
    for (uint8_t param = 0; param < Z; param++){
      vmax[ton][param] = 0; vmin[ton][param] = 9999;
        for (uint8_t ipos = 0; ipos < Y; ipos++){
	        int v0 = ADCvalue[ton][ipos][param];
	        if (v0 > vmax[ton][param]) vmax[ton][param] = v0;
	        if (v0 < vmin[ton][param]) vmin[ton][param] = v0;
      }
    }
  }
	disableCore0WDT();
 	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, PRO_CPU_NUM);
}

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
    if (vt[param[i]] > vmax0) vt[param[i]] = (int)vmax0;
    if (vt[param[i]] < vmin0) vt[param[i]] = (int)vmin0;
  }

  // calculate intial value of s
  ss = 0;
  for (i = 0; i < nParam; i++){
    for (j = 0; j < Y - 1; j++){
      v0 = calcV(Ton, j, param[i]);
      v1 = calcV(Ton, j + 1, param[i]);
      t = (float(vt[i]) - v0) / (v1 - v0);
      if (0 <= t && t <= 1.0) break;
    }
    float ps;
    if (j == Y - 1) ps = pos[Y-1];
    else ps = pos[j] + t * (pos[j+1] - pos[j]);
    sp = (ps - pos[0]) / (pos[Y-1] - pos[0]);
    ss += sp;
  }
  s = ss / nParam;
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
      E1 += calcE(vt[i], Ton, s+ds, param[i]);
      E0 += calcE(vt[i], Ton, s, param[i]);
    }
    float dEds = (E1 - E0) / ds;
    if (E0 < E0MAX || fabs(dEds) < dEdsMAX) break;
    s = s - MU * dEds;
  }
  return(calcPos(s));
}

#define LEN_LINE 64
char buf[LEN_LINE];
uint8_t pBuf = 0;

uint8_t iTon = 0;
int param[2] = {0, 1}; // I(0.1), I(0.7)

float St = 30.0;

void loop() {
	M5.update();
	if (M5.BtnA.wasClicked()){
		iTon = (iTon + 1) % 9;
		Ton = (iTon + 1) * 1000;
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // 1ms, PWM ON
	}

	uint16_t vpot = analogReadMilliVolts(PIN_POT);
	// 0 - 3.3V -> 13 - 35
	St = (float)vpot / 3300 * 22 + 13;
	vt[0] = v0; vt[1] = v1;
	float S = calc_pos(Ton, 2, param, vt);
/*
	// Position Control
#define Kp 5.0
	int16_t dTon = (uint16_t)((S - St) * Kp);
	int16_t Ton_t = Ton + dTon;
#define Ton_MAX 9000
#define Ton_MIN 1000
	if (Ton_t > Ton_MAX) Ton = Ton_MAX;
	else if (Ton_t < Ton_MIN) Ton = Ton_MIN;
	else Ton = Ton_t;

	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // 1ms, PWM ON

	printf("%.2f %.2f %d %d\n", S, St, Ton, dTon);
	M5.Display.fillRect(0, 0, 320, 50, BLACK);
	M5.Display.setCursor(0, 0);
	M5.Display.printf("S:%.2f St:%.2f", S, St);
	M5.Display.setCursor(0, 20);
	M5.Display.printf("Ton:%d", Ton);
*/
}
