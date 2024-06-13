#include <Arduino.h>
#include <M5Unified.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "SliderUI.h"
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

#define NX 320
uint8_t gpos[NX], gposT[NX];
#define NY 200
uint16_t gp = 0;

// for CDS0730140 (Vs=6V, Rf=30k)
#define X 9
#define Y 6
uint16_t ADCvalue[X][Y] = {
{558, 486, 416, 352, 296, 254},
{555, 484, 412, 349, 290, 248},
{554, 484, 410, 345, 289, 242},
{553, 482, 411, 345, 282, 234},
{554, 480, 405, 328, 266, 220},
{550, 468, 382, 306, 242, 203},
{531, 434, 341, 276, 220, 191},
{460, 367, 290, 223, 180, 164},
{327, 253, 196, 154, 136, 129},
};
float Pos[] = {0, 1.08, 2.01, 3.08, 4.06, 4.99};

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

int vmax[X], vmin[X];

uint8_t conv_gpos(float S){
	// S: 0 - 5mm -> 0 - NY - 1
	return((uint8_t)(S / 5.0 * (float)(NY - 1)));
}

static constexpr std::size_t slider_count = 1;
static slider_t slider_list[slider_count];

void setup() {
	M5.begin();
	M5.Display.setTextSize(2);

  slider_list[0].setup({20, NY, 320 - 20,       240 - NY }, 0,  500,  200, TFT_WHITE, TFT_BLACK, TFT_LIGHTGRAY);

	for (uint16_t x = 0; x < NX; x++){ gpos[x] = 0; gposT[x] = 0; }

  M5.Display.setEpdMode(epd_mode_t::epd_fastest);
  for (std::size_t i = 0; i < slider_count; ++i) slider_list[i].draw();

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
    vmax[ton] = 0; vmin[ton] = 9999;
        for (uint8_t ipos = 0; ipos < Y; ipos++){
	        int v0 = ADCvalue[ton][ipos];
	        if (v0 > vmax[ton]) vmax[ton] = v0;
	        if (v0 < vmin[ton]) vmin[ton] = v0;
      }
    }
	disableCore0WDT();
 	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, PRO_CPU_NUM);
}

float calc_pos(int Ton, int vt)
{
  // round ADCvalue to calculated max & min boundary 
	int i_Ton = Ton / 1000 - 1;
	float t = (float)(Ton - (i_Ton + 1) * 1000) / 1000.0;
	float vmax0, vmin0;
  int v0, v1;
	if (i_Ton < X - 1){
  	v0 = vmax[i_Ton];
    v1 = vmax[i_Ton + 1];
    vmax0 = (float)v0 + t * (float)(v1 - v0);
    v0 = vmin[i_Ton];
    v1 = vmin[i_Ton + 1];
    vmin0 = (float)v0 + t * (float)(v1 - v0);
  }
  else{
  	vmax0 = vmax[i_Ton];
   	vmin0 = vmin[i_Ton];
  }
  if (vt > vmax0) vt = (int)vmax0;
  if (vt < vmin0) vt = (int)vmin0;

	uint8_t x, y;
	x = 0; while(x < X - 1){
	  if ((uint16_t)(x+1) * 1000 <= Ton && Ton < (uint16_t)(x+2)*1000) break;
	  x++;
	}
	float x0 = (float)(x + 1);
	t = ((float)Ton / 1000.0 - x0);
	if (t < 0.0) t = 0.0;
	else if (t > 1.0) t = 1.0;
	float s;
    y = 0; while(y < Y - 1){
		float y01 = (1.0 - t) * (float)ADCvalue[x][y] + t * (float)ADCvalue[x+1][y];
		float y23 = (1.0 - t) * (float)ADCvalue[x][y+1] + t * (float)ADCvalue[x+1][y+1];
		s = ((float)vt - y01) / (y23 - y01);
		if (0.0 <= s && s <= 1.0) break;
		y++;
	}
	float Pos_int;
	if (y < Y - 1){
	  Pos_int = (1 - s) * Pos[y] + s * Pos[y+1];
	}
	else{
 	  Pos_int = Pos[Y - 1];
	}
	return(Pos_int);
}

uint8_t iTon = 0;

float St;

void loop() {
	M5.update();

  bool changed = false;
  auto t = M5.Touch.getDetail();
  for (std::size_t i = 0; i < slider_count; ++i)
  {
    if (slider_list[i].update(t)) {
      changed = true;
    }
  }

  if (changed) {
    int w = slider_list[0].getValue();
    if (slider_list[0].wasChanged())
			printf("%d\n", w);
  }

	if (M5.BtnA.wasClicked()){
		iTon = (iTon + 1) % 9;
		Ton = (iTon + 1) * 1000;
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // 1ms, PWM ON
	}

	uint16_t vpot = analogReadMilliVolts(PIN_POT);
	// 0 - 3.3V -> 0 - 4mm
	St = (float)vpot / 3300 * 4.0;
	// Position Control
	float S = calc_pos(Ton, v1 - v0);
#define Kp 5.0
	int16_t dTon = (uint16_t)((St - S) * Kp);
	int16_t Ton_t = Ton + dTon;
#define Ton_MAX 9000
#define Ton_MIN 1000
	if (Ton_t > Ton_MAX) Ton = Ton_MAX;
	else if (Ton_t < Ton_MIN) Ton = Ton_MIN;
	else Ton = Ton_t;

	for (uint16_t x = 0; x < NX - 1; x++){
		M5.Display.drawLine(x, 0, x, NY - 1, BLACK);
		uint16_t px = (gp + x) % NX;
		M5.Display.drawPixel(x, gpos[px], GREEN);
		M5.Display.drawPixel(x, gposT[px], RED);
	}	
	gp = (gp + 1) % NX;

	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // 1ms, PWM ON

//	printf("%.2f %.2f %d %d\n", S, St, Ton, dTon);
//  printf(">Pos:%f\n", S);	printf(">PosT:%f\n", St);
	M5.Display.fillRect(0, 0, 320, 50, BLACK);
	M5.Display.setCursor(0, 0);
	M5.Display.printf("S:%.2f St:%.2f", S, St);
	M5.Display.setCursor(0, 20);
	M5.Display.printf("Ton:%d", Ton);
}
