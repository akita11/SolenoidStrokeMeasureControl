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

void timer_task(void *pvParameters){
	while(1){
		if (xSemaphoreTake(pwmSemaphore, 0) == pdTRUE) {
			if (st_int == 1){
				st_int = 0;
				delayMicroseconds(100); // after 100us of PWM ON
				digitalWrite(PIN_FLAG1, 1);
				v0_ = analogReadMilliVolts(PIN_ADC);
				digitalWrite(PIN_FLAG1, 0);	
			}
			else if (st_int == 2){
				st_int = 0;
				digitalWrite(PIN_FLAG2, 1);
				v1 = analogReadMilliVolts(PIN_ADC);
				digitalWrite(PIN_FLAG2, 0);	
				v0 = v0_;
			}
		}
	}
}


void setup() {
	M5.begin();

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

 	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, APP_CPU_NUM);
}

#define LEN_LINE 64
char buf[LEN_LINE];
uint8_t pBuf = 0;

void loop() {
	float St = 2.0;

	// get target position from serial [mm]
	while(Serial.available() > 0 && pBuf < LEN_LINE){
		char c = Serial.read();
		if (c == '\r'){
	 		buf[pBuf] = '\0';
			Serial.println(buf);
			pBuf = 0;
			St = atof(buf);
		}
		buf[pBuf++] = c;
	}

	uint16_t vpot = analogReadMilliVolts(PIN_POT);
	// 0 - 3.3V -> 2.5 - 4.0
	St = (float)vpot / 3.3 * 1.5 + 2.5;

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

	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // 1ms, PWM ON

	printf("%f %f %d\n", S, St, Ton);
//  printf(">Pos:%f\n", S);	printf(">PosT:%f\n", St);
}
