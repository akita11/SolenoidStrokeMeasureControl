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

#define PIN_PWM 21
#define PIN_ADC 35

#define PIN_FLAG1 2
#define PIN_FLAG2 5

uint16_t v0, v0_, v1;
uint16_t tm = 0;
uint16_t Ton = 1000;
uint16_t Delay = 100;
float duty = 0.1;

// for CBS0730140
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

#define LEN_LINE 64
char buf[LEN_LINE];
uint8_t pBuf = 0;
float St = 3.0;

void timer_task(void *pvParameters){
	while(1){
		if (xSemaphoreTake(pwmSemaphore, 0) == pdTRUE) {
			if (st_int == 1){
				st_int = 0;
				delayMicroseconds(100); // after 100us of PWM ON
				digitalWrite(PIN_FLAG1, 1);	digitalWrite(PIN_FLAG1, 0);	
				v0_ = analogReadMilliVolts(PIN_ADC);
			}
			else if (st_int == 2){
				st_int = 0;
				digitalWrite(PIN_FLAG2, 1);	 digitalWrite(PIN_FLAG2, 0);	
				v1 = analogReadMilliVolts(PIN_ADC);
				v0 = v0_;
			}
		}

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
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 500); // 500us
	MCPWM0.int_ena.val |= BM_INT_TIMER0_TEZ;
	MCPWM0.int_ena.val |= BM_INT_OP0_TEB;
	ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));

 	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, APP_CPU_NUM);
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

void loop() {
/*
	// Position Control
	Ton = (uint16_t)(1000 * duty);
	float S = calc_pos(Ton, v1 - v0);
#define Kp 4.0
	int16_t d_duty = (S - St) * Kp;
	int16_t duty_t = duty + d_duty;
#define duty_MAX 0.9
#define duty_MIN 0.1
	if (duty_t > duty_MAX) duty = duty_MAX;
	else if (duty_t < duty_MIN) duty = duty_MIN;
	else duty = duty_t;
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 10);
	Serial.printf("%d %d %.3f %.3f %.3f\n", tm++, v1 - v0, St, S, duty);
*/
}
