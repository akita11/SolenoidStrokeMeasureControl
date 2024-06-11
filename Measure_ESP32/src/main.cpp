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

// for PortC
#define PIN_FLAG1 13
#define PIN_FLAG2 14

uint8_t n = 0;
uint8_t N = 8; 
uint16_t v0s, v1s;
uint16_t v0, v1;

uint16_t Ton = 1000;
uint16_t Delay = 100;

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
				digitalWrite(PIN_FLAG1, 1);	
				v0s += analogReadMilliVolts(PIN_ADC);
				digitalWrite(PIN_FLAG1, 0);
				}
			else if (st_int == 2){
				st_int = 0;
				digitalWrite(PIN_FLAG2, 1);
				v1s += analogReadMilliVolts(PIN_ADC);
				digitalWrite(PIN_FLAG2, 0);	
				n++;
				if (n == N)
				{
					n = 0;
					v0 = v0s / N; v0s = 0;
					v1 = v1s / N; v1s = 0;
				}
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
	MCPWM0.int_ena.val |= BM_INT_TIMER0_TEZ; // interrupt at PWM ON
	MCPWM0.int_ena.val |= BM_INT_OP0_TEB;    // interrupt at RegB match
	ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));

 	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, APP_CPU_NUM);
}

uint8_t st = 0;
uint16_t tm = 0;
uint8_t fMeasure = 0;

void loop()
{
	M5.update();

	if (M5.BtnA.wasClicked()){
		fMeasure = 1 - fMeasure;
		delay(500);
	}
	
	if (fMeasure == 1){
		M5.Lcd.clear();
		M5.Lcd.printf("Measuring...");
		int v[9][5];
		digitalWrite(13, HIGH);
		uint8_t iTon, iDelay;;
		for (iTon = 0; iTon < 9; iTon++){
			Ton = iTon * 1000 + 1000;
			mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // PWM ON
			v0s = 0; v1s = 0; n = 0;
			delay(1000);
			v[iTon][0] = v0;
			v[iTon][1] = v1;
			printf("%d,%d,%d\n", iTon + 1, v[iTon][0], v[iTon][1]);
		}
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000); // PWM ON
		M5.Lcd.printf("done\n");
		fMeasure = 0;
	}
}
