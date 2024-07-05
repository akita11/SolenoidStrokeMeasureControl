#include <Arduino.h>
#include <M5Unified.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "SliderUI.h"

// source code from ChatGPT, generated by Python code (modified from ChatGPT's one):
// https://colab.research.google.com/drive/1TaIiMXHg7wuIP13Lcu2kMngkYvkSL9mC?usp=sharing

// Solenoid: 5V Solenoid (Measure_SSBH0830-01)
#include "esp32_pos_prediction_SSBH0830-01.c"
//#include "esp32_temp_prediction_SSBH0830-01.c"

// MCPWM0, MCPWM1

#define BM_INT_TIMER0_TEZ (1 << 3)
#define BM_INT_OP0_TEA	(1 << 15)
#define BM_INT_OP0_TEB	(1 << 18)

// for Core2
// PortA
#define PIN_PWM 32
#define PIN_ADC 33

// PortC, for degbugging
#define PIN_FLAG1 13
#define PIN_FLAG2 14

uint16_t v0, v0_, v1;
uint16_t Ton = 5000;
float St = 1.0;
float S;
float Kp = 9.0;

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
				// PWM=0->1
				st_int = 0;
				delayMicroseconds(100); // after 100us of PWM ON
//				digitalWrite(PIN_FLAG1, 1);
				v0_ = analogReadMilliVolts(PIN_ADC);
//				digitalWrite(PIN_FLAG1, 0);	
			}
			else if (st_int == 2){
				// 700us after PWM ON
				st_int = 0;
//				digitalWrite(PIN_FLAG2, 1);
				v1 = analogReadMilliVolts(PIN_ADC);
//				digitalWrite(PIN_FLAG2, 0);	
				v0 = v0_;
				digitalWrite(PIN_FLAG1, 1 - digitalRead(PIN_FLAG1));
				printf("%d,%.3f,%.3f\n", millis(), S, St);
				mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // set PWM
			}
		}
	}
}

static constexpr std::size_t slider_count = 2;
static slider_t slider_list[slider_count];

void setup() {
	M5.begin();
	M5.Display.setTextSize(2);

	// slider for St
  slider_list[0].setup({0, 180, 320, 60}, 0,  500,  100, TFT_WHITE, TFT_BLACK, TFT_LIGHTGRAY);

	// slider for Kp
  slider_list[1].setup({0, 0, 320, 40}, 0,  1000,  (int)(Kp*100), TFT_RED, TFT_BLACK, TFT_RED);

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

	disableCore0WDT();
 	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, PRO_CPU_NUM);
}

uint8_t iTon = 0;

uint16_t xt, xt0;
uint16_t pt, pt0;

void loop() {
	// 1 cycle ~ 5ms
	M5.update();

  bool changed = false;
  auto t = M5.Touch.getDetail();

  if (slider_list[0].update(t)) {
    if (slider_list[0].wasChanged()) St = (float)(slider_list[0].getValue()) / 100;
	}
	if (slider_list[1].update(t)) {
  	if (slider_list[1].wasChanged()){
			Kp = (float)(slider_list[1].getValue()) / 100;
			M5.Display.fillRect(0, 40, 320, 20, TFT_BLACK);
			M5.Display.setCursor(0, 40);
			M5.Display.setTextColor(TFT_RED, TFT_BLACK);
			M5.Display.printf("Kp:%.2f", Kp);
		}
	}
/*
	// for test, set Ton by slider
	if (slider_list[1].update(t)) {
  	if (slider_list[1].wasChanged()){
			Ton = (float)(slider_list[1].getValue()) * 10; // slider:0-1000 -> 0-10000 (10ms)
			M5.Display.fillRect(0, 40, 320, 20, TFT_BLACK);
			M5.Display.setCursor(0, 40);
			M5.Display.setTextColor(TFT_RED, TFT_BLACK);
			M5.Display.printf("Ton:%d", Ton);
			mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // 1ms, PWM ON
		}
	}
*/

	// Position Measure
	float param[3];
	param[0] = (float)Ton / 1000; param[1] = (float)v0; param[2] = (float)v1;
	S = predict_pos(param);
	// Position Control
	int16_t dTon = (uint16_t)((St - S) * Kp);
	int16_t Ton_t = Ton + dTon;
#define Ton_MAX 9500
#define Ton_MIN 1000
	if (Ton_t > Ton_MAX) Ton = Ton_MAX;
	else if (Ton_t < Ton_MIN) Ton = Ton_MIN;
	else Ton = Ton_t;
//	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // set PWM

	xt = (uint16_t)((S / 5.0) * 320);
	M5.Display.drawFastVLine(xt0, 100, 80, TFT_BLACK);
	M5.Display.drawFastVLine(xt,  100, 80, TFT_GREEN);
	pt = (uint16_t)(Ton * 32 / 950);
	M5.Display.drawFastVLine(pt0, 80, 20, TFT_BLACK);
	M5.Display.drawFastVLine(pt,  80, 20, TFT_CYAN);
	xt0 = xt; pt0 = pt;

//  printf(">Pos:%f\n", S);	printf(">PosT:%f\n", St); // for Telepolot
	delay(1);
//	delay(10);
}
