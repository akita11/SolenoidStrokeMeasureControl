// Solenoid Stroke Measure & Control using CNN
// (c) 2024 Junichi Akita (@akita11, akita@ifdl.jp)
// running TensorFlowLite on ESP32/ESP32-S3
// based on Chirale-TensorFlowLite's example, hello_world
// using M5Stack Core2/S3SE　with library: https://github.com/spaziochirale/Chirale_TensorFlowLite
//
// Copyright 2024 Chirale, TensorFlow Authors. All Rights Reserved.
// Licensed under the Apache License, Version 2.0;

#include <M5Unified.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <Chirale_TensorFlowLite.h>

#include "model_SSBH-0830-CoreS3.h"	// generated model file

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// set PWM manually, and measure position. No control
#define TEST

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 4000;
uint8_t tensor_arena[kTensorArenaSize];

#define BM_INT_TIMER0_TEZ (1 << 3)
#define BM_INT_OP0_TEA	(1 << 15)
#define BM_INT_OP0_TEB	(1 << 18)

#if defined(ARDUINO_M5STACK_CORE2)
// for Core2
// for PortA
#define PIN_PWM 32
#define PIN_ADC 33
// for PortC
#define PIN_FLAG1 13
#define PIN_FLAG2 14
#define ADC_CHANNEL	ADC1_CHANNEL_5 // 32K_XN / GPIO33
#define ADC_ATTEN  ADC_ATTEN_DB_11

#elif defined(ARDUINO_M5STACK_CORES3)
// for CoreS3SE
// for PortA
#define PIN_PWM 2
#define PIN_ADC 1 // ADC1's Ch.0
// for PortC
#define PIN_FLAG1 6
#define PIN_FLAG2 7
#define ADC_CHANNEL	ADC1_CHANNEL_0 // GPIO1
#define ADC_ATTEN  ADC_ATTEN_DB_12
#else
#error "No pin definition for this board"
#endif

static esp_adc_cal_characteristics_t adc1_chars;
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT

// memo: ADC_ATTEN_DB_11 == ADC_ATTEN_DB_12 (3)
//   ref: https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32c3/api-reference/peripherals/adc.html#_CPPv415ADC_ATTEN_DB_11

uint16_t v0, v0_, v1;
uint16_t Ton = 5000;
float St = 1.0;
float Kp = 9.0;

typedef struct solenoid{
  float pos;
  float temp;
} Solenoid;

Solenoid sol;

volatile SemaphoreHandle_t pwmSemaphore;
volatile uint8_t st_int = 0;
uint8_t iTon = 0;
uint16_t xt, xt0;
uint16_t pt, pt0;

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

Solenoid predict(float Ton, float v0, float v1)
{
	Solenoid s;
	input->data.f[0] = (Ton - scaler_i_mean[0]) / scaler_i_scale[0];
	input->data.f[1] = (v0	- scaler_i_mean[1]) / scaler_i_scale[1];
	input->data.f[2] = (v1	- scaler_i_mean[2]) / scaler_i_scale[2];
	// Run inference, and report if an error occurs
	TfLiteStatus invoke_status = interpreter->Invoke();
	if (invoke_status != kTfLiteOk) {
		printf("Invoke failed!\n");
		while(1);
	}
	s.pos  = output->data.f[0] * scaler_o_scale[0] + scaler_o_mean[0];
	s.temp = output->data.f[1] * scaler_o_scale[1] + scaler_o_mean[1];

	return(s);
}

void timer_task(void *pvParameters){
	while(1){
		printf("%d,%d,%d\n", Ton, v0, v1); // for debug
		/*
#ifdef TEST
		// for test, set Ton by Fader
//		Ton = (int)(analogReadMilliVolts(8) * 10000 / 3047); // 0-3047 -> 0-10000
		Ton = 1000;
		M5.Display.fillRect(0, 40, 320, 20, TFT_BLACK);
		M5.Display.setCursor(0, 40);
		M5.Display.setTextColor(TFT_RED, TFT_BLACK);
		M5.Display.printf("Ton:%d", Ton);
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton);
#else
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
#endif
		// Position Measure
		sol = predict((float)Ton/1000.0, (float)v0, (float)v1);
		M5.Display.fillRect(0, 1000, 320, 20, TFT_BLACK);
		M5.Display.setCursor(0, 100);
		M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
		M5.Display.printf("S:%.3f / Temp:%.2f", sol.pos, sol.temp);

#ifdef TEST
		printf("%d,%d,%d,%.3f\n", Ton, v0, v1, sol.pos); // for debug
#else
	// Position Control
		int16_t dTon = (uint16_t)((St - sol.pos) * Kp);
		int16_t Ton_t = Ton + dTon;
#define Ton_MAX 9500
#define Ton_MIN 1000
		if (Ton_t > Ton_MAX) Ton = Ton_MAX;
		else if (Ton_t < Ton_MIN) Ton = Ton_MIN;
		else Ton = Ton_t;
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // set PWM

		xt = (uint16_t)((sol.pos / 5.0) * 320);
		M5.Display.drawFastVLine(xt0, 100, 80, TFT_BLACK);
		M5.Display.drawFastVLine(xt,	100, 80, TFT_GREEN);
		pt = (uint16_t)(Ton * 32 / 950);
		M5.Display.drawFastVLine(pt0, 80, 20, TFT_BLACK);
		M5.Display.drawFastVLine(pt,	80, 20, TFT_CYAN);
		xt0 = xt; pt0 = pt;
#endif
*/
		delay(1);
	}
}

void setup() {
	M5.begin();
	M5.Display.setTextSize(2);

	pwmSemaphore = xSemaphoreCreateBinary();

	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, 0);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_PWM);

	mcpwm_config_t pwm_config;
	pwm_config.frequency = 100; // 100Hz,
	pwm_config.cmpr_a = 0; // duty cycle for A
	pwm_config.cmpr_b = 0; // duty cycle for B
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // active high
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,	&pwm_config);

	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000); // 1ms, PWM ON
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 700); // 700us
	MCPWM0.int_ena.val |= BM_INT_TIMER0_TEZ;
	MCPWM0.int_ena.val |= BM_INT_OP0_TEB;
	ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));

    esp_err_t ret;
    ret = esp_adc_cal_check_efuse(ADC_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 0, &adc1_chars);
	}
	//ADC1 config
	ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
	ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN));

	// ------------------------------
	// Setup TensorFlow Lite
	// ------------------------------
	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	model = tflite::GetModel(g_model);

	// Check if model and library have compatible schema version,
	// if not, there is a misalignement between TensorFlow version used
	// to train and generate the TFLite model and the current version of library
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		printf("Model provided and schema version are not equal!\n");
		while(true);
	}
	// This pulls in all the TensorFlow Lite operators.
	static tflite::AllOpsResolver resolver;
	// Build an interpreter to run the model with.
	static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
	interpreter = &static_interpreter;

	// Allocate memory from the tensor_arena for the model's tensors.
	// if an error occurs, stop the program.
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		printf("AllocateTensors() failed\n");
		while(true);
	}
	// Obtain pointers to the model's input and output tensors.
	input = interpreter->input(0);
	output = interpreter->output(0);

	disableCore0WDT();
 	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, PRO_CPU_NUM);
}


void loop() {
	if (st_int == 1){
		st_int = 0;
		delayMicroseconds(100); // after 100us of PWM ON
		digitalWrite(PIN_FLAG1, 1);	
	    v0_ = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC_CHANNEL), &adc1_chars);
		digitalWrite(PIN_FLAG1, 0);
	}
	else if (st_int == 2){
		st_int = 0;
		digitalWrite(PIN_FLAG1, 1);
    	v1 = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC_CHANNEL), &adc1_chars);
		v0 = v0_;
		digitalWrite(PIN_FLAG1, 0);	
	}
}
