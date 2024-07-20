// Solenoid Stroke Measure & Control using CNN
// (c) 2024 Junichi Akita (@akita11, akita@ifdl.jp)
// running TensorFlowLite on ESP32/ESP32-S3
// based on Chirale-TensorFlowLite's example, hello_world
// using M5Stack Core2/S3SE　with library: https://github.com/spaziochirale/Chirale_TensorFlowLite
//
// Copyright 2024 Chirale, TensorFlow Authors. All Rights Reserved.
// Licensed under the Apache License, Version 2.0;

//#define MEASURE_TEMP // measure temp using KmeterISO UNIT

#include <M5Unified.h>
#include "SliderUI.h"
#ifdef MEASURE_TEMP
#include <M5_KMeter.h>
M5_KMeter sensor;
#endif

#include <Chirale_TensorFlowLite.h>

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// generated model file
//#include "model-SSBH-0830-100.h"
//#include "model-CBS0730140-100Hz.h"
#include "model-CH1284123-100.h"

// set PWM manually, and measure position. No control
// #define TEST

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 4000;
uint8_t tensor_arena[kTensorArenaSize];

#if defined(ARDUINO_M5STACK_CORE2)
// for Core2
// for PortA
#define PIN_TXD 32
#define PIN_RXD 33
#define PIN_SDA 14
#define PIN_SCL 13

#elif defined(ARDUINO_M5STACK_CORES3)
// for CoreS3SE
// for PortA
#define PIN_TXD 2
#define PIN_RXD 1
#define PIN_FLAG1 6
#define PIN_SDA 17
#define PIN_SCL 18
#else
#error "No pin definition for this board"
#endif

uint16_t v0, v1;
#ifdef TEST
uint16_t Ton = 1000; // initial PWM duty for debugging
#else
uint16_t Ton = 5000;
#endif
float pos_t = 1.0;
float Kp = 9.0;
#define BUF_LEN 64
char buf[BUF_LEN];
uint8_t pBuf = 0;
uint8_t fValid = 0;

typedef struct solenoid{
  float pos;
  float temp;
} Solenoid;

Solenoid sol;

static constexpr std::size_t slider_count = 2;
static slider_t slider_list[slider_count];

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

/*
// ref: https://moons.link/esp32/post-769/
volatile SemaphoreHandle_t uartSemaphore;
TaskHandle_t taskHandle[2];
void uart_task(void *args){
	while(1){
	}
}

void CreateTasks(void *args) {
	xTaskCreatePinnedToCore(uart_task, "uart_task", 4096, uartSemaphore, 2, &taskHandle[0], APP_CPU_NUM);
	while(1);
}
*/

uint32_t tm0;

void setup() {
	M5.begin();

	Serial2.begin(115200, SERIAL_8N1, PIN_RXD, PIN_TXD);

#ifdef MEASURE_TEMP
  Wire.begin(PIN_SDA, PIN_SCL, 400000L); // SDA/SCL, for PortC (CoreS3)
	sensor.begin(&Wire, 0x66);
#endif

	pinMode(PIN_FLAG1, OUTPUT);

	M5.Display.setTextSize(2);
	// slider for St
	slider_list[0].setup({0, 180, 320, 60}, 0,	500,	100, TFT_WHITE, TFT_BLACK, TFT_LIGHTGRAY);

	// slider for Kp
	slider_list[1].setup({0, 0, 320, 40}, 0,	1000,	(int)(Kp*100), TFT_RED, TFT_BLACK, TFT_RED);

	M5.Display.setEpdMode(epd_mode_t::epd_fastest);
	for (std::size_t i = 0; i < slider_count; ++i) slider_list[i].draw();

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
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		printf("AllocateTensors() failed\n");
		while(true);
	}
	// Obtain pointers to the model's input and output tensors.
	input = interpreter->input(0);
	output = interpreter->output(0);

	delay(1000);
	Serial2.println("START");
	Serial2.println("CYCLE1"); // PWM=100Hz -> every 10ms
/*
	disableCore0WDT();
	uartSemaphore = xSemaphoreCreateBinary();
	// assign timer_task to Core 0 (PRO) (usually used for radio tasks)
//	xTaskCreateUniversal(uart_task, "task1", 8192, NULL, 2, NULL, PRO_CPU_NUM);
 	xSemaphoreGive(uartSemaphore);
	xTaskCreatePinnedToCore(CreateTasks, "CreateTasks", 4096, NULL, 2, &taskHandle[0], PRO_CPU_NUM);
*/
	delay(1000);
	Serial2.println("START");
	Serial2.println("CYCLE1"); // PWM=100Hz -> every 10ms
	tm0 = millis();
}

uint8_t iTon = 0;

uint16_t xt, xt0;
uint16_t pt, pt0;

void loop() {
	M5.update();

	auto t = M5.Touch.getDetail();
#ifdef TEST
	// for test, set Ton by slider
	if (slider_list[1].update(t)) {
		if (slider_list[1].wasChanged()){
			Ton = (float)(slider_list[1].getValue()) * 10; // slider:0-1000 -> 0-10000 (10ms)
			M5.Display.fillRect(0, 40, 320, 20, TFT_BLACK);
			M5.Display.setCursor(0, 40);
			M5.Display.setTextColor(TFT_RED, TFT_BLACK);
			M5.Display.printf("Ton:%d", Ton);
			Serial2.printf("PWMD%d\n", Ton);
		}
	}
#else
	if (slider_list[0].update(t)) {
		if (slider_list[0].wasChanged()) pos_t = (float)(slider_list[0].getValue()) / 100;
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

	if (Serial2.available()){
		char c = Serial2.read();
		if (c == '\n' || c == '\r'){
			buf[pBuf] = '\0';
			if (pBuf == 9){
				// 0000 0000
				buf[4] = '\0';
				v0 = atoi(buf);
				v1 = atoi(buf + 5);
				fValid = 1;
			}
			pBuf = 0;
		}
		else buf[pBuf++] = c;
		if (pBuf == BUF_LEN) pBuf = 0;
	}

	if (fValid == 1){
		// Position Measure
		digitalWrite(PIN_FLAG1, 1);
		sol = predict((float)Ton/1000.0, (float)v0, (float)v1); // 0.5ms
		digitalWrite(PIN_FLAG1, 0); // -1ms
		if (sol.pos < 0) sol.pos = 0.0;
		else if (sol.pos > 5.0) sol.pos = 5.0;
		xt = (uint16_t)((sol.pos / 5.0) * 319);
		M5.Display.drawFastVLine(xt0, 100, 80, TFT_BLACK);
		M5.Display.drawFastVLine(xt,	100, 80, TFT_GREEN);
		pt = (uint16_t)(Ton * 32 / 1000);
		M5.Display.drawFastVLine(pt0, 80, 20, TFT_BLACK);
		M5.Display.drawFastVLine(pt,	80, 20, TFT_CYAN);
		xt0 = xt; pt0 = pt;
		digitalWrite(PIN_FLAG1, 1); // 0.2us
#ifndef TEST
		// Position Control
		int16_t dTon = (uint16_t)((pos_t - sol.pos) * Kp);
		int16_t Ton_t = Ton + dTon;
 #define Ton_MAX 9500
 #define Ton_MIN 1000
		if (Ton_t > Ton_MAX) Ton = Ton_MAX;
		else if (Ton_t < Ton_MIN) Ton = Ton_MIN;
		else Ton = Ton_t;
		Serial2.printf("PWMD%d\n", Ton); // set PWM
 #ifdef MEASURE_TEMP
		sensor.update();
		float temp = sensor.getTemperature();
		printf("%d %.3f %.3f %.3f %.3f\n", millis() - tm0, sol.pos, pos_t, sol.temp, temp);
 #else
		printf("%d %.3f %.3f %.3f\n", millis() - tm0, sol.pos, pos_t, sol.temp);
 #endif
#endif
		fValid = 0;
		digitalWrite(PIN_FLAG1, 0); // -1.2ms in total
	}
/*
	// spends -100ms?
	M5.Display.fillRect(0, 100, 320, 40, TFT_BLACK);
	M5.Display.setCursor(0, 100);
	M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
	M5.Display.printf("S:%.3f / Temp:%.2f\n", sol.pos, sol.temp);
	M5.Display.printf("%4d %4d", v0, v1);
*/
}
