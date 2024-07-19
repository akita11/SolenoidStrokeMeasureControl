#include <Arduino.h>
#include <M5Unified.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

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

// for ATOM-S3
// for PortA
#define PIN_PWM 2
#define PIN_ADC 1 // ADC1's Ch.0

#define PIN_FLAG1 5
#define ADC_CHANNEL	ADC1_CHANNEL_0 // GPIO1
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT

uint8_t n = 0;
uint32_t v0s, v1s;
uint16_t v0, v1;

uint16_t Ton0[] = {1000, 500}; // [us]
uint16_t Tv1[] = {700, 400}; // [us]
//uint8_t iToni[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
uint8_t iToni[] = {1, 9, 2, 8, 3, 7, 6, 4, 5};
volatile SemaphoreHandle_t pwmSemaphore;
volatile uint8_t st_int = 0;
uint8_t fMeasure = 0;
uint8_t fRun = 0;
uint8_t fReady = 0;

static esp_adc_cal_characteristics_t adc1_chars;

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

#define BUF_LEN 64
char buf[BUF_LEN];
uint8_t pBuf = 0;

uint16_t Ton = 1000; // 1000us=1ms
uint16_t Tcycle = 10000; // 10,000us=10ms / 100Hz
uint16_t Tv1s = 700; // 700us
uint8_t Ncycle = 10; // 100Hz / 10 = 10Hz

void SetPWM(uint16_t Ton, uint16_t Tcycle, uint16_t Tv1s)
{
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 1000000/Tcycle;
//	pwm_config.cmpr_a = 0; // duty cycle for A
//	pwm_config.cmpr_b = 0; // duty cycle for B
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // active high
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,  &pwm_config);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Tv1s);
}

void timer_task(void *pvParameters){
	while(1){
		if (xSemaphoreTake(pwmSemaphore, 0) == pdTRUE) {
			if (st_int == 1){
				st_int = 0;
				delayMicroseconds(100); // after 100us of PWM ON
				digitalWrite(PIN_FLAG1, 1);	
				// analogReadMilliVolts() cause jitter of ADC timing
		   		v0s += esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC_CHANNEL), &adc1_chars);
				digitalWrite(PIN_FLAG1, 0);
			}
			else if (st_int == 2){
				st_int = 0;
				digitalWrite(PIN_FLAG1, 1);
			    v1s += esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC_CHANNEL), &adc1_chars);
				digitalWrite(PIN_FLAG1, 0);	
				n++;
				if (n == Ncycle){
					n = 0;
					v0 = v0s / Ncycle; v0s = 0;
					v1 = v1s / Ncycle; v1s = 0;
					fReady = 1;
				}
			}
		}
	}
}

void setup() {
	M5.begin();
	USBSerial.begin(115200);
  	Serial2.begin(115200, SERIAL_8N1, 38, 39); // RX/TX

	Serial2.println("SolenoidMeasureControl v1.0\r\n");

	M5.Display.setTextSize(1);   
	M5.Display.clear(TFT_BLACK);
	M5.Display.setCursor(0, 0);
	M5.Display.printf("Ready\n");

	pwmSemaphore = xSemaphoreCreateBinary();

	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, 0);

	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_PWM);

	SetPWM(Ton, Tcycle, Tv1s);

	MCPWM0.int_ena.val |= BM_INT_TIMER0_TEZ; // interrupt at PWM ON
	MCPWM0.int_ena.val |= BM_INT_OP0_TEB;    // interrupt at RegB match
	ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));

	// assign timer_task to Core 0 (PRO) (usually used for radio tasks)
	disableCore0WDT();
//	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, APP_CPU_NUM);
	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, PRO_CPU_NUM);

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

}

uint8_t st = 0;

void loop()
{
	if (USBSerial.available()){
		char c = USBSerial.read();
		if (c == '\n' || c == '\r'){
			buf[pBuf] = '\0';
			if (strncmp(buf, "MEASURE", 6) == 0){
				Serial2.printf("Measure start\r\n");
				fMeasure = 1;
			}
			else{
				if (pBuf > 0) Serial2.printf("?\r\n");
			}
			pBuf = 0;
		}
		else buf[pBuf++] = c;
		if (pBuf == BUF_LEN) pBuf = 0;
	}
	if (Serial2.available()){
		char c = Serial2.read();
		if (c == '\n' || c == '\r'){
			buf[pBuf] = '\0';
			if (strncmp(buf, "MEASURE", 6) == 0){
				Serial2.printf("Measure start\r\n");
				fMeasure = 1;
			}
			else if (strncmp(buf, "START", 5) == 0){
				Serial2.printf("START / %d\r\n", Ncycle);
				fRun = 1;
			}
			else if (strncmp(buf, "STOP", 4) == 0){
				Serial2.printf("STOP\r\n");
				fRun = 0;
			} 
			else if (strncmp(buf, "PWMD", 4) == 0){
				// set PWM duty [us]
				Ton = atoi(buf + 4);
				Serial2.printf("Ton = %d\r\n", Ton);
				printf("Ton=%d\n", Ton);
				SetPWM(Ton, Tcycle, Tv1s);
			}
			else if (strncmp(buf, "PWMT", 4) == 0){
				// set PWM cycle [us]
				Tcycle = atoi(buf + 4);
				Serial2.printf("Tcycle = %d\r\n", Tcycle);
				printf("PWM cycle=%d\n", Tcycle);
				SetPWM(Ton, Tcycle, Tv1s);
			}
			else if (strncmp(buf, "TV1", 3) == 0){
				// set V1 samling point
				Tv1s = atoi(buf + 3);
				Serial2.printf("Tv1 = %d\r\n", Tv1s);
				printf("V1 sampling point=%d\n", Tv1s);
				SetPWM(Ton, Tcycle, Tv1s);
			}
			else if (strncmp(buf, "CYCLE", 5) == 0){
				// set output cycle [x PWMcycle]
				Ncycle = atoi(buf + 5);
				Serial2.printf("Ncycle = %d\r\n", Ncycle);
				printf("Ncycle=%d\n", Ncycle);
			}
			else{
				if (pBuf > 0) Serial2.printf("?\r\n");
			}
			pBuf = 0;
		}
		else buf[pBuf++] = c;
		if (pBuf == BUF_LEN) pBuf = 0;
	}
	#define N_SAMPLE 5
	if (fMeasure == 1){
		int v[9][2][2][5];
		uint8_t iTon, iDelay;;
		uint16_t Ton;
		for (uint8_t ns = 0; ns < N_SAMPLE; ns++){
			for (uint8_t f = 0; f < 2; f++){
				M5.Display.clear(TFT_BLACK);
				M5.Display.setCursor(0, 0);
				M5.Display.printf("n=%d f=%d\n", ns, f);
				for (iTon = 0; iTon < 9; iTon++){
					Ton = iToni[iTon] * Ton0[f];
					SetPWM(Ton, Ton0[f] * 10, Tv1[f]);
					delay(1000);
					fReady = 0;
					while(fReady == 0) delayMicroseconds(100);
					v[iTon][0][f][ns] = v0;
					v[iTon][1][f][ns] = v1;
					M5.Display.printf("%d,%d,%d,%d\n", Ton0[f], iToni[iTon], v[iTon][0][f][ns], v[iTon][1][f][ns]);
				}
			}
			SetPWM(0, Tcycle, Tv1s);
		}
		for (iTon = 0; iTon < 9; iTon++){
			for (uint8_t f = 0; f < 2; f++){
				Ton = iToni[iTon] * Ton0[f];
				printf("%.2f,", (float)Ton/1000);
				uint32_t s0 = 0, s1 = 0;
				for (uint8_t ns = 0; ns < N_SAMPLE; ns++){
					s0 += v[iTon][0][f][ns];
					s1 += v[iTon][1][f][ns];
				}
				printf("%.2f,%.2f,", (float)s0/(float)N_SAMPLE, (float)s1/(float)N_SAMPLE);
			}
			for (uint8_t f = 0; f < 2; f++){
				for (uint8_t ns = 0; ns < N_SAMPLE; ns++){
					printf("%d,%d,", v[iTon][0][f][ns], v[iTon][1][f][ns]);
				}
			}
			printf("\n");
		}
		Serial2.printf("Measure done\r\n");
		SetPWM(0, Tcycle, Tv1s);
		fMeasure = 0;
		M5.Display.clear(TFT_BLACK);
		M5.Display.setCursor(0, 0);
		M5.Display.printf("Ready\n");
	}
	if (fRun == 1){
		if (fReady == 1){
			fReady = 0;
			Serial2.printf("%d %d\r\n", v0, v1);
		}
	}
}
