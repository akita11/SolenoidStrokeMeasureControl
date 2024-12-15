#include <Arduino.h>
#include <M5Unified.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <Wire.h>

//#define MEASURE_TEMP

#ifdef MEASURE_TEMP
#include <M5_KMeter.h>
M5_KMeter sensor;
#endif

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

#if defined(ARDUINO_M5STACK_Core2)
// for Core2
// for PortA
#define PIN_PWM 32
#define PIN_ADC 33 // ADC1's Ch.5
// for PortC
#define PIN_FLAG1 13
#define PIN_FLAG2 14
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define ADC_CHANNEL	ADC1_CHANNEL_5 // 32K_XN / GPIO33

#elif defined( ARDUINO_M5STACK_CORES3 )
// for CoreS3SE
// for PortA
#define PIN_PWM 2
#define PIN_ADC 1 // ADC1's Ch.0
// for PortC
#define PIN_FLAG1 6
#define PIN_FLAG2 7
#define ADC_CHANNEL	ADC1_CHANNEL_0 // GPIO1
#define ADC_ATTEN   ADC_ATTEN_DB_12
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#elif defined( ARDUINO_M5STACK_ATOMS3 )
#endif

uint8_t n = 0;
uint8_t N = 16; 
uint32_t v0s, v1s;
uint16_t v0, v1;

uint16_t Ton0[] = {1000, 500}; // [us]
uint16_t Tv1[] = {700, 400}; // [us]
//uint8_t iToni[] = {1, 9, 2, 8, 3, 7, 4, 6, 5};
uint8_t iToni[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
volatile SemaphoreHandle_t pwmSemaphore;
volatile uint8_t st_int = 0;
uint8_t fMeasure = 0;

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

uint32_t nn = 0;
void timer_task(void *pvParameters){
	while(1){
		if (xSemaphoreTake(pwmSemaphore, 0) == pdTRUE) {
			M5.update(); // M5.update() makes jitter of ADC timing
			if (M5.Touch.getDetail().isPressed()){
				fMeasure = 1;
			}
			if (fMeasure == 1){
				int v[9][2][2][5];
				uint8_t iTon, iDelay;;
				uint16_t Ton;
				for (uint8_t ns = 0; ns < 5; ns++){
					for (uint8_t f = 0; f < 2; f++){
						M5.Display.clear(TFT_BLACK);
						M5.Display.setCursor(0, 0);
						M5.Display.printf("n=%d f=%d\n", ns, f);
						mcpwm_config_t pwm_config;
						pwm_config.frequency = 100000/Ton0[f]; // 100Hz,
						pwm_config.cmpr_a = 0; // duty cycle for A
						pwm_config.cmpr_b = 0; // duty cycle for B
						pwm_config.counter_mode = MCPWM_UP_COUNTER;
						pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // active high
						mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,  &pwm_config);
						mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Tv1[f]);
						for (iTon = 0; iTon < 9; iTon++){
							Ton = iToni[iTon] * Ton0[f];
							mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton); // set PWM
							delay(1000);
							v0s = 0; v1s = 0; n = 0;
							delay(2000);
							v[iTon][0][f][ns] = v0;
							v[iTon][1][f][ns] = v1;
#ifdef MEASURE_TEMP
							sensor.update();
							float tmp = sensor.getTemperature();
#endif
							M5.Display.printf("%d,%d,%d,%d\n", Ton0[f], iToni[iTon], v[iTon][0][f][ns], v[iTon][1][f][ns]);
						}
					}
					mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0); // PWM OFF (cool down)
				}
				for (iTon = 0; iTon < 9; iTon++){
					for (uint8_t f = 0; f < 2; f++){
						Ton = iToni[iTon] * Ton0[f];
						printf("%.2f,", (float)Ton/1000);
						uint32_t s0 = 0, s1 = 0;
						for (uint8_t ns = 0; ns < 5; ns++){
							s0 += v[iTon][0][f][ns];
							s1 += v[iTon][1][f][ns];
						}
						printf("%.2f,%.2f,", (float)s0/5.0, (float)s1/5.0);
					}
					for (uint8_t f = 0; f < 2; f++){
						for (uint8_t ns = 0; ns < 5; ns++){
							printf("%d,%d,", v[iTon][0][f][ns], v[iTon][1][f][ns]);
						}
					}
					printf("\n");
				}
				mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0); // PWM OFF
				fMeasure = 0;
				M5.Display.clear(TFT_BLACK);
				M5.Display.setCursor(0, 0);
				M5.Display.printf("BtnA to start\n");
			}
		}
		delay(1);
	}
}

void setup() {
	M5.begin();

	M5.Display.setTextSize(2);   
	M5.Display.clear(TFT_BLACK);
	M5.Display.setCursor(0, 0);
	M5.Display.printf("BtnA to start\n");

	pwmSemaphore = xSemaphoreCreateBinary();

	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, 0);

	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_PWM);

	// Ton0[]=1000/500[us] -> Tcycle=10*Ton[0]=10000/5000[us] -> f=1000000/Tcycle
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 100000/Ton0[0]; // 100Hz,
	pwm_config.cmpr_a = 0; // duty cycle for A
	pwm_config.cmpr_b = 0; // duty cycle for B
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // active high
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,  &pwm_config);

	//	f=Treq[Hz], T=1000/f[ms]
	// 100Hz/200Hz -> 10ms/5ms -> 1ms/0.5ms
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, Ton0[0]); // 10% duyty
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, Tv1[0]); // 700 / 400us

	MCPWM0.int_ena.val |= BM_INT_TIMER0_TEZ; // interrupt at PWM ON
	MCPWM0.int_ena.val |= BM_INT_OP0_TEB;    // interrupt at RegB match
	ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));

	// assign timer_task to Core 0 (PRO) (usually used for radio tasks)
	disableCore0WDT();
//	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, APP_CPU_NUM);
	xTaskCreateUniversal(timer_task, "task1", 8192, NULL, 2/*=priority*/,	NULL, PRO_CPU_NUM);

#ifdef MEASURE_TEMP
    Wire.begin(14, 13, 400000L); // for PortC (Core2)
	sensor.begin(&Wire, 0x66);
#endif

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
	if (st_int == 1){
		st_int = 0;
		delayMicroseconds(100); // after 100us of PWM ON
		digitalWrite(PIN_FLAG1, 1);	
//		v0s += analogReadMilliVolts(PIN_ADC); // cause jitter of ADC timing
    v0s += esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC_CHANNEL), &adc1_chars);
		digitalWrite(PIN_FLAG1, 0);
	}
	else if (st_int == 2){
		st_int = 0;
		digitalWrite(PIN_FLAG1, 1);
    v1s += esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC_CHANNEL), &adc1_chars);
		digitalWrite(PIN_FLAG1, 0);	
		n++;
		if (n == N)
		{
			n = 0;
			v0 = v0s / N; v0s = 0;
			v1 = v1s / N; v1s = 0;
		}
	}
}
