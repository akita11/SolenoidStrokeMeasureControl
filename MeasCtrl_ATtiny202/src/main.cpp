#include <Arduino.h>

// PA0(p6) : UPDI       / D5
// PA1(P4) : ADC (AIN1) / D2
// PA2(p5) : PWM (=WO2) / D3
// PA3(p7) : SW         / D4
// PA6(p2) : TXD        / D0
// PA7(p3) : RXD        / D1

#define pinPWM PIN_PA2
#define pinADC PIN_A1

void setup() {
	Serial.begin(115200);

	// generating PWM: https://qiita.com/ricelectric/items/8909016f22b879acb428
	
	// WO0 : ADC timing of t0 = 100us
	// WO1 : ADC timing of t1 = 700us
	// WO2 : PWM / f=100Hz (cycle 10ms)

	// fsys=20MHz, CLK_PER by MainClockPreecaler (default=1/6)
	CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_10X_gc; // 20MHz / 10 = 2MHz

	pinMode(pinPWM, OUTPUT);
	PORTMUX_CTRLC = PORTMUX_TCA00_DEFAULT_gc; // WO0 to PA2
  
	takeOverTCA0(); //disable and reset the timer
	// TCA clock = fPER / 2 = 1MHz(1us)
	TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV2_gc; //start the timer, set the prescaler to 2
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP2EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc; //enable WO2, select the single slope PWM mode
	TCA0.SINGLE.PER = 9999; // cycle=10000us=10ms (100Hz)
	TCA0.SINGLE.CMP0 = 99; // 100us
	TCA0.SINGLE.CMP1 = 699; //  700us
	TCA0.SINGLE.CMP2 = 0; //duty cycle

	ADC0_CTRLC = ADC_SAMPCAP_bm |	ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV4_gc; // ADC clock = fPER / 4 = 500kHz, Vref=VDD, small sampling capacitance
	ADC0_MUXPOS = ADC_MUXPOS_AIN1_gc; // select AIN1 (PA1)
//	EVSYS_SYNCCH0 = EVSYS_SYNCCH0_TCA0_CMP0_gc; // TCA0.CMP0 as the event source
//	EVSYS_SYNCCH1 = EVSYS_SYNCCH1_TCA0_CMP1_gc; // TCA0.CMP1 as the event source
//	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm; //enable interrupt of CMP0 & CMP1
//	ADC0_EVCTRL = ADC_STARTEI_bm; // enable ADC event trigger
}

void loop() {
//	TCA0.SINGLE.CMP2 = duty--; //change the duty cycle
	Serial.println("Hello, world!");
	delay(1000);
}
