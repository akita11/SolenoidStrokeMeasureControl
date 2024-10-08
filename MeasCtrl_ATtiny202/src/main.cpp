#include <Arduino.h>

// PA0(p6) : UPDI       / D5
// PA1(P4) : ADC (AIN1) / D2
// PA2(p5) : PWM (=WO2) / D3
// PA3(p7) : SW         / D4
// PA6(p2) : TXD        / D0
// PA7(p3) : RXD        / D1

#define pinPWM PIN_PA2
#define pinADC PIN_A1
#define pinSW  PIN_PA3
#define pinTXD PIN_PA6
#define pinRXD PIN_PA7

uint8_t fADC = 0;
uint16_t v0, v1;

#define RX_BUF_SIZE 64
char rxBuf[RX_BUF_SIZE];
uint8_t p_rxBuf = 0;

// set PWM duty [*0.8us]
void SetPWM(uint16_t duty)
{
	if (duty >= TCA0.SINGLE.PER) duty = TCA0.SINGLE.PER;
	TCA0.SINGLE.CMP2 = duty;
}

void putChar(char c){
	while (!(USART0_STATUS & USART_DREIF_bm)); // wait for TX complete
	USART0_TXDATAL = c;
}
void putString(char *str) {
  for (size_t i = 0; i < strlen(str); i++) putChar(str[i]);
}


ISR(USART0_RXC_vect)
{
	rxBuf[p_rxBuf++] = USART0_RXDATAL;
	if (p_rxBuf == RX_BUF_SIZE) p_rxBuf = 0;
}

ISR(ADC0_RESRDY_vect)
{
	digitalWrite(pinSW, 1);
	if (fADC == 0) v0 = ADC0.RES;
	else v1 = ADC0.RES;
	fADC = 1 - fADC;
	digitalWrite(pinSW, 0);
}

//	ISR(TCA0_CMP0_vect)
//	ISR(TCA0_CMP1_vect)

// Workflow
// PWM ___~~~~~~~~~~~~~_______~~~~
// TCA    0 |    |    |       0
//         CMP0 CMP1 CMP2    PER
// ADC      |v0  |v1

// f_cpu in platformio.ini : https://qiita.com/ji6czd/items/ed307a21541f906165a5

#define F_CPU 10000000L // 20MHz / 2 = 10MHz
// using UART: from https://forum.arduino.cc/t/attiny202-uart-in-100-bytes/1247790/4
#define BAUDRATE 115200
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(64 * F_CPU / (16 * (float)BAUD_RATE)) + 0.5)

void setup() {
	// fMAIN=10MHz @ OSC=20MHz
  _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, (CLKCTRL_PEN_bm | CLKCTRL_PDIV_2X_gc));
//	pinMode(pinSW, INPUT_PULLUP); // for SW
	pinMode(pinSW, OUTPUT); // for debug

	// setup UART
	pinMode(pinTXD, OUTPUT);
//	VPORTA_DIR |= PIN6_bm; //set pin 6 of PORT A (TXd) as output*/
	USART0_BAUD = (uint16_t)(USART0_BAUD_RATE(BAUDRATE)); // set baud rate
	USART0_CTRLC = USART_CHSIZE0_bm | USART_CHSIZE1_bm; // N81N
	USART0_CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable TX&RX
	USART0_CTRLA = USART_RXCIE_bm; // enable RXD interrupt

	//setup PWM / TCA
	// generating PWM: https://qiita.com/ricelectric/items/8909016f22b879acb428
	// WO0 : ADC timing of t0 = 100us
	// WO1 : ADC timing of t1 = 700us
	// WO2 : PWM / f=100Hz (cycle 10ms)
	pinMode(pinPWM, OUTPUT);
	takeOverTCA0(); //disable and reset the timer
	// TCA clock = fPER / 8 = 1.25MHz(0.8us)
	TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV8_gc; // enabel TCA, div=8
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP2EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc; // enable WO2, single slope PWM
	TCA0.SINGLE.PER = 12499; // cycle=10000us=10ms (100Hz)
	TCA0.SINGLE.CMP0 = 124;  // 100us for v0
	TCA0.SINGLE.CMP1 = 874; // 700us for v1
	TCA0.SINGLE.CMP2 = 1249;   // PWM duty cycle

	ADC0_CTRLC = ADC_SAMPCAP_bm |	ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc; // ADC clock = fPER / 16 = 625kHz, Vref=VDD, small sampling capacitance
	ADC0_MUXPOS = ADC_MUXPOS_AIN1_gc; // select AIN1 (PA1)

	// ToDo: EVSYS: CMP0 -> CMP1 -> CMP0 -> CMP1 -> ... is it ok?
	EVSYS_SYNCCH0 = EVSYS_SYNCCH0_TCA0_CMP0_gc; // TCA0.CMP0 as the event source
	EVSYS_SYNCUSER0 = EVSYS_SYNCUSER0_SYNCCH0_gc; // SYNCUSER_0(TCA) as SYNCCH0

	// ToCheck: SYNCUSER1 for TCA? 
//	EVSYS_SYNCCH0 = EVSYS_SYNCCH1_TCA0_CMP1_gc; // TCA0.CMP1 as the event source

	// ToCheck: is this needed_
//	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm; // enable interrupt of CMP0 & CMP1
	ADC0_EVCTRL = ADC_STARTEI_bm; // enable ADC event trigger
	ADC0_CTRLA = ADC_ENABLE_bm; // enable ADC
	ADC0_INTCTRL = ADC_RESRDY_bm; // enable ADC result ready interrupt
	interrupts(); // enable all interrupts
}

void loop() {
	putChar(0x55);
	// delay(500); // delay uses TCA
}
