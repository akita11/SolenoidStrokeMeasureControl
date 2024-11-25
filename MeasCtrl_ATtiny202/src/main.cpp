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

uint16_t v0, v1;

#define RX_BUF_SIZE 64
char rxBuf[RX_BUF_SIZE];
#define BUF_LEN 32
char buf[BUF_LEN];
uint8_t pBuf = 0;
uint8_t p_rxBuf = 0;
uint8_t p_rxBufRead = 0;

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

void putDec(uint16_t val) {
  char buf[6];
  itoa(val, buf, 10);
  putString(buf);
}

char getChar() {
  char c;
  if (p_rxBufRead == p_rxBuf) c = -1;
  else{
    c = rxBuf[p_rxBufRead];
    p_rxBufRead = (p_rxBufRead + 1) % RX_BUF_SIZE;
  }
  return c;
}

char isRxAvailable(){
  return p_rxBufRead != p_rxBuf;
}

void putCRLF(){
  putChar('\r'); putChar('\n');
}

ISR(USART0_RXC_vect)
{
	rxBuf[p_rxBuf++] = USART0_RXDATAL;
	if (p_rxBuf == RX_BUF_SIZE) p_rxBuf = 0;
}

ISR(TCA0_CMP0_vect)
{
	digitalWrite(pinSW, 1);
  ADC0_COMMAND = ADC_STCONV_bm; // start conversion
  while(ADC0_COMMAND & ADC_STCONV_bm); // wait for conversion complete
  v0 = ADC0.RES;
	digitalWrite(pinSW, 0);
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm; // clear interrupt flag
}

ISR(TCA0_CMP1_vect)
{
	digitalWrite(pinSW, 1);
  ADC0_COMMAND = ADC_STCONV_bm; // start conversion
  while(ADC0_COMMAND & ADC_STCONV_bm); // wait for conversion complete
  v1 = ADC0.RES;
	digitalWrite(pinSW, 0);
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP1_bm; // clear interrupt flag
}

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
//	VPORTA_DIR |= PIN6_bm; //set pin 6 of PORT A (TXd) as output
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

  // ADC converstion: 50us
	ADC0_CTRLC = ADC_SAMPCAP_bm |	ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc; // ADC clock = fPER / 16 = 625kHz, Vref=VDD, small sampling capacitance
  ADC0_CTRLA = ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm; // 10bit resolution, enable ADC
	ADC0_MUXPOS = ADC_MUXPOS_AIN1_gc; // select AIN1 (PA1)

/*
	// ToDo: EVSYS: CMP0 -> CMP1 -> CMP0 -> CMP1 -> ... is it ok?
	EVSYS_SYNCCH0 = EVSYS_SYNCCH0_TCA0_CMP0_gc; // TCA0.CMP0 as the event source
	EVSYS_SYNCUSER0 = EVSYS_SYNCUSER0_SYNCCH0_gc; // SYNCUSER_0(TCA) as SYNCCH0

	// ToCheck: SYNCUSER1 for TCA? 
	EVSYS_SYNCCH0 = EVSYS_SYNCCH1_TCA0_CMP1_gc; // TCA0.CMP1 as the event source

	// ToCheck: is this needed_
	ADC0_EVCTRL = ADC_STARTEI_bm; // enable ADC event trigger
	ADC0_CTRLA = ADC_ENABLE_bm; // enable ADC
	ADC0_INTCTRL = ADC_RESRDY_bm; // enable ADC result ready interrupt
*/
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm; // enable interrupt of CMP0 & CMP1
	interrupts(); // enable all interrupts
  strcpy(buf,"SolPosMeasCtrl(ATtiny202) v1.0\r\n");
  putString(buf);
}

void loop() {
	// delay(500); // delay uses TCA, don't use it
  asm("nop"); // needed for UART receive?

	if (isRxAvailable()){
		char c = getChar();
		if (c == '\n' || c == '\r'){
			buf[pBuf] = '\0';
			if (strncmp(buf, "MEASURE", 7) == 0){
				putString("Measure start\r\n");
//				fMeasure = 1;
			}
/*
			else if (strncmp(buf, "START", 5) == 0){
				Serial.print("START / "); Serial.println(Ncycle);
				fRun = 1;
			}
			else if (strncmp(buf, "STOP", 4) == 0){
				Serial.println("STOP");
				fRun = 0;
			} 
			else if (strncmp(buf, "PWMD", 4) == 0){
				// set PWM duty [us]
				Ton = atoi(buf + 4);
				Serial.print("Ton = "); Serial.println(Ton);
				SetPWM(Ton, Tcycle, Tv1s);
			}
			else if (strncmp(buf, "PWMT", 4) == 0){
				// set PWM cycle [us]
				Tcycle = atoi(buf + 4);
				Serial.print("Tcycle = "); Serial.println(Tcycle);
				SetPWM(Ton, Tcycle, Tv1s);
			}
			else if (strncmp(buf, "TV1", 3) == 0){
				// set V1 samling point
				Tv1s = atoi(buf + 3);
				Serial.print("Tv1 = "); Serial.println(Tv1s);
				SetPWM(Ton, Tcycle, Tv1s);
			}
			else if (strncmp(buf, "CYCLE", 5) == 0){
				// set output cycle [x PWMcycle]
				Ncycle = atoi(buf + 5);
				Serial.print("Ncycle = "); Serial.println(Ncycle);
			}
*/
			else{
				if (pBuf > 0) putString("?\r\n");
			}
			pBuf = 0;
		}
		else buf[pBuf++] = c;
		if (pBuf == BUF_LEN) pBuf = 0;
  }
}
