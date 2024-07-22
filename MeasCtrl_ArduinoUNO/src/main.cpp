#include <Arduino.h>

#define PIN_FLAG1 2
#define PIN_ADC A0
#define PIN_PWM 9
#define PIN_SW 4
#define PIN_LED 13

uint8_t n = 0;
uint32_t v0s, v1s;
uint16_t v0, v1;

uint16_t Ton0[] = {1000, 500}; // [us]
uint16_t Tv1[] = {700, 400}; // [us]
uint8_t iToni[] = {1, 9, 2, 8, 3, 7, 6, 4, 5};
uint8_t fMeasure = 0;
uint8_t fRun = 0;
uint8_t fReady = 0;

#define BUF_LEN 64
char buf[BUF_LEN];
uint8_t pBuf = 0;

uint16_t Ton = 1000; // 1000us=1ms
uint16_t Tcycle = 10000; // 10,000us=10ms / 100Hz
uint16_t Tv1s = 700; // 700us
uint8_t Ncycle = 10; // 100Hz / 10 = 10Hz

void SetPWM(uint16_t Ton, uint16_t Tcycle, uint16_t Tv1s)
{
	ICR1 = Tcycle * 2 - 1;		 // 2MHz/20000=100Hz(10ms) / TOP
	OCR1A = Ton * 2 - 1; // PWM Duty Cycle
	OCR1B = Tv1s * 2 - 1;		 // 0.5us(2MHz)*1400 = 0.7ms
}

// Timer1 overflow interrupt (=PWM ON)
ISR(TIMER1_OVF_vect)
{
	delayMicroseconds(100);
	PORTD |= _BV(PD2); // D2
//	v0s += analogRead(PIN_ADC);
	ADCSRA |= 0x40; // start conversion
	while(ADCSRA & 0x40); // wait for ADC convesion
	v0s += ADC;
	PORTD &= ~(_BV(PD2));
}

// Timer1 CompareMatchB interrupt
ISR(TIMER1_COMPB_vect)
{
	PORTD |= _BV(PD2);
//	v1s += analogRead(PIN_ADC);
	ADCSRA |= 0x40; // start conversion
	while(ADCSRA & 0x40); // wait for ADC convesion
	v1s += ADC;
	PORTD &= ~(_BV(PD2));
	n++;
	if (n == Ncycle)
	{
		n = 0;
		v0 = v0s / Ncycle; v0s = 0;
		v1 = v1s / Ncycle; v1s = 0;
		fReady = 1;
	}
}

void setup() {
	pinMode(PIN_PWM, OUTPUT);
	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, LOW);
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);
	Serial.begin(115200);
	// setup Timer1
	TCCR1A = _BV(COM1A1) | _BV(WGM11);			  // Ch.A: non-inverting, WGM11=1 (mode14, Fast PWM)
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // WGM13:12=11, CS11=1 (16MHz/8 = 2MHz)

	SetPWM(Ton, Tcycle, Tv1s);

	// setup ADC
	ADMUX = 0x40; // Ref=AVCC, LeftAdj=0, MUX=0
	ADCSRA = 0x80; // enable ADC

	// enable interrupts
	TIMSK1 |= _BV(TOIE1);  // enable Timer1 OVF interrupt (=PWM ON)
	TIMSK1 |= _BV(OCIE1B); // enable Timer1 COMPB interrupt
	sei();				   // enable global interrupt
}

void loop()
{
/*
	if (Serial.available()){
		char c = Serial.read();
		if (c == '\n' || c == '\r'){
			buf[pBuf] = '\0';
			if (strncmp(buf, "MEASURE", 7) == 0){
				Serial.println("Measure start");
				fMeasure = 1;
			}
			else if (strncmp(buf, "PWMD", 4) == 0){
				// set PWM duty [us]
				Ton = atoi(buf + 4);
				Serial.print("Ton = "; Serial.println(Ton);
				SetPWM(Ton, Tcycle, Tv1s);
			}
			else{
				if (pBuf > 0) Serial.println("?");
			}
			pBuf = 0;
		}
		else buf[pBuf++] = c;
		if (pBuf == BUF_LEN) pBuf = 0;
	}
*/
	if (Serial.available()){
		char c = Serial.read();
		if (c == '\n' || c == '\r'){
			buf[pBuf] = '\0';
			if (strncmp(buf, "MEASURE", 7) == 0){
				Serial.println("Measure start");
				fMeasure = 1;
			}
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
			else{
				if (pBuf > 0) Serial.println("?");
			}
			pBuf = 0;
		}
		else buf[pBuf++] = c;
		if (pBuf == BUF_LEN) pBuf = 0;
	}
/*
	#define N_SAMPLE 5
	if (fMeasure == 1){
		int v[9][2][2][5];
		uint8_t iTon, iDelay;;
		uint16_t Ton;
		for (uint8_t ns = 0; ns < N_SAMPLE; ns++){
			for (uint8_t f = 0; f < 2; f++){
				M5.Display.clear(TFT_RED);
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
*/
	if (fRun == 1){
		if (fReady == 1){
			fReady = 0;
			if (v0 < 1000) Serial.print('0');
			if (v0 < 100) Serial.print('0');
			if (v0 < 10) Serial.print('0');
			Serial.print(v0);
			Serial.print(' ');
			if (v1 < 1000) Serial.print('0');
			if (v1 < 100) Serial.print('0');
			if (v1 < 10) Serial.print('0');
			Serial.println(v1);
		}
	}
}
