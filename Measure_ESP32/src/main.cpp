#include <M5Unified.h>

// work with SolenoidStrokeUNIT v1.0 (with ATtiny202)

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
#define PIN_TXD 1
#define PIN_RXD 2
#define PIN_FLAG1 6
// for Port C
#define PIN_SDA 17
#define PIN_SCL 18
//USBCDC USBSerial;

#else
#error "No pin definition for this board"
#endif

volatile uint8_t fValid = 0;
uint16_t ns = 0;
uint8_t iTon = 0;
uint8_t iToni[] = {1, 9, 2, 8, 3, 7, 4, 6, 5};
uint16_t v0s = 0, v1s = 0;
uint16_t v0, v1;
#define BUF_LEN 64
char buf[BUF_LEN];
uint8_t pBuf = 0;

void setPWM(uint8_t Ton) // [ms]
{
	Serial2.printf("D%d\n", Ton * 1250); // D=[0.8us]
}

void setup() {
	M5.begin();

	Serial2.begin(115200, SERIAL_8N1, PIN_RXD, PIN_TXD);
	Serial.begin(115200);

	M5.Display.setTextSize(2);

	delay(500);
	Serial2.println("P");
	Serial2.println("D1"); // PWM off (idle)

	M5.Display.println("Touch to start");
}

#define N_SAMPLE 10

void startMeasurement()
{
	M5.Display.clearDisplay();
	M5.Display.setCursor(0, 0);
	iTon = 0;
	setPWM(iTon[iToni]); delay(100);
	ns = 0; v0s = 0; v1s = 0;
	Serial2.println('S');
}

void loop() {
	M5.update();

	auto t = M5.Touch.getDetail();
	if (t.isPressed()){
		startMeasurement();
	}

	if (fValid == 1){
		v0s += v0; v1s += v1;
		fValid = 0;
		ns++;
		if (ns == N_SAMPLE){
			Serial2.println('P');
			fValid = 0; // skip remaining data
			while(fValid == 1) delay(10); // skip remaining data
			Serial.printf("%d %.2f %.2f\n", iToni[iTon], (float)v0s / (float)N_SAMPLE, (float)v1s / (float)N_SAMPLE);
			M5.Display.printf("%d %.2f %.2f\n", iToni[iTon], (float)v0s / (float)N_SAMPLE, (float)v1s / (float)N_SAMPLE);
			ns = 0; v0s = 0; v1s = 0;
			iTon++;
			if (iTon == 9){
				Serial2.println("P");
				Serial2.println("D1"); // PWM off (idle)
			}
			else{
				setPWM(iToni[iTon]);
				Serial2.println('S');
			}
		}
	}


	if (Serial.available()){
		char c = Serial.read();
		if (c == '\n' || c == '\r'){
			buf[pBuf] = '\0';
			if (buf[0] == 'S'){
				startMeasurement();
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
			if (pBuf == 8){
				// 11112222
				v1 = atoi(buf + 4);
				buf[4] = '\0';
				v0 = atoi(buf);
				fValid = 1;
			}
			pBuf = 0;
		}
		else buf[pBuf++] = c;
		if (pBuf == BUF_LEN) pBuf = 0;
	}

}
