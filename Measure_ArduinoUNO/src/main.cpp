#include <Arduino.h>

#define PIN_FLAG1 2
#define PIN_FLAG2 3

void setup()
{
	pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, LOW);
	pinMode(PIN_FLAG2, OUTPUT); digitalWrite(PIN_FLAG2, LOW);
	Serial.begin(115200);
	// Timer1 設定
	TCCR1A = _BV(COM1A1) | _BV(WGM11);			  // 非反転モードでチャネルAを設定し、WGM11=1 (モード14, Fast PWM)
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // WGM13:12=11, CS11=1 (プリスケーラ8 = 2MHz)

	// ICR1 は PWM のトップ値を設定
	ICR1 = 19999; // 2MHz/20000=1000Hz(10ms)

	OCR1A = 1999; // 2MHz/2000=1kHz / 1ms / PWM Duty Cycle
	OCR1B = 999; // 2MHz/1000=2kHz / 0.5ms

	// 割り込み設定
	TIMSK1 = _BV(TOIE1);  // enable Timer1 OVF interrupt
	TIMSK1 = _BV(OCIE1B); // enable Timer1 COMPB interrupt 

	sei(); // グローバル割り込みを有効化
}
// note: TCNT=OCR1AのときにTimer1_COMPA割り込み

// Timer1 のオーバーフロー割り込み
ISR(TIMER1_OVF_vect)
{
	digitalWrite(PIN_FLAG1, 1);
	digitalWrite(PIN_FLAG1, 0);
}

// Timer1 のCompareMatchB割り込み
ISR(TIMER1_COMPB_vect)
{
	digitalWrite(PIN_FLAG2, 1);
	digitalWrite(PIN_FLAG2, 0);
}

void loop()
{
}
