#include <Arduino.h>
#include <M5Unified.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// using MCPWM
// https://qiita.com/motorcontrolman/items/18abd9738860f6ba5620
// https://rt-net.jp/mobility/archives/10150

// MCPWM0, MCPWM1

// events
// UTEP : count-up, timer=period (PWM_TIMERx_PERIOD)
// UTEZ : count-up, timer=zero
// UTEA : count-up, timer=regA
// UTEB : count-up, timer=regB

//static const mcpwm_unit_t UNIT = MCPWM_UNIT_0;
//#define UNIT MCPWM_UNIT_0

#define BM_INT_TIMER0_TEZ (1 << 3)
#define BM_INT_OP0_TEA    (1 << 15)

#define PIN_FLAG1 32
#define PIN_FLAG2 22

void IRAM_ATTR isr_handler(void *XX)
{
  if (MCPWM0.int_st.val & BM_INT_TIMER0_TEZ){
    // interrupt of Timer0 == 0
    digitalWrite(PIN_FLAG1, 1);
    digitalWrite(PIN_FLAG1, 0);
  }
  if (MCPWM0.int_st.val & BM_INT_OP0_TEA){
    // interrupt of Timer0 == REGA
    digitalWrite(PIN_FLAG2, 1);
    digitalWrite(PIN_FLAG2, 0);
  }

  MCPWM0.int_clr.val = MCPWM0.int_st.val; // clear interrupt flags
}

void setup() {
  pinMode(PIN_FLAG1, OUTPUT); digitalWrite(PIN_FLAG1, 0);
  pinMode(PIN_FLAG2, OUTPUT); digitalWrite(PIN_FLAG2, 0);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_33);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 10*1000; // 10kHz,
  pwm_config.cmpr_a = 0; // duty cycle for A
  pwm_config.cmpr_b = 0; // duty cycle for B
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // active high
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,  &pwm_config);

  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 10);

  MCPWM0.int_ena.val |= BM_INT_TIMER0_TEZ;
  MCPWM0.int_ena.val |= BM_INT_OP0_TEA;
  ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));
}

void loop() {
}
