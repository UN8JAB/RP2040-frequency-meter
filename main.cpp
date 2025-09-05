#include <Arduino.h>
#include <hardware/pwm.h>
#include "FreqCountRP2.h"
#include "hardware/clocks.h"

const int inputPin = 3;     // Frequency counter input pin (odd-numbered GPIO)
const int testPin = 22;     // Test signal output pin 1KHz
const int timerMs = 1000;   // Measurement gate time (ms)

void setup() {
  Serial.begin(57600);
  delay(3000);
  Serial.println("=== RP2040 Generator + Frequency Counter ===");

  // --- Start frequency counter on GPIO3 ---
  FreqCountRP2.beginTimer(inputPin, timerMs);

  // --- Generate 1 kHz test signal on GPIO22 ---
  gpio_set_function(testPin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(testPin);

  pwm_config config = pwm_get_default_config();

  uint32_t sys_clk = clock_get_hz(clk_sys);       // Get actual system clock frequency
  float divider = (float)sys_clk / 1000000.0;     // Scale down to 1 MHz timer ticks
  pwm_config_set_clkdiv(&config, divider);

  pwm_config_set_wrap(&config, 999);              // 1 MHz / 1000 = 1 kHz
  pwm_init(slice_num, &config, true);

  pwm_set_gpio_level(testPin, 500);               // 50% duty cycle
}

void loop() {
  if (FreqCountRP2.available()) {
    unsigned long count = FreqCountRP2.read();
    Serial.print("Measured: ");
    Serial.print(count);
    Serial.println(" Hz");
  }
}
