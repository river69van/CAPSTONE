#include <Arduino.h>
#include <xtensa/core-macros.h>

#define GPS_PPS_pin       34
#define STM_trigger_pin   35

// CPU clock
const uint32_t CPU_HZ = 240000000UL;
const double   time_per_cycle = 1.0 / CPU_HZ;  // = 4.16667e-9

// ISR‑shared state
volatile uint32_t count1   = 0;
volatile uint32_t elapsed  = 0;
volatile bool     test_trig = false;

void IRAM_ATTR GPS_PPS_func() {
  count1 = xthal_get_ccount();
}

void IRAM_ATTR STM_trigger_func() {
  uint32_t now = xthal_get_ccount();
  // handle possible wrap‑around
  if (now >= count1) {
    elapsed = now - count1;
  } else {
    elapsed = (0xFFFFFFFFu - count1) + now + 1;
  }
  test_trig = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(GPS_PPS_pin,     INPUT_PULLUP);
  pinMode(STM_trigger_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(GPS_PPS_pin), GPS_PPS_func, FALLING);
  attachInterrupt(digitalPinToInterrupt(STM_trigger_pin), STM_trigger_func, FALLING);
}

void loop() {
  if (test_trig) {
    // Convert cycles to seconds
    double dt_sec = elapsed * time_per_cycle;
    Serial.println("Since late reset");
    Serial.printf("Elapsed cycles: %u  (%.9f s)\n", elapsed, dt_sec);
    test_trig = false;
  }
}
