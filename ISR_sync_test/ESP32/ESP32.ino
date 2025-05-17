/*
#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include "soc/rtc.h"           // for rtc_clk_cpu_freq_get()
#include "esp32/rom/ets_sys.h" // for xthal_get_ccount()
#include "soc/soc.h"           // pulls in common SoC macros
#include "soc/gpio_struct.h"   // defines the gpio_dev_t struct and extern 'GPIO'
#include <driver/gpio.h>
#include <esp_intr_alloc.h>

#define LOW_PRIO_FLAGS   (ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM)
#define HIGH_PRIO_FLAGS  (ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM)

const gpio_num_t PIN_LOW  = GPIO_NUM_4;
const gpio_num_t PIN_HIGH = GPIO_NUM_5;

volatile uint32_t start_t = 0;
volatile uint32_t elapsed_t = 0;
volatile bool     new_data = false;

static void IRAM_ATTR low_handler(void* arg) {
    uint32_t pin = (uint32_t)arg;
    uint32_t intr_status = REG_READ(GPIO_STATUS_REG);
    REG_WRITE(GPIO_STATUS_W1TC_REG, intr_status);

    if (intr_status & (1u << pin)) {
        //start_ccount = xthal_get_ccount();
    }

    //start_t = micros();
    new_data = true;
}

static void IRAM_ATTR high_handler(void* arg) {
    uint32_t pin = (uint32_t)arg;
    uint32_t intr_status = REG_READ(GPIO_STATUS_REG);
    REG_WRITE(GPIO_STATUS_W1TC_REG, intr_status);

    if (intr_status & (1u << pin)) {
        //elapsed_cycles = xthal_get_ccount() - start_ccount;
        
    }
    //uint32_t now = micros();
    //elapsed_t = now - start_t;
    new_data = true;
}


void setup() {
    //start_t = micros();
    Serial.begin(115200);

    // Configure both pins as inputs with pullup
    pinMode(PIN_LOW,  INPUT_PULLUP);
    pinMode(PIN_HIGH, INPUT_PULLUP);
    gpio_set_intr_type(PIN_LOW,  GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(PIN_HIGH, GPIO_INTR_NEGEDGE);

    // Allocate the LOW‐priority ISR first
    intr_handle_t low_handle;
    esp_intr_alloc(
        ETS_GPIO_INTR_SOURCE,
        LOW_PRIO_FLAGS,
        low_handler,
        (void*)PIN_LOW,
        &low_handle
    );

    // Then allocate the HIGH‐priority ISR
    intr_handle_t high_handle;
    esp_intr_alloc(
        ETS_GPIO_INTR_SOURCE,
        HIGH_PRIO_FLAGS,
        high_handler,
        (void*)PIN_HIGH,
        &high_handle
    );

    // Finally, enable both GPIO interrupts in hardware
    gpio_intr_enable(PIN_LOW);
    gpio_intr_enable(PIN_HIGH);
}

void loop() {
    if (new_data) {
        //uint32_t us = cycles_to_us(elapsed_cycles);
        Serial.println("elapseed: ");
        Serial.println(elapsed_t);
        new_data = false;
    }
}
*/


#include "soc/soc.h"        // for REG_READ / REG_WRITE
#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include "esp32/rom/ets_sys.h"  // for xthal_get_ccount()
#include "soc/rtc.h"           // for rtc_clk_cpu_freq_get()

#define LOW_PRIO_FLAGS   (ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM)
#define HIGH_PRIO_FLAGS  (ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM)

const gpio_num_t PIN_LOW  = GPIO_NUM_4;
const gpio_num_t PIN_HIGH = GPIO_NUM_5;

volatile uint32_t start_ccount   = 0;
volatile uint32_t elapsed_cycles = 0;
volatile bool     new_data       = false;

// Helper: convert cycles → microseconds
static inline uint32_t cycles_to_us(uint32_t cycles) {
    uint32_t fcpu = 240000000UL;
    return (uint32_t)((uint64_t)cycles * 1000000 / fcpu);
}

static void IRAM_ATTR low_handler(void* arg) {
    uint32_t pin = (uint32_t)arg;
    uint32_t intr_status = REG_READ(GPIO_STATUS_REG);
    REG_WRITE(GPIO_STATUS_W1TC_REG, intr_status);

    if (intr_status & (1u << pin)) {
        start_ccount = xthal_get_ccount();
    }
    //new_data = true;
}

static void IRAM_ATTR high_handler(void* arg) {
    uint32_t pin = (uint32_t)arg;
    uint32_t intr_status = REG_READ(GPIO_STATUS_REG);
    REG_WRITE(GPIO_STATUS_W1TC_REG, intr_status);

    if (intr_status & (1u << pin)) {
        elapsed_cycles = xthal_get_ccount() - start_ccount;
        new_data = true;
    }
    new_data = true;
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_LOW,  INPUT_PULLUP);
    pinMode(PIN_HIGH, INPUT_PULLUP);
    gpio_set_intr_type(PIN_LOW,  GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(PIN_HIGH, GPIO_INTR_NEGEDGE);

    intr_handle_t hLow;
    esp_intr_alloc(
      ETS_GPIO_INTR_SOURCE,
      LOW_PRIO_FLAGS,
      low_handler,
      (void*)PIN_LOW,
      &hLow
    );

    intr_handle_t hHigh;
    esp_intr_alloc(
      ETS_GPIO_INTR_SOURCE,
      HIGH_PRIO_FLAGS,
      high_handler,
      (void*)PIN_HIGH,
      &hHigh
    );

    gpio_intr_enable(PIN_LOW);
    gpio_intr_enable(PIN_HIGH);
}

void loop() {
    if (new_data) {
        //uint32_t us = cycles_to_us(elapsed_cycles);
        Serial.println("hehe: ");
        Serial.println(cycles_to_us(elapsed_cycles));
        new_data = false;
    }
}

