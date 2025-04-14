#include <libmaple/libmaple_types.h>                                          //
#include <libmaple/timer.h>   
#include "stm32f10x.h" 
#define MY_TIMER TIMER1                                                       //


const int bufferSize = 128;
float x1[bufferSize];
float x2[bufferSize];

volatile bool bufferReady = false;      // Flag for main loop
volatile bool usingBuffer1 = true;      // Track which buffer is filling
volatile int counter = 0;



void timer_setup(uint16 frequency_interrupt){    

  frequency_interrupt = 1e6/(2*frequency_interrupt);
  

  timer_pause(MY_TIMER);                                                      //
  // Set prescaler (assuming 72MHz clock, 1MHz timer clock)                   //
  timer_set_prescaler(MY_TIMER, 72-1);                                          //
  //uint32_t x = 1000000;                                                     //
  // Set reload value (period_us interval at 1MHz timer clock)                //
  timer_set_reload(MY_TIMER, frequency_interrupt);                                         //
  // Generate an update event to reload the prescaler value immediately       //
  timer_generate_update(MY_TIMER);                                            //
  // Resume the timer                                                         //
  timer_resume(MY_TIMER);                                                     //                                                          
}                                                                             //
void TimerAttachInterrupt(void (*callback)()) {                               //
  // Attach the interrupt callback function                                   //
  timer_attach_interrupt(MY_TIMER, TIMER_UPDATE_INTERRUPT, callback);         //
}   

volatile uint8_t flipflop = 0;
void blinkLed(void){
float sample = analogRead(PA0) - 2048;

	if (usingBuffer1) {
	x1[counter] = sample;
	} else {
	x2[counter] = sample;
	}
	
	digitalWrite(PB10, !digitalRead(PB10));
	counter++;

	if (counter >= bufferSize) {
	counter = 0;
	usingBuffer1 = !usingBuffer1;      // Swap buffer
	bufferReady = true;                // Signal main loop
	timer_detach_interrupt(MY_TIMER, TIMER_UPDATE_INTERRUPT);          // Pause sampling while processing
	}
}


void setup() {
  pinMode(PB10, OUTPUT);
  timer_setup(40000);                                                //
  TimerAttachInterrupt(blinkLed); 
}

void loop() {
  if (bufferReady) {
    // Disable interrupts while accessing shared data
    noInterrupts();
    float* readyBuffer = usingBuffer1 ? x2 : x1;  // The one NOT currently filling
    bufferReady = false;  // Clear the flag
    interrupts();         // Re-enable interrupts

    // Send the buffer over serial
    for (int i = 0; i < bufferSize; i++) {
      Serial.println(readyBuffer[i], 4);
    }

    // Re-enable Timer interrupt to fill the next buffer
    TimerAttachInterrupt(blinkLed);
  }
}
