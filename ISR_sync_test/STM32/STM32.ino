#include <libmaple/timer.h> 
#define MY_TIMER TIMER1                                                       

const int pin1 = PB10;
const int pin2 = PB11; 
bool flip_flop1 = false;
bool flip_flop2 = false;
unsigned long previousMillis = 0;

void run_func(void){
  if(!flip_flop2){

    flip_flop1 ^= true;

  }else{

    flip_flop1 = false;

  }
}


void timer_setup(uint32_t p_time){                                            //
  p_time = p_time/2;
  timer_pause(MY_TIMER);                                                      //
  // Set prescaler (assuming 72MHz clock, 1MHz timer clock)                   //
  timer_set_prescaler(MY_TIMER, 70);                                        //
  //uint32_t x = 1000000;                                                     //
  // Set reload value (period_us interval at 1MHz timer clock)                //
  timer_set_reload(MY_TIMER, p_time);                                         //
  // Generate an update event to reload the prescaler value immediately       //
  timer_generate_update(MY_TIMER);                                            //
  // Resume the timer                                                         //
  timer_resume(MY_TIMER);                                                     //                                                          
}                                                                       

void TimerAttachInterrupt(void (*callback)()) {                             

  // Attach the interrupt callback function                                   //
  timer_attach_interrupt(MY_TIMER, TIMER_UPDATE_INTERRUPT, callback);         //

}  



void setup(){
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  timer_setup(1000000);
  TimerAttachInterrupt(run_func);
}



void loop(){

  unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= 1000){
      previousMillis = currentMillis;

      flip_flop2 ^= true;

    }
  
  digitalWrite(pin2, flip_flop2); //pan ISR2
  digitalWrite(pin1, flip_flop1); //pan ISR1

}