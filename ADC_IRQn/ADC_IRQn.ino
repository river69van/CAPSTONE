
#define USE_STDPERIPH_DRIVER
#define STM32F10X_MD
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "misc.h"            // NVIC

#define ADC_BUFFER_SIZE   1024
#define THRESHOLD_LOW     1000    // 12‑bit: 0..4095
#define THRESHOLD_HIGH    3000

volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t  awd_flag = 0;

void tim3_trigger_init(uint32_t sample_rate_hz) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // 72MHz / (PSC+1) = 1MHz, so PSC = 71
    TIM_TimeBaseInitTypeDef  tb;
    tb.TIM_Prescaler     = 71;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_Period        = (1000000 / sample_rate_hz) - 1;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tb);

    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);  // TRGO = update event
    TIM_Cmd(TIM3, ENABLE);
}

void adc_dma_init(void) {
    GPIO_InitTypeDef  gpio;
    DMA_InitTypeDef   dma;
    ADC_InitTypeDef   adc;
    //NVIC_InitTypeDef  nvic;

    // 1) Clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,       ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 |
                           RCC_APB2Periph_GPIOA,   ENABLE);

    // 2) PA0 = ADC1_IN0
    gpio.GPIO_Pin  = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    // 3) DMA1 Channel1: ADC1→adc_buffer circular
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr     = (uint32_t)adc_buffer;
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize         = ADC_BUFFER_SIZE;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // 4) ADC1: one channel, external TRGO, DMA enabled
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &adc);

    
    


    // 5) Channel0, sample time
    ADC_RegularChannelConfig(
        ADC1,
        ADC_Channel_0,
        1,
        ADC_SampleTime_71Cycles5);

    // 6) Enable DMA for ADC
    ADC_DMACmd(ADC1, ENABLE);
    
    

    // 7) **Analog watchdog** thresholds & single‐channel
    ADC_AnalogWatchdogThresholdsConfig(ADC1, THRESHOLD_HIGH, THRESHOLD_LOW);
    ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_0);
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);


    //debug
  

	
    // 8) Enable AWD interrupt
	
	ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);       // turn on AWD IRQ in the ADC
	ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);      // clear any “stuck” AWD flag
	NVIC_ClearPendingIRQ(ADC1_2_IRQn);            // clear NVIC pending bit

	NVIC_InitTypeDef nvic = {
		.NVIC_IRQChannel                   = ADC1_2_IRQn,
		.NVIC_IRQChannelPreemptionPriority = 1,
		.NVIC_IRQChannelSubPriority        = 0,
		.NVIC_IRQChannelCmd                = ENABLE,
	};
	
	

    // 10) Turn on, calibrate
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // 11) Enable external trigger
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	
	
}


void setup() {
    Serial.begin(115200);
    tim3_trigger_init(10000);   // 10 kHz sampling
    adc_dma_init();


}

void loop() {

  if (awd_flag) {
      awd_flag = 0;
      Serial.println("Analog‐watchdog threshold crossed!");
  }

  if (ADC_GetFlagStatus(ADC1, ADC_FLAG_AWD)) {
    Serial.println("AWD flag is set in SR!");
    ADC_ClearFlag(ADC1, ADC_FLAG_AWD);
  }


    /*
    for(int i=0; i <ADC_BUFFER_SIZE; i++)
    Serial.println(adc_buffer[i]);
    */
    //Serial.println();
    // You can also stream out adc_buffer[] here…
    delay(50);
}




/*
// ADC1 & ADC2 share IRQ vector
extern "C" void ADC1_2_IRQHandler(void) {
    // Analog Watchdog interrupt?
    if (ADC_GetITStatus(ADC1, ADC_IT_AWD)) {
        ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
        awd_flag = 1;
    }
    //debug
    //Serial.println("na run dd");

}
*/
