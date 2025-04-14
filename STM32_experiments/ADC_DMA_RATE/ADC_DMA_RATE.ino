#define USE_STDPERIPH_DRIVER
#define STM32F10X_MD
#include "stm32f10x.h"

volatile uint16_t adc_value = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  adc_init();
  timer3_init(1000);  // 1 kHz TRGO from TIM3
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 500) {
    t = millis();
    Serial.println(adc_value);
  }
}

void adc_init() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

  // PA0 as analog input
  GPIO_InitTypeDef gpio;
  gpio.GPIO_Pin = GPIO_Pin_0;
  gpio.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &gpio);

  // ADC configuration
  ADC_InitTypeDef adc;
  adc.ADC_Mode = ADC_Mode_Independent;
  adc.ADC_ScanConvMode = DISABLE;
  adc.ADC_ContinuousConvMode = DISABLE;
  adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO; // TIM3 is trigger source
  adc.ADC_DataAlign = ADC_DataAlign_Right;
  adc.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &adc);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1));

  // Enable interrupt on conversion complete
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  //NVIC_EnableIRQ(ADC1_IRQn);
}

void timer3_init(uint32_t period_us) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  uint16_t prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz
  uint16_t period = period_us - 1;

  TIM_TimeBaseInitTypeDef tim;
  tim.TIM_Prescaler = prescaler;
  tim.TIM_CounterMode = TIM_CounterMode_Up;
  tim.TIM_Period = period;
  tim.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM3, &tim);

  // TRGO on update
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

  TIM_Cmd(TIM3, ENABLE);
}

extern "C" void ADC1_IRQHandler(void) {
  if (ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
    adc_value = ADC_GetConversionValue(ADC1);
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  }
}
