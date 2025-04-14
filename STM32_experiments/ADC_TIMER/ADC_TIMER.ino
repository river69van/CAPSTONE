#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"

void adc_init() {
    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA0 (ADC1_IN0)
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    // ADC configuration
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;  // External trigger only
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);

    ADC_Cmd(ADC1, ENABLE);

    // Calibration
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // Start external triggering (no software trigger needed)
    // Don't call ADC_SoftwareStartConvCmd()!
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
}

void tim1_init(uint32_t sample_rate_hz) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    uint32_t timer_clk = 72000000; // 72 MHz APB2
    uint16_t prescaler = 71;       // 72MHz / (71+1) = 1MHz timer clock
    uint16_t period = (1000000 / sample_rate_hz) - 1; // 1 MHz / SR

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Prescaler = prescaler;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = period;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    // Use OC1 as trigger
    TIM_OCInitTypeDef oc;
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = period / 2;  // 50% duty cycle
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1); // Trigger ADC on OC1

    TIM_CtrlPWMOutputs(TIM1, ENABLE);  // Important for TIM1
    TIM_Cmd(TIM1, ENABLE);
}

void setup() {
    Serial.begin(115200);
    tim1_init(1000); // 1kHz sample rate
    adc_init();
    Serial.println("Timer-triggered ADC (no DMA) initialized.");
}

void loop() {
    //if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        uint16_t val = ADC_GetConversionValue(ADC1);
        float voltage = val * (3.3f / 4095.0f);

        Serial.print("ADC: ");
        Serial.print(val);
        Serial.print(" | Voltage: ");
        Serial.print(voltage, 3);
        Serial.println(" V");

        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    //}

    delay(5);  // Polling loop, make sure this is fast enough
}
