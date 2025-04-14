#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"
#include "arduinoFFT.h"


#define ADC_BUFFER_SIZE 1024                   // Single-channel DMA

volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];

void adc_dma_init() {
    // Enable peripheral clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA0 (ADC Channel 0) as analog input
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    // DMA1 Channel1 config
    DMA_InitTypeDef dma;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)adc_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = ADC_BUFFER_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;             // Continuous mode
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);

    // Enable DMA1 Channel1
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // ADC1 configuration
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    // Configure ADC Channel 0 (PA0)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);

    // Enable DMA for ADC
    ADC_DMACmd(ADC1, ENABLE);

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);

    // Calibrate ADC
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // Start ADC conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void setup() {
    adc_dma_init();
    Serial.begin(115200);
    //Serial.println("ADC DMA Initialized.");
}

void loop() {
    // Read the latest ADC value (automatically updated by DMA)
    for(int i = 0; i < ADC_BUFFER_SIZE; i++){
    uint16_t a = adc_buffer[i];

    // Convert to voltage (optional, assuming Vref = 3.3V, 12-bit ADC)
    //float voltage = a * (3.3f / 4095.0f);

    // Print the raw value and voltage
    //Serial.print("ADC Raw: ");
    Serial.println(a-2048);
    //Serial.print(" | Voltage: ");
    //Serial.print(voltage, 3);
    //Serial.println(" V");

    //delay(20);
    }
}
