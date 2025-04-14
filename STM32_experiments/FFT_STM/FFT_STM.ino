#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"
#include "arduinoFFT.h"



/*
#define ADC_BUFFER_SIZE 8
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];

void adc_dma_init() {
    // Enable clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA0 (ADC Channel 0) as analog input
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    // DMA1 Channel1 config (ADC1 → Memory)
    DMA_InitTypeDef dma;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)adc_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = ADC_BUFFER_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);

    // Enable DMA
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // ADC configuration
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    // Configure ADC1 channel 0 (PA0)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);

    // Enable DMA for ADC
    ADC_DMACmd(ADC1, ENABLE);

    // Enable ADC
    ADC_Cmd(ADC1, ENABLE);

    // Calibration
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // Start conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void setup() {
    adc_dma_init();
    // Setup serial if needed
     Serial.begin(115200); // if using STM32 serial redirect
}

void loop() {
    // Example: print buffer
    for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
        // Serial.println(adc_buffer[i]); // if using redirect
    }
    for (volatile int i = 0; i < 100000; i++); // crude delay
}
*/


#define ADC_BUFFER_SIZE 1024


const float samplingFrequency = 100; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
//unsigned long 6;
float adc_buffer[ADC_BUFFER_SIZE];
float vImag[ADC_BUFFER_SIZE];
ArduinoFFT<float> FFT = ArduinoFFT<float>(adc_buffer, vImag, ADC_BUFFER_SIZE, samplingFrequency, true);
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void adc_dma_init() {
    // Enable clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA0 (ADC Channel 0) as analog input
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    // DMA1 Channel1 config (ADC1 → Memory)
    DMA_InitTypeDef dma;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)adc_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = ADC_BUFFER_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);

    // Enable DMA
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // ADC configuration
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    // Configure ADC1 channel 0 (PA0)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);

    // Enable DMA for ADC
    ADC_DMACmd(ADC1, ENABLE);

    // Enable ADC
    ADC_Cmd(ADC1, ENABLE);

    // Calibration
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // Start conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void setup() {
  //adc_dma_init();
	sampling_period_us = round(1000000*(1.0/samplingFrequency));
	Serial.begin(115200);
	Serial.println("Ready");
	
	
	for(int i=0; i<ADC_BUFFER_SIZE; i++){

	vImag[i] = 0;
	
	}
	
    // Setup serial if needed
    // Serial.begin(115200); // if using STM32 serial redirect
}

void loop() {

  Serial.println("Data:");
  PrintVector(adc_buffer, ADC_BUFFER_SIZE, SCL_TIME);
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	
  Serial.println("Weighed data:");
  PrintVector(adc_buffer, ADC_BUFFER_SIZE, SCL_TIME);
  FFT.compute(FFTDirection::Forward); 
  Serial.println("Computed Real values:");
  PrintVector(adc_buffer, ADC_BUFFER_SIZE, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, ADC_BUFFER_SIZE, SCL_INDEX);
  FFT.complexToMagnitude(); 
  Serial.println("Computed magnitudes:");
  PrintVector(adc_buffer, (ADC_BUFFER_SIZE >> 1), SCL_FREQUENCY);
  float x = FFT.majorPeak();
  Serial.println(x, 6); //Print out what frequency is the most dominant.
  while(1);
  
}

void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    float abscissa;
  
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / ADC_BUFFER_SIZE);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

