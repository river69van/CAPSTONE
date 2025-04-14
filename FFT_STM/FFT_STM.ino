#define USE_STDPERIPH_DRIVER
#define STM32F10X_MD
#include "stm32f10x.h"
#include "arduinoFFT.h"


#define ADC_BUFFER_SIZE 1024

int buffer_count;
const float samplingFrequency = 15000; 

//unsigned long 6;
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
float vReal[ADC_BUFFER_SIZE];
float vImag[ADC_BUFFER_SIZE];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, ADC_BUFFER_SIZE, samplingFrequency, true);
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void adc_dma_init() {
    // Clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA0 (ADC_IN0)
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    // DMA config
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
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // ADC config
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;  // <-- TIM3 TRGO
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    // Calibrate
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // Enable external trigger
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
}

void tim3_trigger_init(uint32_t sample_rate_hz) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    uint32_t timer_clock = 72000000;  // 72 MHz
    uint16_t prescaler = 71;          // 1 MHz timer clock
    uint16_t period = (1000000 / sample_rate_hz) - 1;

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Prescaler = prescaler;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = period;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tim);

    // TRGO on update event
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

    // Start TIM3
    TIM_Cmd(TIM3, ENABLE);
}

void setup() {


	Serial.begin(115200);
	//Serial.println("Ready");
	
	tim3_trigger_init(15000);  // 15 kHz ADC trigger rate via TIM3
  adc_dma_init();
	delay(200);
	
  for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
    vReal[i] = (float)adc_buffer[i] - 2048.0;
    vImag[i] = 0.0f;
  }
	
    // Setup serial if needed
    // Serial.begin(115200); // if using STM32 serial redirect
	delay(200);
}


void loop() {
  // Convert ADC buffer to float for FFT input
  for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
    vReal[i] = (float)adc_buffer[i] - 2048.0f;
    vImag[i] = 0.0f;
  }

  Serial.println("Data:");
  PrintVector(vReal, ADC_BUFFER_SIZE, SCL_TIME);

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	
  Serial.println("Weighed data:");
  PrintVector(vReal, ADC_BUFFER_SIZE, SCL_TIME);

  FFT.compute(FFTDirection::Forward); 
  Serial.println("Computed Real values:");
  PrintVector(vReal, ADC_BUFFER_SIZE, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, ADC_BUFFER_SIZE, SCL_INDEX);

  FFT.complexToMagnitude(); 
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (ADC_BUFFER_SIZE >> 1), SCL_FREQUENCY);

  float x = FFT.majorPeak();
  Serial.println(x, 6); // Print dominant frequency

  delay(1000);  // Optional for pacing
}


/*
void loop() {
 
  Serial.println("Data:");
  PrintVector(vReal, ADC_BUFFER_SIZE, SCL_TIME);
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	
  Serial.println("Weighed data:");
  PrintVector(vReal, ADC_BUFFER_SIZE, SCL_TIME);
  FFT.compute(FFTDirection::Forward); 
  Serial.println("Computed Real values:");
  PrintVector(vReal, ADC_BUFFER_SIZE, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, ADC_BUFFER_SIZE, SCL_INDEX);
  FFT.complexToMagnitude(); 
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (ADC_BUFFER_SIZE >> 1), SCL_FREQUENCY);
  float x = FFT.majorPeak();
  Serial.println(x, 6); //Print out what frequency is the most dominant.
  delay(1000);
 
 
}
*/








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

