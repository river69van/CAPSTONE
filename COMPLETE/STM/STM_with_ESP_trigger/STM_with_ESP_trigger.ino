#define USE_STDPERIPH_DRIVER
#define STM32F10X_MD
#include "stm32f10x.h"
#include "arduinoFFT.h"
#include "ref_sig/sig.h"

#define ADC_BUFFER_SIZE 1024
#define THRESHOLD_LOW     1800    // 12‑bit: 0..4095
#define THRESHOLD_HIGH    2296

#define FFT_BINS       (ADC_BUFFER_SIZE/2) 

const int to_trigger_ESP = PA1;
//dre gun ine na pin ura ura nga priority kay pan verify la kun match an signal 
const int verify_ESP_pin = PA2;

int buffer_count;
const float samplingFrequency = 10000; 

//unsigned long 6;
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
float vReal[ADC_BUFFER_SIZE];
float vImag[ADC_BUFFER_SIZE];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, ADC_BUFFER_SIZE, samplingFrequency, true);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

float cross_cor(){
	
	//ig uncomment pag mayda na hin ref signal
	
	float dot = 0, normA = 0, normB = 0;
	
	for (int i = 0; i < ADC_BUFFER_SIZE/2; i++) {
		
		dot += reference_sig_real[i] * vReal[i];
		normA += reference_sig_real[i] * reference_sig_real[i];
		normB += vReal[i] * vReal[i];
		
	}
	
	return dot / sqrt(normA * normB);
	
	
}

float cross_cor_with_bin_tolerance(float bin_thresh = 4390.0273f) {
    float dot = 0, normA = 0, normB = 0;

    for (int i = 0; i < ADC_BUFFER_SIZE/2; i++) {
        float a = reference_sig_real[i];
        float b = vReal[i];

        // Ignore bins with very low energy (noise)
        if (fabs(a) < bin_thresh && fabs(b) < bin_thresh) continue;

        dot   += a * b;
        normA += a * a;
        normB += b * b;
    }

    float denom = sqrt(normA * normB);
    if (denom == 0) return 0.0f;

    return dot / denom;
}

float cross_cor_with_shift_tolerance(const float *reference_sig_real,
                                     int maxShift,
                                     int *bestShiftPtr = nullptr)
{
    float bestSim = 0.0f;
    int   bestShift = 0;

    // Loop over all allowed shifts
    for (int shift = -maxShift; shift <= maxShift; shift++) {
        float dot = 0, normA = 0, normB = 0;

        // Align ref[i] with vReal[i+shift], but only where indices stay in [0..FFT_BINS)
        for (int i = 0; i < FFT_BINS; i++) {
            int j = i + shift;
            if (j < 0 || j >= FFT_BINS) continue;

            float a = reference_sig_real[i];
            float b = vReal[j];
            dot   += a * b;
            normA += a * a;
            normB += b * b;
        }

        // Compute similarity for this shift
        float denom = sqrtf(normA * normB);
        if (denom <= 0.0f) continue;      // avoid div‑by‑zero
        float sim = dot / denom;

        // Remember the best
        if (sim > bestSim) {
            bestSim = sim;
            bestShift = shift;
        }
    }

    if (bestShiftPtr) *bestShiftPtr = bestShift;
    return bestSim;
}


float cross_cor_with_shift_and_bin_tolerance(
    const float *reference_sig_real,
    int           maxShift,
    float         binThresh,
    int          *bestShiftPtr = nullptr
) {
    float bestSim   = 0.0f;
    int   bestShift = 0;

    // Try every shift in [-maxShift … +maxShift]
    for (int shift = -maxShift; shift <= maxShift; shift++) {
        float dot   = 0.0f;
        float normA = 0.0f;
        float normB = 0.0f;

        // Align ref[i] with vReal[i+shift]
        for (int i = 0; i < FFT_BINS; i++) {
            int j = i + shift;
            if (j < 0 || j >= FFT_BINS) continue;

            float a = reference_sig_real[i];
            float b = vReal[j];

            // skip low‐energy bins (both below threshold)
            if (fabsf(a) < binThresh && fabsf(b) < binThresh) 
                continue;

            dot   += a * b;
            normA += a * a;
            normB += b * b;
        }

        float denom = sqrtf(normA * normB);
        if (denom <= 0.0f) 
            continue;   // no valid bins for this shift

        float sim = dot / denom;
        if (sim > bestSim) {
            bestSim   = sim;
            bestShift = shift;
        }
    }

    if (bestShiftPtr) 
        *bestShiftPtr = bestShift;

    return bestSim;
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

void FFT_setup(){
	
	tim3_trigger_init(10000);  // 15 kHz ADC trigger rate via TIM3
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


void setup() {
	
	pinMode(to_trigger_ESP, OUTPUT);
	digitalWrite(to_trigger_ESP, 1);
	pinMode(verify_ESP_pin, OUTPUT);
	
	Serial.begin(115200);
	tim3_trigger_init(10000);   // 10 kHz sampling
	adc_dma_init();
	//Serial.println("Ready");
	
	
}


void loop() {

  
  
	if (ADC_GetFlagStatus(ADC1, ADC_FLAG_AWD)) {
		//ig trigger pag nag lapos na ha threshold
		digitalWrite(to_trigger_ESP, 0);
		ADC_ClearFlag(ADC1, ADC_FLAG_AWD);
		delay(30);
		FFT_run();
		//if()
		//Serial.println("AWD flag is set in SR!");
	}else{
		
		digitalWrite(to_trigger_ESP, 1);
	}
  


}



void FFT_run(){

	// Convert ADC buffer to float for FFT input
	for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
	vReal[i] = (float)adc_buffer[i] - 2048.0f;
	vImag[i] = 0.0f;
	}

	calculate_FFT();
	Serial.println("Computed magnitudes:");
	//PrintVector(vReal, (ADC_BUFFER_SIZE >> 1), SCL_FREQUENCY);

	PrintVector(vReal, (ADC_BUFFER_SIZE >> 1), SCL_FREQUENCY);

	float x = FFT.majorPeak();
	Serial.println(x, 6); // Print dominant frequenc
	
	
	int shiftUsed;
	float similarity = cross_cor_with_shift_and_bin_tolerance(
	reference_sig_real,
	/*maxShift=*/3,
	/*binThresh=*/5.0f,   // skip bins with |value|<5
	&shiftUsed
	);

	float binWidth    = samplingFrequency / ADC_BUFFER_SIZE;
	float freqShiftHz = shiftUsed * binWidth;

	Serial.print("Similarity: ");
	Serial.println(similarity, 6);
	Serial.print("Bin shift: ");
	Serial.println(shiftUsed);
	Serial.print("Freq shift: ");
	Serial.println(freqShiftHz, 3);
	
	if(similarity > 0.9){
		digitalWrite(verify_ESP_pin, 0);
	}else{
		digitalWrite(verify_ESP_pin, 1);
	}
	//while(1);
	delay(1000);  // Optional for pacing
	
}

void calculate_FFT(){
	FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
	FFT.compute(FFTDirection::Forward);
	FFT.complexToMagnitude();	

}


void calculate_print(){
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

