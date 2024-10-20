#include "practice2.h"

//Sample samples[8] = {{1,1}, {2,2}, {3,3}, {4,4}, {5,5}, {6,6}, {7,7}, {8,8}};
uint16_t samples[16] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8};
Sample samplesCopy[8];
int sampleCounter = 0;
int send = 0, startSamples = 1;

// Enable ADC1, DMA2 and GPIO clocks
void initSamples() {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    // DMA2 Stream0 channel0  peripheral to memory
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(ADC1->DR) ; // Peripheral @ - ADC1_DR_ADDRESS
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &samples; // Memory @
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // transfer from peripheral to memory
    DMA_InitStructure.DMA_BufferSize = 16; // closest to 12 bits form the ADC
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // Not increment peripheral @ (always same port)
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // Increment memory @ (increment arr pos)
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 2 bytes(16 bits) data size
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 2 bytes(16 bits) data size
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // circular mode
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; // Direct mode enabled
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; // doesnt matter
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;  // not needed
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; // not needed
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    // DMA2 Stream1 channel1 memory to memory
	DMA_InitTypeDef DMA_InitStructure2;
	DMA_InitStructure2.DMA_Channel = DMA_Channel_1; // channel 3
	DMA_InitStructure2.DMA_PeripheralBaseAddr = (uint32_t) &samples[0];
	DMA_InitStructure2.DMA_Memory0BaseAddr = (uint32_t) &samplesCopy[0];
	DMA_InitStructure2.DMA_DIR = DMA_DIR_MemoryToMemory;
	DMA_InitStructure2.DMA_BufferSize = 16;
	DMA_InitStructure2.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	DMA_InitStructure2.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure2.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure2.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure2.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure2.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure2.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure2.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure2.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure2.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);

    // Init PA1 & PA2(ADC123_IN1 & IN2) as analog
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 2;
    ADC_Init(ADC1, &ADC_InitStructure);

    // ADC3 regular channel1 and 2 configuration
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_112Cycles);

    // Enable DMA request after last transfer
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    // Enable ADC1 DMA & ADC1
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

}

Sample2 getMean() {
	Sample2 res = {2, -2};

	for (int i = 0; i < 8; i++) {
		res.sensorX += samplesCopy[i].sensorX;
		res.sensorY += samplesCopy[i].sensorY;
	}

	// 19856 = 2482 (2 V) * 8
	if (res.sensorX > 19856) res.sensorX = 4;
	else res.sensorX = -4 + (res.sensorX / 2482); // formula is -4 + (x*8 / 2482)

	if (res.sensorY > 19856) res.sensorY = 4;
	else res.sensorY = -4 + (res.sensorY / 2482);

	return res;
}

void initTimerADC() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseInitTypeDef timerStructure;
    timerStructure.TIM_Prescaler = 10 - 1;
    timerStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerStructure.TIM_Period = 500; // 50 us
    timerStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM7, &timerStructure);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM7_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 2;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_Cmd(TIM7, ENABLE);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}

void TIM7_IRQHandler() {
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
    	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

    	if (startSamples == 1 && sampleCounter < 8) {
    		ADC_SoftwareStartConv(ADC1);
    		STM_EVAL_LEDToggle(LED4);
    		sampleCounter++;
    	} else if (sampleCounter == 8) {
    		startSamples = 0;
            sampleCounter = 0;
            send = 1;
        }

        if (send) {
        	DMA_Cmd(DMA2_Stream1, ENABLE);

        	send = 0;
        }
    }
}

void setStartSamples() {
	startSamples = 1;
}
