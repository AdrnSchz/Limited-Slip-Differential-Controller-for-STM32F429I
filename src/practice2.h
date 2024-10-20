#ifndef PRACTICE2_H_
#define PRACTICE2_H_

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f4xx_gpio.h"

typedef struct {
    uint16_t sensorX;
    uint16_t sensorY;
} Sample;

typedef struct {
    int16_t sensorX;
    int16_t sensorY;
} Sample2;

// Config of the DMA to get the samples and DMA to transfer them, ADC3 and ports needed
void initSamples();

// Config ADC time timer with an interrupt every 50 us to get samples
void initTimerADC();

// Calculate the arithmetic mean of each sensor and returns the LCD position
Sample2 getMean();

// Sets the flag to start getting samples
void setStartSamples();
#endif
