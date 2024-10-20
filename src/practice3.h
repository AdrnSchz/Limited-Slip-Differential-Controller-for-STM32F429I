#ifndef PRACTICE3_H_
#define PRACTICE3_H_

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "practice2.h"

typedef enum {
 NO_OK = 0 ,
 OK = !NO_OK
} RetSt ;

typedef struct {
    uint16_t col;
    uint16_t row;
} Positions;

void initLCD();

RetSt SetPixel(uint16_t col, uint16_t row, uint8_t alpha, uint8_t Rval, uint8_t Gval, uint8_t Bval );

uint32_t GetPixel (uint16_t col, uint16_t row);

RetSt DrawHorizontalLine (uint16_t col_start, uint16_t col_end, uint16_t row, uint8_t alpha, uint8_t Rval,
		uint8_t Gval, uint8_t Bval );

RetSt DrawVerticalLine (uint16_t col, uint16_t row_start, uint16_t row_end, uint8_t alpha, uint8_t Rval,
		uint8_t Gval, uint8_t Bval );

RetSt DrawCircumference (uint16_t ccol, uint16_t crow, uint16_t radius, uint8_t alpha, uint8_t Rval,
		uint8_t Gval, uint8_t Bval );

RetSt DrawBitmap (uint16_t col, uint16_t row, uint8_t alpha);

RetSt ClearScreen (uint8_t Rval, uint8_t Gval, uint8_t Bval );

void displayAccelerations(Sample2 newSample, int index);

void displayInstantaneous();

void setIsFilled();

#endif

