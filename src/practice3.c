#include "practice3.h"

static uint32_t frameBuffer = LCD_FRAME_BUFFER;
Positions prevGraph = {0,0};
Positions sensor[201];
int isFilled = 0, storePtr = 0;
Sample2 stored[10];


void changeLayer(uint32_t layer) {
    if (layer == LCD_BACKGROUND_LAYER) frameBuffer = LCD_FRAME_BUFFER;
    else frameBuffer = LCD_FRAME_BUFFER + BUFFER_OFFSET; //LCD_FOREGROUND_LAYER
}

uint16_t ARGBtoColor(uint8_t alpha, uint8_t red, uint8_t green, uint8_t blue) {
    return ((alpha >> 4) << 12) | ((red >> 4) << 8) | ((green >> 4) << 4) | (blue >> 4);
}

int isInside(uint16_t col, uint16_t row) {
    return (col >= 0 && col < LCD_PIXEL_WIDTH && row >= 0 && row < LCD_PIXEL_HEIGHT);
}

RetSt SetPixel(uint16_t col, uint16_t row, uint8_t alpha, uint8_t Rval, uint8_t Gval, uint8_t Bval ) {

    if (!isInside(col, row)) return NO_OK;

    *(__IO uint16_t*) (frameBuffer + (2 * (LCD_PIXEL_WIDTH * row + col))) = ARGBtoColor(alpha, Rval, Gval, Bval);

	return OK;
}

uint32_t GetPixel(uint16_t col, uint16_t row) {
	if (!isInside(col, row)) return 0x0000;

	uint16_t pixelValue = *(__IO uint16_t*)(frameBuffer + 2 * (LCD_PIXEL_WIDTH * row + col));

	return 0xFF00 | pixelValue;
}

RetSt DrawHorizontalLine(uint16_t col_start, uint16_t col_end, uint16_t row, uint8_t alpha, uint8_t Rval,
		uint8_t Gval, uint8_t Bval ) {

    if (!isInside(col_start, row) || !isInside(col_end, row)) return NO_OK;

    if (col_start > col_end) {
		uint16_t temp_col = col_start;
		col_start = col_end;
		col_end = temp_col;
	}

	for (uint16_t i = col_start; i <= col_end; i++) {
		SetPixel(i, row, alpha, Rval, Gval, Bval);
	}

	return OK;
}

RetSt DrawVerticalLine(uint16_t col, uint16_t row_start, uint16_t row_end, uint8_t alpha, uint8_t Rval,
		uint8_t Gval, uint8_t Bval ) {

    if (!isInside(col, row_start) || !isInside(col, row_end)) return NO_OK;

    if (row_start > row_end) {
    	uint16_t temp_row = row_start;
    	row_start = row_end;
    	row_end = temp_row;
    }

	for (uint16_t i = row_start; i <= row_end; i++) {
		SetPixel(col, i, alpha, Rval, Gval, Bval);
	}

	return OK;
}

RetSt DrawCircumference(uint16_t ccol, uint16_t crow, uint16_t radius, uint8_t alpha, uint8_t Rval,
		uint8_t Gval, uint8_t Bval ) {

	if (!isInside(ccol - radius, crow) || !isInside(ccol, crow - radius) ||
			!isInside(ccol + radius, crow) || !isInside(ccol, crow + radius)) return NO_OK;

	int x = 0, y = radius, d = 3 - (radius << 1);

	while (x <= y) {
		SetPixel( x + ccol, y + crow , alpha, Rval, Gval, Bval);
		SetPixel(-x + ccol, y + crow, alpha, Rval, Gval, Bval);
		SetPixel( x + ccol, -y + crow, alpha, Rval, Gval, Bval);
		SetPixel(-x + ccol, -y + crow, alpha, Rval, Gval, Bval);
		SetPixel( y + ccol, x + crow, alpha, Rval, Gval, Bval);
		SetPixel(-y + ccol, x + crow, alpha, Rval, Gval, Bval);
		SetPixel( y + ccol, -x + crow, alpha, Rval, Gval, Bval);
		SetPixel(-y + ccol, -x + crow, alpha, Rval, Gval, Bval);

		if( d < 0 ) d += (x<<2) + 6;
		else {
			d += ((x-y)<<2) + 10;
			y--;
		}
		x++;
	}

	return OK;
 }

RetSt DrawBitmap(uint16_t col, uint16_t row, uint8_t alpha) {
	if (!isInside(col -1, row - 1) || !isInside(col + 1, row + 1)) return NO_OK;

	for (uint16_t i = col - 1; i <= col + 1; i++) {
		for (uint16_t j = row - 1; j <= row + 1; j++) {
				SetPixel(i, j, alpha, 0xFF, 0x00, 0x00);
		}
	}

	return OK;
}

RetSt ClearScreen(uint8_t Rval, uint8_t Gval, uint8_t Bval ) {
	uint16_t color = ARGBtoColor(0xFF, Rval, Gval, Bval);
	for (uint32_t i = 0; i < BUFFER_OFFSET; i++) {
	    *(__IO uint16_t*)(frameBuffer + (2 * i)) = color;
	}

	return OK;
}

RetSt ClearLayer2(uint8_t Rval, uint8_t Gval, uint8_t Bval ) {
	uint16_t color = ARGBtoColor(0, Rval, Gval, Bval);
	for (uint32_t i = 0; i < BUFFER_OFFSET; i++) {
	    *(__IO uint16_t*)(frameBuffer + (2 * i)) = color;
	}

	return OK;
}

void drawBackground() {
	// Horizontal rectangle (vertical in the statement image)
	DrawHorizontalLine(35, 236, 4, 0xFF, 0, 0, 0);
	DrawHorizontalLine(35, 236, 86, 0xFF, 0, 0, 0);
	DrawVerticalLine(35, 4, 86, 0xFF, 0, 0, 0);
	DrawVerticalLine(236, 4, 86, 0xFF, 0, 0, 0);
	DrawHorizontalLine(35, 236, 45, 0xFF, 0, 0, 0); // middle line

	// Vertical rectangle (horizontal in the statement image)
	DrawHorizontalLine(154, 236, 101, 0xFF, 0, 0, 0);
	DrawHorizontalLine(154, 236, 302, 0xFF, 0, 0, 0);
	DrawVerticalLine(154, 101, 302, 0xFF, 0, 0, 0);
	DrawVerticalLine(236, 101, 302, 0xFF, 0, 0, 0);
	DrawVerticalLine(195, 101, 302, 0xFF, 0, 0, 0); // middle line(exactly would be 100,5)

    // X-Y graph
    DrawVerticalLine(76, 130, 270, 0xFF, 0, 0, 0);
    DrawHorizontalLine(6, 146, 200, 0xFF, 0, 0, 0);
    DrawCircumference(76, 200, 15, 0xFF, 0, 0, 0);
    DrawCircumference(76, 200, 30, 0xFF, 0, 0, 0);
    DrawCircumference(75, 200, 45, 0xFF, 0, 0, 0);
    DrawCircumference(76, 200, 60, 0xFF, 0, 0, 0);
}

void initLCD () {
    LCD_Init();
    LCD_LayerInit();

    SDRAM_Init();
    FMC_SDRAMWriteProtectionConfig(FMC_Bank2_SDRAM, DISABLE);

    LTDC_Cmd(ENABLE);
    LTDC_LayerPixelFormat(LTDC_Layer1, LTDC_Pixelformat_ARGB4444);
	LTDC_LayerPixelFormat(LTDC_Layer2, LTDC_Pixelformat_ARGB4444);
	LTDC_ReloadConfig(LTDC_VBReload);

	// Background set up
    ClearScreen(0xFF, 0xFF, 0xFF);
    drawBackground();

    // Change to layer2
    changeLayer(LCD_FOREGROUND_LAYER);
    ClearLayer2(0,0,0);
}

void storeSample(Sample2 newSample) {
	stored[storePtr] = newSample;
	storePtr++;

	if (storePtr == 10) storePtr = 0;
}
Positions getPosition(Sample2 newSample) {
	Positions newPos;
	storeSample(newSample);

	newPos.col = 45.5 +  -10.125 * newSample.sensorY;
	newPos.row = 195.5 +  -10.125 * newSample.sensorX;

	return newPos;
}

void displayAccelerations(Sample2 newSample, int index) {
	if (isFilled == 0) {
		sensor[index - 1] = getPosition(newSample);

		SetPixel(35 + index, sensor[index - 1].col, 0xFF, 0xFF, 0, 0);
		SetPixel(sensor[index - 1].row, 302 - index, 0xFF, 0, 0, 0xFF);
	} else {
		sensor[200] = getPosition(newSample);
		for (int i = 0; i < 200; i++) {
			SetPixel(36 + i, sensor[i].col, 0, 0, 0, 0);
			SetPixel(sensor[i].row, 301 - i, 0, 0, 0, 0);

			SetPixel(36 + i, sensor[i + 1].col, 0xFF, 0xFF, 0, 0);
			SetPixel(sensor[i + 1].row, 301 - i, 0xFF, 0, 0, 0xFF);

			sensor[i] = sensor[i + 1];
		}
	}
}

Positions getInstantaneous() {
	Sample2 samp = {0,0};
	Positions newPos;

	for (int i = 0; i < 10; i++) {
		samp.sensorX += stored[i].sensorX;
		samp.sensorY += stored[i].sensorY;
	}

	samp.sensorX /= 10;
	samp.sensorY /= 10;

	newPos.col = 200 +  -15 * samp.sensorY;
	newPos.row = 76 +  -15 * samp.sensorX;

	return newPos;

}

void displayInstantaneous() {
	Positions newPos = getInstantaneous();

	DrawBitmap(prevGraph.row, prevGraph.col, 0);
	DrawBitmap(newPos.row, newPos.col, 0xFF);

	prevGraph = newPos;
}

void setIsFilled() {
	isFilled = 1;
}

