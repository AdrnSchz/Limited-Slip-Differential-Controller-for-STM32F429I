/* Includes */
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f4xx_gpio.h"
#include "practice2.h"
#include "practice3.h"

/* Private macro */
/* Private variables */
int counterMs = 0, lastChange = 0;
float speed = 0, speeds[] = { 0, 11250, 3214.28, 2500, 1125, 416.66 };
float correlation = 1, correlations[] = { 1.0, 1.25, 1.35, 1.8, 2.2 };
int speed_ptr = 0, correlation_ptr = 0, status_low = 0, status_high = 0;
uint32_t pwmPeriodL = 1, pwmPeriodR = 1, lastPwmTimeL = 0, lastPwmTimeR = 0;
int numTransfers = 0, numSamples = 0, arrayFilled = 0;
int inc = 0, testPtr = 0, values[9] = {-4,-3,-2,-1,0,1,2,3,4};
TIM_TimeBaseInitTypeDef timer_right, timer_left;
Sample2 sampleMean[10];
/* Private function prototypes */
void getSpeed(int* speed_right, int* speed_left);
uint32_t getPeriod(int wheel);
void updatePeriod();
void calculateRevolutions();
void buttonHandler();
Sample2 calculateMean();
void initPorts();
void initPwmListeners();
void initPwm();
void initButton();
void initDACOut();
void initTimer1ms();
void initTimersPwm();
void initFreeTimer();

/* Private functions */
int main(void)
{
  // P1
  initPorts();
  initTimer1ms();
  initTimersPwm();
  initFreeTimer();

  // P3
  initLCD();

  // P2
  initSamples();
  initTimerADC();

  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED4);

  /* Infinite loop */
  float  revolutionsR, revolutionsL, corre;
  while (1)
  {
	  if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_SET) buttonHandler();
	  //generate VC
	  calculateRevolutions(&revolutionsR, &revolutionsL, &corre);
	  if (revolutionsR > 5.55 && revolutionsL > 5.55 && corre > 1.3) {
		  if (corre >= 2 && revolutionsR > revolutionsL) {
			  // Vc = Vfs
			  DAC_SetChannel1Data(DAC_Align_12b_R, 0xFFF);
		  } else if (corre >= 2 && revolutionsL > revolutionsR) {
			  // Vc = 0 V
			  DAC_SetChannel1Data(DAC_Align_12b_R, 0x000);
		  } else {
			  float x = corre - 0.3;
			  int y = (1.65 * x - 0.495) / 0.7;
			  if (revolutionsR > revolutionsL) {
				  // Vc = y V
				  DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(y * 0xFFF / 3.3 ));
			  } else {
				  // Vc = (3.3 - y) V
				  DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)((3.3 - y) * 0xFFF) / 3.3);
			  }
		  }
	  } else {
		  // Vc = (Max_voltage / 2) V
		  DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(0xFFF / 2));
	  }
  }
}

void calculateRevolutions(float *revolutionsR, float *revolutionsL, float *corre) {
	if (pwmPeriodR == 1) {
		*revolutionsR = 1;
		*revolutionsL = 1;
		*corre = 1;
	}
	*revolutionsR = 31250 / pwmPeriodR;
	*revolutionsL = 31250 / pwmPeriodL;

	if (revolutionsR > revolutionsL) *corre = *revolutionsR / *revolutionsL;
	else *corre = *revolutionsL / *revolutionsR;
}

//___________________________________________DMA_HANDLER___________________________________________

void DMA2_Stream1_IRQHandler() {
	if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1)) {
	    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);

	    sampleMean[numTransfers] = getMean();
	    //Sample2 test = {values[testPtr], values[testPtr] * - 1};
	    //sampleMean[numTransfers] = test;
	    numTransfers++;

	    if (numTransfers == 10) {
	    	numTransfers = 0;
			numSamples++;

	    	displayAccelerations(calculateMean(), numSamples);
	    }

	    if (numSamples % 10 == 0) displayInstantaneous();
	    if (numSamples == 200) {
	    	numSamples = 0;
	    	setIsFilled();
	    }
	}
}

Sample2 calculateMean() {
	Sample2 newSample = {0,0};
	for (int i = 0; i < 10; i++) {
		newSample.sensorX += sampleMean[i].sensorX;
		newSample.sensorY += sampleMean[i].sensorY;
	}

	newSample.sensorX /= 10;
	newSample.sensorY /= 10;

	return newSample;
}
//___________________________________________TIMER_HANDLER___________________________________________
// Module 3: ISR of the left wheel PWM timer
void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		if (speed == 0) GPIO_ResetBits(GPIOG, GPIO_Pin_2);
		else GPIO_ToggleBits(GPIOG, GPIO_Pin_2);
	}
}

// Module 3: ISR of the right wheel PWM timer
void TIM3_IRQHandler() {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		if (speed == 0) GPIO_ResetBits(GPIOG, GPIO_Pin_3);
		else GPIO_ToggleBits(GPIOG, GPIO_Pin_3);
	}
}

// Module 1: ISR of the 1ms timer
void TIM4_IRQHandler() {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        counterMs++; // counts how many ms have passed
        lastChange++; // last time since rising edge if pwm
        setStartSamples();

        if (counterMs == 45) {
        	if (inc == 0) {
				testPtr++;
				if (testPtr == 8) inc = 1;
			} else {
				testPtr--;
				if (testPtr == 0) inc = 0;
			}
        }
        // When 200 ms have passed it toggles the LED.
        if (counterMs == 200) {
        	counterMs = 0;
        	STM_EVAL_LEDToggle(LED3);
        }

        if (lastChange == 40) {
        	lastChange = 0;
        	pwmPeriodL = 1;
        	pwmPeriodR = 1;
        }
    }
}

// Module 3: function to get the speed for both right and left wheel.
void getSpeed(int* speed_right, int* speed_left) {
	if (status_high < 2) {
		*speed_right = speed;
		*speed_left =  speed / correlation;
	} else {
		*speed_left = speed;
		*speed_right = speed / correlation;
	}
}

// Module 3: Function to get the period of each sensor PWM
uint32_t getPeriod(int wheel) {
	// When speed is 0 it sets a default value.
	if (speed == 0) return (uint32_t) 10000;

	int speed_right, speed_left;
	getSpeed(&speed_right, &speed_left);

	// when wheel is 1 it returns the period for the right wheel, otherwise for the left
	if (wheel == 1) return (uint32_t)speed_right;

	return (uint32_t)speed_left;
}

// Module 3: function to update the period of each sensor when the speed or correlation changes
void updatePeriod() {
	timer_left.TIM_Period = getPeriod(0);
	TIM_TimeBaseInit(TIM2, &timer_left);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	timer_right.TIM_Period = getPeriod(1);
	TIM_TimeBaseInit(TIM3, &timer_right);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

//___________________________________________EXTERNAL__HANDLERS___________________________________________
// Module 3: ISR of the button
void buttonHandler() {
	// when auxiliary bit is set changes correlation, when not set changes speed
	if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_9) == Bit_SET) {
		if (speed == 0) {
			status_high = 0;
			correlation_ptr = 0;
			correlation = 1;
		} else {
			if (status_high % 2 == 0) correlation_ptr++;
			else correlation_ptr--;

			if (correlation_ptr == 4 || correlation_ptr == 0) {
				status_high++;
				if (status_high == 4) status_high = 0;
			}

			correlation = correlations[correlation_ptr];
		}
	} else {
		if (status_low == 0) speed_ptr++;
		else speed_ptr--;

		if (speed_ptr == 5) status_low = 1;
		else if (speed_ptr == 0) status_low = 0;

		speed = speeds[speed_ptr];
	}
	updatePeriod();
}

// calculate period of left wheel PWM
void EXTI0_IRQHandler() {
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line0);
		uint32_t currentTime = TIM_GetCounter(TIM5);
		lastChange = 0;

		if (currentTime > lastPwmTimeL) pwmPeriodL = currentTime - lastPwmTimeL;
		else pwmPeriodL = currentTime + (0xFFFF - lastPwmTimeL);

		lastPwmTimeL = currentTime;
	}

	/*if (EXTI_GetITStatus(EXTI_Line10) != RESET && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET) {
		EXTI_ClearITPendingBit(EXTI_Line0);
		buttonHandler();
	}*/
}


// calculate period of right wheel PWM
void EXTI1_IRQHandler() {
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line1);
		uint32_t currentTime = TIM_GetCounter(TIM5);
		lastChange = 0;

		if (currentTime > lastPwmTimeR) pwmPeriodR = currentTime - lastPwmTimeR;
		else pwmPeriodR = currentTime + (0xFFFF - lastPwmTimeR);

		lastPwmTimeR = currentTime;
	}
}



//___________________________________________CONFIG___________________________________________
// function to initialize all ports
void initPorts() {
	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Config input port to listen pwms (PC0 & PC1)
	initPwmListeners();
	// Config wheel sensors PWM's (PG2 & PG3) & auxiliary pin for button (PG9)
	initPwm();
	// Config button (PA0)
	initButton();
	// Config Vc signal (PF0)
	initDACOut();
}

// Module 2: Config input port to listen pwms (PC0 & PC1)
void initPwmListeners() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_0;
	gpioStructure.GPIO_Mode = GPIO_Mode_IN;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &gpioStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

	EXTI_InitTypeDef extiStructure;
	extiStructure.EXTI_Line = EXTI_Line0;
	extiStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	extiStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	extiStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&extiStructure);

	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = EXTI0_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

	gpioStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC, &gpioStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);

	extiStructure.EXTI_Line = EXTI_Line1;
	EXTI_Init(&extiStructure);

	nvicStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&nvicStructure);
}
// Module 3: Config wheel sensors PWM's (PG2 & PG3) & auxiliary pin for button (PG9)
void initPwm() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	gpioStructure.GPIO_OType = GPIO_OType_PP;
	gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &gpioStructure);
	GPIO_ResetBits(GPIOG, GPIO_Pin_2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_3);

	gpioStructure.GPIO_Pin = GPIO_Pin_9;
	gpioStructure.GPIO_Mode = GPIO_Mode_IN;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	gpioStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOG, &gpioStructure);
}

// Module 3: Config button (PA0)
void initButton() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef buttonStructure;
	buttonStructure.GPIO_Pin = GPIO_Pin_0;
	buttonStructure.GPIO_Mode = GPIO_Mode_IN;
	buttonStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &buttonStructure);

	/*SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	EXTI_InitTypeDef extiStructure;
	extiStructure.EXTI_Line = EXTI_Line0;
	extiStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	extiStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	extiStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&extiStructure);

	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = EXTI0_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 4;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);*/
}

// Config GPIO for Vc analog signal and DAC
void initDACOut() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	// Config analog GPIO
	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_4;
	gpioStructure.GPIO_Mode = GPIO_Mode_AN;
	gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpioStructure);
	/*
	// Config DAC
	DAC_InitTypeDef dacStructure;
	DAC_StructInit(&dacStructure);
	DAC_Init(DAC_Channel_1, &dacStructure);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);*/
}

// Module 1: Initializes the timer to count up to 1ms and sets an interrupt for it
void initTimer1ms() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Timer config
    TIM_TimeBaseInitTypeDef timerStructure;
    timerStructure.TIM_Prescaler = 1000 - 1;
    timerStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerStructure.TIM_Period = 90 - 1;
    timerStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM4, &timerStructure);

    // Interrupt config
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 5;
    nvicStructure.NVIC_IRQChannelSubPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

// Module 2: Initializes the timer as a free-running timer
void initFreeTimer() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	TIM_TimeBaseInitTypeDef timerStructure;
	timerStructure.TIM_Prescaler = 90 - 1;
	timerStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerStructure.TIM_Period = 0xFFFF;
	timerStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM5, &timerStructure);

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_Cmd(TIM5, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}

// Module 3: Initializes the timer for each sensor PWM and sets an interrupt to each one
void initTimersPwm() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Left timer config
	timer_left.TIM_Prescaler = 90 - 1;
	timer_left.TIM_CounterMode = TIM_CounterMode_Up;
	timer_left.TIM_Period = getPeriod(0);
	timer_left.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM2, &timer_left);

	// Left timer interrupt config
	NVIC_InitTypeDef nvicStructure2;
	nvicStructure2.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure2.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure2.NVIC_IRQChannelSubPriority = 0;
	nvicStructure2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure2);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // Right timer config
    timer_right.TIM_Prescaler = 90 - 1;
    timer_right.TIM_CounterMode = TIM_CounterMode_Up;
    timer_right.TIM_Period = getPeriod(1);
    timer_right.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM3, &timer_right);

    // Right timer interrupt config
	NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}
/*
 * Callback used by stm324xg_eval_i2c_ee.c.
 * Refer to stm324xg_eval_i2c_ee.h for more info.
 */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
  /* TODO, implement your code here */
  while (1)
  {
  }
}
