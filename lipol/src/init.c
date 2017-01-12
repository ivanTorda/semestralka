#include "stm32l1xx_nucleo.h"

/*
 * Inicializiacia ADC prevodnika, pre meranie napatia baterky a pretekajuceho prudu.
 * Vyziva sa konkretne ADC1 prevodnik, a z neho 2 kanaly.
 * Fyzicky su to porty PA0 PA1
 */
void adc_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* Configure ADCx Channel 2 as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* Enable the HSI oscillator */
	RCC_HSICmd(ENABLE);

	/* Check that HSI oscillator is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	/* Enable ADC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* ADC1 configuration */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Enable the ADC */
	ADC_Cmd(ADC1, ENABLE);

}

/*
 * Nie som si isty ci toto potrebujeme, mala by to byt inicializacia
 * prerusenia (IRQ) pre ADC1 prevodnik. To znamena ze v pravidelnom intervale si ADC1 prevodnik
 * vynuti prerusenie a hodi hodnotu do premennej.
 * Lenze my dostavame data vlastnym internym prerusenim.
 * Proste si to sami vzorkujeme v pravidelnom intervale
 */
void NVICInit(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * Inicializacia UARTU, konkretne USART1, vyuzivaju sa
 * piny PA9 a PA10, rychlost 9600 BaudRate/s
 * Plus dole je k tomu inicializacia prerusenia (IRQ) pre UART.
 * To znamena ze ked nieco pride na uart, nejaky znak
 * vykona sa prerusenie a prijaty znak sa ulozi do premennej
 *
 */
void UART_init(void){

	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

}

/*
 * Inicializacia LEDky.. nic extra
 * blikne ak zaregistroval povel cez UART, riesi sa to v hlavnej funkcii main()
 */
void LED_init(void) {
	STM_EVAL_LEDInit(LED2);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}
/*
 * Inicializacia portu pre vystup. Pouzivame 2 vystupy.
 * Prvy je dolezity PA7, ten spina rele a rele zase celkovu zataz (ziarovku)
 *
 * Druha je PA6, to je len taka notifikacna LEDka.. blikne pri kazdom merani..
 */
void outputPortInit() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitTypeDef gpioInitStruct;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_6;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	GPIO_SetBits(GPIOA, GPIO_Pin_6);
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_7;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	GPIO_SetBits(GPIOA, GPIO_Pin_7);
}


/*
 * Incializacia internych preruseni TIM3 a TIM4 (Timerov)
 * takze Pouzivame 2 interne prerusenia,
 * Prvy Timer TIM4 zbehne kazdych 0.5 sekundy (vid TIM.TIM_Period = 500; //ms)
 * Tento vyuzivame aby kazdu tuto periodu riesila zapis do EEPROM, a vobec cely
 * meraci proces.
 * Este tu kontrolujem, ci napatie nekleslo pod nebezpecnu hodnotu.. 10.2V
 *  Podrobnejsie sa mu budem venovat v main.c -> TIM4_IRQHandler
 *
 * Druhy Timer TIM3 zbehne kazdych 50ms (vid TIM.TIM_Period = 50;)
 * Tu sa riesi samotne vzorkovanie prudu a napatie
 * napatie sa sice vzorkuje kazdych 50ms ale s prudom je to trosku zlozitejsie..
 * ale nic hrozne..
 * slubujem.
 */
void InitializeTimer(void) {
	TIM_TimeBaseInitTypeDef TIM;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// Time base configuration
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Prescaler = (SystemCoreClock / 1000) - 1;
	TIM.TIM_Period = 500; // 500ms interval
	TIM.TIM_ClockDivision = 0;
	TIM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM4_IRQn); // Enable TIM4 IRQ
	TIM_Cmd(TIM4, ENABLE); // Enable counter on TIM4

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// Time base configuration
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Prescaler = (SystemCoreClock / 1000) - 1;
	TIM.TIM_Period = 50; // 50ms interval
	TIM.TIM_ClockDivision = 0;
	TIM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM4 IRQ
	TIM_Cmd(TIM3, ENABLE); // Enable counter on TIM4
}
/*
 * Tato funkcia by tu nemala byt, pretoze sa tu nic neinicializuje..
 *
 * Je to funkcia ktora vrati hodnotu ADC (napatie/prud).
 * Ak chcem napatie tak hodim do parametra ADC_Channel_1
 * Ak chcem prud tak ADC_Channel_0
 *
 */
uint16_t Read_AD_Value(uint8_t channel) {

	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_16Cycles);

	/* Start ADC Software Conversion */
	ADC_SoftwareStartConv(ADC1);

	/* Wait till done */
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
	}

	return ADC_GetConversionValue(ADC1);

}
