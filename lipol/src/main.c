/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "init.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32l1xx_flash.h"

#define NUMBER_OF_SAMPLES 10
#define MAX_STRLEN 12 // this is the maximum string length of our string in characters

char received_string[MAX_STRLEN+1]; // this will hold the recieved string
uint8_t eeprom[256];
FLASH_Status FLASHStatus;
///////////////////////////////////////////////////////////
//vsetky tieto premenne su nejake stavove premenne, FLAGY,
//actualCurrent je najnovsia prefiltrovana vzorka prudu
/////////////////////////////////////////////////////////
uint16_t currentSamples[NUMBER_OF_SAMPLES];
uint8_t sampleCounter = 0;

float actualCurrent = 0.00;

float current = 0.00;
uint16_t voltage = 0.00;

uint32_t eepromCurrentPosition = 0;
uint8_t currentMeasureTime = 0; //counter for make a sample
uint8_t voltageMeasureTime = 0;
uint8_t startMeasureFlag = 0; //start/stop flag for measuring
uint8_t clearEEPROMFlag = 0; //start/stop flag for measuring

uint8_t measureStatus = 0; //0 - not measuring; 1 - measuring; 2 - stopped measure, EEPROM overflow; 3 - stopped measure low battery;
uint8_t startRead = 0;


//Ak je clanok plne nabity, tak ma napatie 4.2 Volta
//Ak je clanok vybity, tak kriticke napatie na clanku, 3.4Volta
//a ked sa nezastavi vybijaci proces pod tuto hranicu tak dochadza k poskodeniu clanku :P
uint8_t lowVoltageCell = 3.4;

//pocet clankov v baterii, ja som pouzival 3 clanky, cize dokopy pri nabitej baterii je 3*4.2V = 12.6V
//pri vybitej baterii je to zase 3*3.4V = 10.2V
uint8_t numberOfCell = 3;

// to je premenna pre kriticke napatie baterky 3*3.4V = 10.2V
uint8_t voltageCutOff;

//counter
uint8_t i = 0;

uint16_t receivedChar;
RCC_ClocksTypeDef RCC_Clocks;


/*
 * Hlavna funkcia main, tu sa inicializuju vsetky veci, kazda ta vec je rozpisana v init.c
 *
 * While(1) je nekonecny cyklus, v ktorom sa dokola pytam, ci prijaty znak receivedChar z UARTU nie je
 * pre mna dolezity. Premenna receivedChar sa naplna v preruseni (vid USART1_IRQHandler)
 * Ak je nejaky znak spravny, tak sa podla toho zariad
 * podrobne to opisem pri kazdom IF-e
 *
 * Ak vsetky inicializacie prebehli ok, tak posle na UART 'ready'.
 */
int main(void) {

	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

	adc_init();
	NVICInit();
	UART_init();
	InitializeTimer();
	outputPortInit();
	LED_init();
	clearEEPROM();
	USART_puts("ready\r");

	voltageCutOff = lowVoltageCell * numberOfCell;

	memset(currentSamples, 0, NUMBER_OF_SAMPLES);


	while (1) {


	}
}

/*
 * Interne prerusenie TIM4 (Timer, Casovac), vykonava sa v pravidelnom intervale (mame v initie nastavenych 500ms)
 * cela logika je tejto funkcie je zavesena na premennej 'startMeasureFlag', cize ak jeho hodnota je == 0, tak sa nic nedeje a bezi naprazdno..
 * Ked sa nastavi startMeasureFlag =1, tak sa zahaji cely proces merania,
 *
 * ospravedlnujem sa, je to divocina, ten kod by som najradsej cely prerobil, lenze cas sa krati..
 *
 * Zacnem od vrchu..
 *
 * pri podmienke if (voltageMeasureTime >= 10)  riesim kazdych 5sekund, ci napatie nekleslo pod kriticku uroven
 * 		Tych 5sekund je zamerne, pretoze pri nabehu sa ziarovka rozsvecuje a je tam prudky pokles napatia.. tazko sa to rozpisuje
 * 		Ked napatie klesne pod hranicu, baterka sa vybila a zastavi sa cely proces merania, a vyhodi stav measureStatus =3, co znamena ze 'stopped measure low battery'
 *
 * pri podmienke if (currentMeasureTime == 0) riesim kazdych 30sekund, zapis do EEPROM, ak je EEPROM plna, skonci meranie so stavom measureStatus = 2 'stopped measure, EEPROM overflow'
 * ak je vsetko ok, hodi measureStatus = '1' co znamena  measuring;
 *
 * //0 - not measuring; 1 - measuring; 2 - stopped measure, EEPROM overflow; 3 - stopped measure low battery;
 *
 * To je vsetko.
 */
void TIM4_IRQHandler(void) {
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {

		if (startMeasureFlag == 1) {

			//curent time counter reset//29
			if(currentMeasureTime >= 6){
				currentMeasureTime=0;
			}

			//get 5second delay for undervoltage protection
			if (voltageMeasureTime >= 10) {

				if (getVoltage() <= 10.2) {
					stopMeasure();
					measureStatus = 3;
					voltageMeasureTime = 0;
					currentMeasureTime = 0;
					GPIO_SetBits(GPIOA, GPIO_Pin_7);
					TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
					return;
				}
				voltageMeasureTime = 0;
			}

			//measure on current every 30seconds
			if (currentMeasureTime == 0) {
				GPIO_ResetBits(GPIOA, GPIO_Pin_7);
				//reset counter if EEPROM is full
				if (eepromCurrentPosition >= 255) {
					stopMeasure();
					measureStatus = 2;
					GPIO_SetBits(GPIOA, GPIO_Pin_7);
					TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
					return;
				}else{
					writeEEPROMByte(eepromCurrentPosition,(uint8_t) actualCurrent); //write current into EEPROM
					eeprom[eepromCurrentPosition]=actualCurrent;
					eepromCurrentPosition++;
					measureStatus = 1;
					GPIO_SetBits(GPIOA, GPIO_Pin_7);
				}
			}

			currentMeasureTime++;
			voltageMeasureTime++;
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

/*
 * Interne prerusenie (Timer, Casovac), vykonava sa v pravidelnom intervale (mame v initie nastavenych 50ms)
 * Tu sa riesi aky aktualny prud tecie obvodom (ziarovkou).
 * Cela veda je ta, ze zoberiem 10vzoriek, (aktualneho prudu) a spravim z nich aritmeticky priemer..
 * Je to z toho titulu, ze ten nas odbornik nam dal nevhodny Hallov senzor, (pre prudy +- 12Amperov), kde u nas je max.prud 2Ampere
 * preto nejake desatinky ampera hore dole ten senzor nejak netrapii a dost tie hodnoty potom skacu.. proste no-comment
 *
 * Ked sa spravi 10vzoriek tak sa spravi priemer, prenasobi odcita bulharskymi konstantami a dostaneme prud.
 *
 * Este pripomeniem, ze prerusenie nastava kazdych 50ms lenze ak chcem 10 vzoriek tak potom aktualny prud viem az o 500ms..
 *
 * 50ms je tam preto taka doba, lebo som niekde cital, ze ADC prevodnik mensie intervaly nezvlada (proste vracia cudne hodnoty)
 * ale nie som si tym isty..
 */
void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		//get 10 samples, save it into array
		if (sampleCounter < 10) {
			currentSamples[sampleCounter] = Read_AD_Value(ADC_Channel_0);
			sampleCounter++;
		}

		// if we got 10 samples then make an average
		else {
			i = 0;
			while (i < NUMBER_OF_SAMPLES) {
				current += currentSamples[i];
				i++;
			}
			current = current / NUMBER_OF_SAMPLES;
			actualCurrent = fabs(current - 3095); //actualCurrent = abs(current - 3095) * 0.0165;
			sampleCounter = 0;
			current = 0.00;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/*
 * USART prerusenie, ak pride na UART nejaky znak, prerusi sa cely kod a vykona sa prave tato funkcia.
 * jedina jej starost je, aby ulozila prijaty znak do globalnej premennej receivedChar.
 */
void USART1_IRQHandler(void) {
	if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
			static uint8_t cnt = 0; // this counter is used to determine the string length
			char t = USART1->DR; // the character from the USART1 data register is saved in t
			if ((t != '\n') && (cnt < MAX_STRLEN)) {
				received_string[cnt] = t;
				cnt++;
			} else {
				cnt = 0;
				//USART_puts(USART1, USART_Handle(received_string));
				handleUSARTCommands();
				for (uint8_t i = 0; i <= MAX_STRLEN + 1; i++) {
					received_string[i] = '\0';
				}
			}
		}
}

/*
 * funkcia ktora dokaze poslat aj String cez UART
 * cize nie len pismenka ale aj cele slovo..
 */

void USART_puts(volatile char *s) {

	while (*s) {
		// wait until data register is empty
		while (!(USART1->SR & 0x00000040))
			;
		USART_SendData(USART1, *s);
		*s++;
	}
}

/*Funkcia, ktora po zavolani spusti proces merania, nastavia sa flagy ktore sa riesia v TIM4_IRQHandler
* Zapne ziarovku, zatazovy obvod..
* (zapne pin PA6, ktory napa rele a ten zase ziarovku)
*
*/
void startMeasure() {
	measureStatus = 0;
	voltageMeasureTime = 0;
	startMeasureFlag = 1;
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
}

/*
 * Funkcia, ktora po zavolani zastavi celkovo proces merania, nastavi sa startMeasureFlag a aj measureStatus na nulu.
 * A natvrdo sa odpoji ziarovkovy, zatazovy obvod..
 * (vypne pin PA6, ktory napa rele a ten zase ziarovku)
 * Viem ze pre vypnutie tam nema byt Set ale Reset, lenze to rele ma prevratenu logiku.. don't care
 */
void stopMeasure() {
	startMeasureFlag = 0;
	measureStatus = 0;
	GPIO_SetBits(GPIOA, GPIO_Pin_6);

}

void handleUSARTCommands(){
	char str[10];
	if (strcmp(received_string, "at\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		USART_puts("OK\r");
		STM_EVAL_LEDOff(LED2);
		return;
	}
	if (strcmp(received_string, "amp\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		sprintf(str, "%.3f\r", actualCurrent);
		USART_puts(str);
		STM_EVAL_LEDOff(LED2);
		return;
		//Ak pride 'v'(Voltage), tak mi hod naspat aktualne napatie baterky, a blikni ledkou..
	}
	if (strcmp(received_string, "volt\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		sprintf(str, "%.3f\r", getVoltage());
		USART_puts(str);
		STM_EVAL_LEDOff(LED2);
		return;
		//Ak pride 'm' (Measure status), tak mi hod naspat stav, ci sa meria/nemeria/kalibruje.. proste aby som vedel
		//ze ci nieco robi :P
	}
	if (strcmp(received_string, "ms\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		sprintf(str, "%i\r", measureStatus);
		USART_puts(str);
		STM_EVAL_LEDOff(LED2);
		return;
		//Ak pride 'p' (Possition), tak mi hod naspat aktualny stav naplnenia EEPROM. priklad, odosle mi ze
		// uz som zapisal na 100tu poziciu z 256..
	}
	if (strcmp(received_string, "pos\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		sprintf(str, "%i\r", eepromCurrentPosition);
		USART_puts(str);
		STM_EVAL_LEDOff(LED2);
		return;
		//Ak pride 's' (Start measure), tak spusti meraciu proceduru, zavola sa funkcia startMeasure();
	}
	if (strcmp(received_string, "startM\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		startMeasure();
		STM_EVAL_LEDOff(LED2);
		return;
		//Ak pride 'e' (End measure), tak ukonci meraciu proceduru, zavola sa funkcia stopMeasure();
	}
	if (strcmp(received_string, "stopM\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		stopMeasure();
		STM_EVAL_LEDOff(LED2);
		return;
	}
	if (strcmp(received_string, "startC\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		GPIO_ResetBits(GPIOA, GPIO_Pin_6);
		STM_EVAL_LEDOff(LED2);
		return;
		//Ak pride 'e' (End measure), tak ukonci meraciu proceduru, zavola sa funkcia stopMeasure();
	}
	if (strcmp(received_string, "stopC\r") == 0) {
		STM_EVAL_LEDOn(LED2);
		memset(received_string, '0', sizeof(received_string));
		GPIO_SetBits(GPIOA, GPIO_Pin_6);
		STM_EVAL_LEDOff(LED2);
		return;
	}
	char src[10];
	char dest[6];
	memset(dest, '\0', sizeof(dest));
	strcpy(src, received_string);
	strncpy(dest, src, 4);
	if (strcmp(dest, "read") == 0) {
		STM_EVAL_LEDOn(LED2);
		char subbuff[6];
		memcpy(subbuff, &received_string[4], 6);
		subbuff[5] = '\0';
		char s[5];
		strcpy(s, replace(subbuff, '\r', '\0'));
		u_int8_t value = 0;
		value = atoi(s);
		sprintf(str, "%i\r", eeprom[value]);
		USART_puts(str);
		memset(received_string, '0', sizeof(received_string));
		STM_EVAL_LEDOff(LED2);
		return;
	}
}


/*
 * funkcia, ktora mi pri zavolani vypluje aktualne napatie vo Voltoch
 * je to prenasobene bulharskou konstantou 0.0065 kt. som ziskal empirickym meranim
 */
float getVoltage(void) {
	return Read_AD_Value(ADC_Channel_1) * 0.0065;
}

void clearEEPROM() {
	for (uint16_t j = 0; j < 256; j++) {
		FLASHStatus = writeEEPROMByte(j, (uint8_t) 0);
		eeprom[j]=0;
	}
	eepromCurrentPosition = 0;
}

FLASH_Status writeEEPROMByte(uint32_t address, uint8_t data) {
    FLASH_Status status = FLASH_COMPLETE;
    address = address + 0x08080000;
    DATA_EEPROM_Unlock();  //Unprotect the EEPROM to allow writing
    status = DATA_EEPROM_ProgramByte(address, data);
    DATA_EEPROM_Lock();  // Reprotect the EEPROM
    return status;
}

uint8_t readEEPROMByte(uint32_t address) {
    uint8_t tmp = 0;
    address = address + 0x08080000;
    tmp = *(__IO uint32_t*)address;

    return tmp;
}

char *replace(const char *s, char ch, const char *repl) {
    int count = 0;
    const char *t;
    for(t=s; *t; t++)
        count += (*t == ch);

    size_t rlen = strlen(repl);
    char *res = malloc(strlen(s) + (rlen-1)*count + 1);
    char *ptr = res;
    for(t=s; *t; t++) {
        if(*t == ch) {
            memcpy(ptr, repl, rlen);
            ptr += rlen;
        } else {
            *ptr++ = *t;
        }
    }
    *ptr = 0;
    return res;
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
	while (1)
	{
	}
}
#endif

