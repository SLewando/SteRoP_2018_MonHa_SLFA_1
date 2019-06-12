/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "dma.h"
#include "lcd.h"
#include "quadspi.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "stm32l476g_discovery_qspi.h"
#include "menu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_DANYCH 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RTC_DateTypeDef RTC_Calendar;
RTC_TimeTypeDef RTC_Time;
int32_t RecBuff[2048];

double const filtr[64]={0.00234434917719461,0.00253607247271912,0.00285229122268397,0.00329035375942028,0.00384640000611839,0.00451539722262479,0.00529118751439567,0.00616654664968141,0.00713325361488786,0.00818217022853550,0.00930333003144620,0.0104860355757936,0.0117189631494397,0.0129902738954427,0.0142877302205574,0.0155988163316447,0.0169108616957447,0.0182111661885955,0.0194871256779324,0.0207263567821822,0.0219168195522430,0.0230469368438535,0.0241057091804112,0.0250828239506856,0.0259687578422278,0.0267548714788497,0.0274334953086374,0.0279980058767921,0.0284428917142534,0.0287638081775825,0.0289576206868904,0.0290224359255544,0.0289576206868904,0.0287638081775825,0.0284428917142534,0.0279980058767921,0.0274334953086374,0.0267548714788497,0.0259687578422278,0.0250828239506856,0.0241057091804112,0.0230469368438535,0.0219168195522430,0.0207263567821822,0.0194871256779324,0.0182111661885955,0.0169108616957447,0.0155988163316447,0.0142877302205574,0.0129902738954427,0.0117189631494397,0.0104860355757936,0.00930333003144620,0.00818217022853550,0.00713325361488786,0.00616654664968141,0.00529118751439567,0.00451539722262479,0.00384640000611839,0.00329035375942028,0.00285229122268397,0.00253607247271912,0.00234434917719461,0.00227854094737764};
double filtrowany[2048];

volatile int JCenter_flag = 0;
volatile int JUp_flag = 0;
volatile int JRight_flag = 0;
volatile int JLeft_flag = 0;
volatile int JDown_flag = 0;

int32_t ProgDzwieku = 600000;
int OkresProbkowania = 10; // s
int okres_probki=1;
int32_t curtime=0,timval=0;
double statystyka[3600];
_Bool flagaprobek=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
int _write(int file, char *ptr, int len);

/*-----------------  FUNKCJE CALLBACK MENU ---------------------------*/
// Prototypy spisane w main.h

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  BSP_LCD_GLASS_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LCD_Init();
  MX_QUADSPI_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_DFSDM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim6);

	RTC_Calendar.Year = 19;
	RTC_Calendar.Month = 06;
	RTC_Calendar.Date = 12;

	HAL_RTC_SetDate(&hrtc, &RTC_Calendar, RTC_FORMAT_BIN);

	HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);

	RTC_Time.Hours = 0;
	RTC_Time.Minutes = 0;
	RTC_Time.Seconds = 0;

	HAL_RTC_SetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);

	/* Start DFSDM conversions */
	if (HAL_OK!= HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff,2048)) {
		Error_Handler();
	}
	if(BSP_QSPI_Init() != QSPI_OK){
		Error_Handler();
	}
	/* Start DFSDM conversions */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Menu_Odswiez();

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  if(JRight_flag == 1){
			 Menu_Nast();
			 JRight_flag = 0;
		  }
		  if(JLeft_flag == 1){
			 Menu_Poprz();
			 JLeft_flag  = 0;
		  }
		  if(JDown_flag == 1){
			  Menu_Dziecko();
			 JDown_flag  = 0;
		  }
		  if(JUp_flag == 1){
		  		 // Przerwanie uzywane do wyjscia z dzialajacej funkcji menu
			  	 // lub przejscia do rodzica pozycji menu
			  	 Menu_Rodzic();
		  		 JUp_flag  = 0;
		  	  }
		  if(JCenter_flag == 1){
			 Menu_Funkcja();
			 JCenter_flag  = 0;
		  }

	}//koniec while'a
	if(BSP_QSPI_DeInit() != QSPI_OK){
		Error_Handler();
	}
	BSP_LCD_GLASS_DeInit();
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == JCenter_EXTI0_Pin){
		JCenter_flag = 1;
	}
	if(GPIO_Pin == JLeft_EXTI1_Pin){
		JLeft_flag = 1;
	}
	if(GPIO_Pin == JRight_EXTI2_Pin){
		JRight_flag = 1;
	}
	if(GPIO_Pin == JUp_EXTI2_Pin){
		JUp_flag = 1;
	}
	if(GPIO_Pin == JDown_EXTI5_Pin){
		JDown_flag = 1;
	}
}

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, ptr, len, 50);
	return len;
}

/*-----------------  FUNKCJE CALLBACK MENU ---------------------------*/

void Prog_callback(void){
	// ZMIENNE MIKROFONU
	//double const filtr[64]={0.00234434917719461,0.00253607247271912,0.00285229122268397,0.00329035375942028,0.00384640000611839,0.00451539722262479,0.00529118751439567,0.00616654664968141,0.00713325361488786,0.00818217022853550,0.00930333003144620,0.0104860355757936,0.0117189631494397,0.0129902738954427,0.0142877302205574,0.0155988163316447,0.0169108616957447,0.0182111661885955,0.0194871256779324,0.0207263567821822,0.0219168195522430,0.0230469368438535,0.0241057091804112,0.0250828239506856,0.0259687578422278,0.0267548714788497,0.0274334953086374,0.0279980058767921,0.0284428917142534,0.0287638081775825,0.0289576206868904,0.0290224359255544,0.0289576206868904,0.0287638081775825,0.0284428917142534,0.0279980058767921,0.0274334953086374,0.0267548714788497,0.0259687578422278,0.0250828239506856,0.0241057091804112,0.0230469368438535,0.0219168195522430,0.0207263567821822,0.0194871256779324,0.0182111661885955,0.0169108616957447,0.0155988163316447,0.0142877302205574,0.0129902738954427,0.0117189631494397,0.0104860355757936,0.00930333003144620,0.00818217022853550,0.00713325361488786,0.00616654664968141,0.00529118751439567,0.00451539722262479,0.00384640000611839,0.00329035375942028,0.00285229122268397,0.00253607247271912,0.00234434917719461,0.00227854094737764};
	int64_t	max;
	//double filtrowany[2048];

	// ZMIENNE FLASH
	// ZAPIS
	uint8_t DaneDoZapisu[N_DANYCH] = {};		// Dane w formacie szesciu liczb uint
	const uint8_t *PtrDaneZapis = &DaneDoZapisu;		// Wskaznik na pierwszy elem. tablicy
	uint32_t AdresKomorki = 0x00; 				// Adres gdzie zacznie sie zapis
	uint32_t RozmiarPaczkiDanych = N_DANYCH * sizeof(uint8_t);
	// ODCZYT
	uint8_t Odczytano[N_DANYCH];		// Tablica do ktorej zapisze sie odczytane dane
	const uint8_t *PtrOdczytano = &Odczytano;	// Wskaznik na pierwszy elem. tablicy odczytu
	// Block ma 4 kB -> przyjmuje 499 logow(data zajmie 1 B)
	if(BSP_QSPI_Erase_Block(AdresKomorki) != QSPI_OK){	// Wyczyszczenie bloku, z pierwszym bajtem
		Error_Handler();
	}

	HAL_RTC_GetDate(&hrtc, &RTC_Calendar, RTC_FORMAT_BIN); //pobranie do RTC_Calendar daty
	printf("Data Data rtc:%d.%d.20%d\r\n", RTC_Calendar.Date, RTC_Calendar.Month, RTC_Calendar.Year);
	DaneDoZapisu[0] = RTC_Calendar.Date;
	DaneDoZapisu[1] = RTC_Calendar.Month;
	DaneDoZapisu[2] = RTC_Calendar.Year;

	if (BSP_QSPI_Write(PtrDaneZapis, AdresKomorki, RozmiarPaczkiDanych) != QSPI_OK){
		printf("Blad zapisu!\r\n");
	}
	AdresKomorki += RozmiarPaczkiDanych;

	BSP_LCD_GLASS_DisplayString("LOGUJE");
	while(!JUp_flag){
		//filtr Fir srodkowo przepustowy (300-300 000)hz
		for (int i3 = 0; i3 < 2048; i3++){
			filtrowany[i3] = 0;//zerowanie po poprzedniej filtracji
			 for (int i4 = 0; i4 < 64; i4++){
				if (i3>=i4){
					filtrowany[i3]=filtrowany[i3]+(  ( (double)RecBuff[i3-i4] )  *filtr[i4]);
				}
			}
		}
		//szukanie maximum
		max = 0;
		for (int var = 0; var < 1600; ++var) {
			if (filtrowany[var]>max){
				max = filtrowany[var];
			}
		}
		//trigger
		if (max >= ProgDzwieku){
			HAL_GPIO_TogglePin(LD_R_GPIO_Port, LD_R_Pin);
			HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN); //zaktualizowanie czasu RTC_Time
			printf(" Czas rtc: %dh\t%dm\t%ds \r\n", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);


			DaneDoZapisu[0] = RTC_Time.Hours;
			DaneDoZapisu[1] = RTC_Time.Minutes;
			DaneDoZapisu[2] = RTC_Time.Seconds;
			if (BSP_QSPI_Write(PtrDaneZapis, AdresKomorki, RozmiarPaczkiDanych) != QSPI_OK){
				printf("Blad zapisu!\r\n");
			}
			AdresKomorki += RozmiarPaczkiDanych;
		}
	} // Koniec logowania przekraczania progu
	BSP_LCD_GLASS_DisplayString("KONIEC");
	HAL_Delay(500);
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);

	// Odczytywanie logow
	if (BSP_QSPI_Read(PtrOdczytano, 0x00, RozmiarPaczkiDanych) == QSPI_OK){
		printf("Data: %d.%d.%d \r\n", Odczytano[0], Odczytano[1], Odczytano[2]);
	} else {
		printf("Blad odczytu!\r\n");
	}
	uint8_t buffor[6];
	for(uint32_t i = 0x03; i < AdresKomorki; i+= RozmiarPaczkiDanych){
		sprintf(buffor, "LOG %d", i/3);
		BSP_LCD_GLASS_DisplayString(buffor);
		HAL_Delay(500);
		if (BSP_QSPI_Read(PtrOdczytano, i, RozmiarPaczkiDanych) == QSPI_OK){
			printf("Log %d: %dh\t%dm\t%ds \r\n", i/3, Odczytano[0], Odczytano[1], Odczytano[2]);
//zapewnienie formatu hhmmss
			if (Odczytano[0]<10){
				if (Odczytano[1]<10){

					if (Odczytano[2]<10){
						sprintf(buffor, "0%d0%d0%d", Odczytano[0], Odczytano[1], Odczytano[2]);
					}else{
						sprintf(buffor, "0%d0%d%d", Odczytano[0], Odczytano[1], Odczytano[2]);
					}

				}else{

					if (Odczytano[2]<10){
						sprintf(buffor, "0%d%d0%d", Odczytano[0], Odczytano[1], Odczytano[2]);
					}else{
						sprintf(buffor, "0%d%d%d", Odczytano[0], Odczytano[1], Odczytano[2]);
					}

				}
			}else{
				if (Odczytano[1]<10){

					if (Odczytano[2]<10){
						sprintf(buffor, "%d0%d0%d", Odczytano[0], Odczytano[1], Odczytano[2]);
					}else{
						sprintf(buffor, "%d0%d%d", Odczytano[0], Odczytano[1], Odczytano[2]);
					}

				}else{

					if (Odczytano[2]<10){
						sprintf(buffor, "%d%d0%d", Odczytano[0], Odczytano[1], Odczytano[2]);
					}else{
						sprintf(buffor, "%d%d%d", Odczytano[0], Odczytano[1], Odczytano[2]);
					}
				}
			}
			//alternatywa do wyœwietlania bez zapewnienia formatu hhmmss
			//sprintf(buffor, "%d%d%d", Odczytano[0], Odczytano[1], Odczytano[2]);
			//koniec wyœwietlania pomiarów

			BSP_LCD_GLASS_DisplayString(buffor);
			HAL_Delay(1000);
		} else {
			printf("Blad odczytu!\r\n");
			BSP_LCD_GLASS_DisplayString("BLAD");
		}
	}
	// DEINIT
}

//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
//***************************************************************************************************************************************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
if (htim==&htim6 && flagaprobek==1){

	timval=timval+1;
	//printf(" timval : %d \r\n", timval);
	HAL_GPIO_TogglePin(LD_G_GPIO_Port,LD_G_Pin);
	}
}
void Okres_callback(void){
	flagaprobek=1;
	printf("Rozpoczynam okresowe zbieranie probek\r\n");

	// ZMIENNE MIKROFONU
	uint32_t srednia = 0;
	double AVERAGE = 0;
	uint32_t licznik=0,lasttime=0;
	//double const filtr[64]={0.00234434917719461,0.00253607247271912,0.00285229122268397,0.00329035375942028,0.00384640000611839,0.00451539722262479,0.00529118751439567,0.00616654664968141,0.00713325361488786,0.00818217022853550,0.00930333003144620,0.0104860355757936,0.0117189631494397,0.0129902738954427,0.0142877302205574,0.0155988163316447,0.0169108616957447,0.0182111661885955,0.0194871256779324,0.0207263567821822,0.0219168195522430,0.0230469368438535,0.0241057091804112,0.0250828239506856,0.0259687578422278,0.0267548714788497,0.0274334953086374,0.0279980058767921,0.0284428917142534,0.0287638081775825,0.0289576206868904,0.0290224359255544,0.0289576206868904,0.0287638081775825,0.0284428917142534,0.0279980058767921,0.0274334953086374,0.0267548714788497,0.0259687578422278,0.0250828239506856,0.0241057091804112,0.0230469368438535,0.0219168195522430,0.0207263567821822,0.0194871256779324,0.0182111661885955,0.0169108616957447,0.0155988163316447,0.0142877302205574,0.0129902738954427,0.0117189631494397,0.0104860355757936,0.00930333003144620,0.00818217022853550,0.00713325361488786,0.00616654664968141,0.00529118751439567,0.00451539722262479,0.00384640000611839,0.00329035375942028,0.00285229122268397,0.00253607247271912,0.00234434917719461,0.00227854094737764};
	//double filtrowany[2048];
	timval=0;
	// ZMIENNE FLASH
	// ZAPIS
	uint32_t DaneDoZapisu;
	const uint32_t *PtrDaneZapis = &DaneDoZapisu;		// Wskaznik na pierwszy elem. tablicy
	uint32_t AdresKomorki = 0x1000; 				// Adres gdzie zacznie sie zapis; zacznij od bloku wyzej niz Tryb Progu
	uint32_t RozmiarWartosciSredniej = sizeof(uint32_t);
	// ODCZYT
	uint32_t Odczytano;		// Tablica do ktorej zapisze sie odczytane dane
	const uint32_t *PtrOdczytano = &Odczytano;	// Wskaznik na pierwszy elem. tablicy odczytu
	// Block ma 4 kB
	if(BSP_QSPI_Erase_Block(AdresKomorki) != QSPI_OK){	// Wyczyszczenie bloku, z pierwszym bajtem
		Error_Handler();
	}

	BSP_LCD_GLASS_DisplayString("LOGUJE");
	while(!JUp_flag){
		HAL_Delay(100);


		if (OkresProbkowania<=timval) {
			printf(" timval : %d \r\n", timval);
			timval=0;

			AVERAGE=(double)0;
			for (int var = 0; var < licznik; ++var) {
				AVERAGE+=statystyka[var];
			}
			AVERAGE/=licznik;//œrednia z pomiarów w danym okresie

			//kod zapisuj¹cy do pamiêci
			srednia = (uint32_t) AVERAGE;
			// audio mambo jumbo
			DaneDoZapisu = srednia;
			printf(" Srednia do zapisu : %d \r\n", DaneDoZapisu);
			if (BSP_QSPI_Write(PtrDaneZapis, AdresKomorki, RozmiarWartosciSredniej) != QSPI_OK){
			printf("Blad zapisu!\r\n");
			}
			AdresKomorki += RozmiarWartosciSredniej;
			licznik=0;
			HAL_GPIO_TogglePin(LD_R_GPIO_Port, LD_R_Pin);

			//znacznik
		}else{
			if (timval>=(lasttime+okres_probki)){
				lasttime=timval;//zapisanie kiedy odby³ siê ostatni pomiar
				//kod pridukuj¹cy pomiary
				HAL_GPIO_TogglePin(LD_R_GPIO_Port,LD_R_Pin);

				// zbieranie pomiarow i filtracja
				for (int i3 = 0; i3 < 2048; i3++){
					filtrowany[i3] = 0;		//zerowanie po poprzedniej filtracji
					 for (int i4 = 0; i4 < 64; i4++){
						if (i3>=i4){
							filtrowany[i3]=filtrowany[i3]+(  ( (double)RecBuff[i3-i4] )  *filtr[i4]);
						}
					}
				}
				//statystyka.
				AVERAGE = 0;
				for (int var = 0; var < 1600; ++var) {
						AVERAGE += filtrowany[var];
					}
				AVERAGE/=1600;
				statystyka[licznik++]=AVERAGE;
			}
		}
	} // Koniec logowania sredniej z okresu

	BSP_LCD_GLASS_DisplayString("KONIEC");
	HAL_Delay(500);
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);

	// Odczytywanie logow
	uint8_t buffor[6];
	int k = 0; // zmienna pomocnicza, do wyswietlania numeru logu
	for(uint32_t i = 0x1000; i < AdresKomorki; i+= RozmiarWartosciSredniej){
		k++;
		sprintf(buffor, "LOG %d", k);
		BSP_LCD_GLASS_DisplayString(buffor);
		HAL_Delay(500);
		if (BSP_QSPI_Read(PtrOdczytano, i, RozmiarWartosciSredniej) == QSPI_OK){
			printf(" Srednia wynosi: %d \r\n", *PtrOdczytano);
			sprintf(buffor, "%d", Odczytano);
			BSP_LCD_GLASS_DisplayString(buffor);
			HAL_Delay(1000);
		} else {
			printf("Blad odczytu!\r\n");
			BSP_LCD_GLASS_DisplayString("BLAD");
		}
	}
	// DEINIT
	flagaprobek=0;
	printf("Koncze zbieranie probek\r\n");
}

void Wyswietl_Prog_callback(void){
	uint8_t* WartoscProgu[10]={};
	sprintf(WartoscProgu, "%d", ProgDzwieku);
	BSP_LCD_GLASS_DisplayString(WartoscProgu);
	HAL_Delay(1000);
}

void Wyswietl_Okres_callback(void){
	uint8_t* WartoscOkresu[10]={};
	sprintf(WartoscOkresu, "%d", OkresProbkowania);
	BSP_LCD_GLASS_DisplayString(WartoscOkresu);
	HAL_Delay(1000);
}
/*-----------------  FUNKCJE CALLBACK MENU ---------------------------*/


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printf("Error Handler\r\n");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
