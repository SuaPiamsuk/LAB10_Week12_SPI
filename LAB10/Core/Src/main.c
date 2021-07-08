/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> //printf
#include <string.h> //str lenght
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t ADCin = 0;
uint64_t _micro = 0;

//12bits dac
uint16_t dataOut = 0;
//config 4 bit
uint8_t DACConfig = 0b0011;


//uart
char TxDataBuffer[32] = {0};

char RxDataBuffer[32] = {0};

int16_t inputchar2=0;

uint8_t waveform = 0;
float period   = 100.00;
float freq_saw  = 1.0;
float freq_sine = 1.0;
float freq_sqr  = 1.0;



float  Vmax   = 3.3;
float  Vmin   = 0.001;
float ADCmax_saw = 4096;
float ADCmin_saw = 0.001;


char slope_char[10] ="UP";
uint8_t slope = 1;

//sine wave
float degree = 0.001;
float Amplitude = 3.3;
float ref = 0.0001;
float ADCmax_sin = 4096;
float ADCmin_sin = 0.001;
float  Vmax_sine   = 3.3;
float  Vmin_sine   = 0.001;

//square wave
uint16_t ON_time = 0;
uint16_t Dutycycle = 1;
float ADCmax_sqr = 4096;
float ADCmin_sqr = 0.001;
float  Vmax_sqr = 3.3;
float  Vmin_sqr = 0.001;


uint64_t timestamp = 0;
////////////////////////////////

typedef enum
{
	Main_Menu_Print,
	Main_Menu_Select,
	Menu_1_Print,
	Menu_1_Select,
	Menu_2_Print,
	Menu_2_Select,
	Menu_3_Print,
	Menu_3_Select,

}State_machine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();

//UART
int16_t UARTRecieveIT();


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim11);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1);
  //  HAL_ADC_Start_IT(&hadc1);

  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);


  //UART
//  {
// 	  char temp[] = "HELLO WORLD\r\n please type something to test UART\r\n";
// 	  HAL_UART_Transmit_IT(&huart2,(uint8_t*) temp, strlen(temp));
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		  //////////////////////////////////////////////////////////////////////////


		  HAL_UART_Receive_IT(&huart2, (uint8_t*)RxDataBuffer, 32);

		  int16_t inputchar = UARTRecieveIT();

		  static State_machine state = Main_Menu_Print;

		  switch(state)
		  {
			  case Main_Menu_Print:
				sprintf(TxDataBuffer, "\n--------------------------\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, "  Choose Waveform\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, "--------------------------\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, "[1] Sawtooth\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, "[2] Sine Wave\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, "[3] Square Wave\n\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				state = Main_Menu_Select;
				break;

			  case Main_Menu_Select:

				switch(inputchar)
				{
					case -1 :
						break;
					case '1':
						period = (1000000/(freq_saw * (ADCmax_saw - ADCmin_saw)));
						state = Menu_1_Print;
						break;
					case '2':
						Amplitude 	= (ADCmax_sin - ADCmin_sin)/2.0;
						ref  		= (ADCmax_sin + ADCmin_sin)/2.0;
//						period 		= 1000000 / (freq_sine*6.28);
//						period = 100;
						state = Menu_2_Print;
						break;
					case '3':
	//					wave = 3;
	//					period = 1090000 / (freqx10[3]*100/10);
						state = Menu_3_Print;
						break;
					case 'x':
//						sprintf(TxDataBuffer, "No More Back\n\r");
//						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
//						state = Main_Menu_Print;
						break;
					default:
						sprintf(TxDataBuffer, "\nOnly [1][2][3] Key Available\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						state = Main_Menu_Print;
						break;
				}
				break;

			case Menu_1_Print:

				sprintf(TxDataBuffer, "\n                   >>> Saw tooth <<<   \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, " ______________________________________________________________\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, " V_high    = %.1f  V  | Press[a]-> '+' | Press[d]-> '-' |\n\r", Vmax);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, " V_low     = %.1f  V  | Press[w]-> '+' | Press[s]-> '-' |\n\r", Vmin);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, " Frequency = %.1f Hz  | Press[q]-> '+' | Press[e]-> '-' |\n\r", freq_saw);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				sprintf(TxDataBuffer, " Slope     = %s      | Press[z]|    \n\r", slope_char);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 300);

				sprintf(TxDataBuffer, " Press -> [x] -> Back              \n\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);

				state = Menu_1_Select;
				break;

			case Menu_1_Select:
				switch(inputchar)
				{
					case -1 :  //not responding
						if(slope == 1)
						{
							if (micros() - timestamp >= period)
							{
									timestamp = micros();

									if(dataOut >= ADCmax_saw) //ADCmax_saw
									{
										dataOut = ADCmin_saw;
									}
									else
									{
										dataOut++;
									}
									if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin) == GPIO_PIN_SET)
									{
										MCP4922SetOutput(DACConfig, dataOut);
									}
							}
						}
						else if(slope == 0)
						{
							if (micros() - timestamp >= period)
							{
									timestamp = micros();

									if(dataOut <= ADCmin_saw) //ADCmax_saw
									{
										dataOut = ADCmax_saw;
									}
									else
									{
										dataOut--;
									}
									if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin) == GPIO_PIN_SET)
									{
										MCP4922SetOutput(DACConfig, dataOut);
									}
							}

						}

						break;
					case 'a' : //+Vmax

						Vmax += 0.1;
						if(Vmax > 3.3)
						{
							Vmax = 3.3;
							ADCmax_saw = 4096;
							state = Menu_1_Print;
							break;
						}
						ADCmax_saw += (4096*0.1/3.3);
						period = 1000000/(freq_saw*(ADCmax_saw - ADCmin_saw));

						state = Menu_1_Print;
						break;
					case 'd' : //-Vmax
						Vmax -= 0.1;
						if(Vmax-Vmin < 0.1)
						{
							Vmax  = Vmin + 0.1;
							ADCmax_saw = Vmax*4096/3.3;
							state = Menu_1_Print;
							break;
						}
						ADCmax_saw -= 124.12; //4096*0.1/3.3)
						period = (1000000/(freq_saw*(ADCmax_saw - ADCmin_saw)));
						state = Menu_1_Print;
						break;
					case 'w' : //+Vmin
						Vmin += 0.10;
						if(Vmax - Vmin < 0.1)
						{
							Vmin  = Vmax-0.1;
							ADCmin_saw = Vmin*4096/3.3;
							state = Menu_1_Print;
							break;
						}
						ADCmin_saw += 124.12;
						period = 1000000/(freq_saw*(ADCmax_saw - ADCmin_saw));
						state = Menu_1_Print;
						break;
					case 's' : //-Vmin
						Vmin -= 0.10;
						if(Vmin<0)
						{
							Vmin = 0.0001;
							ADCmin_saw = 0;
							state = Menu_1_Print;
							break;
						}
						ADCmin_saw -= 124.12;
						period = 1000000/(freq_saw*(ADCmax_saw - ADCmin_saw));
						state = Menu_1_Print;
						break;
					case 'q' : //+freq
						freq_saw += 0.1;
						if(freq_saw > 10)
						{
							freq_saw = 10;
						}
						period = 1000000/(freq_saw*(ADCmax_saw - ADCmin_saw));
						state = Menu_1_Print;
						break;
					case 'e' : //-freq
						freq_saw -= 0.1;
						if(freq_saw < 0)
						{
							freq_saw = 0;
						}
						period = 1000000/(freq_saw*(ADCmax_saw - ADCmin_saw));
						state = Menu_1_Print;
						break;
					case 'z' :
						if(slope == 1)
						{

							sprintf(slope_char, "DOWN");
							slope = !slope;
						}
						else
						{
							sprintf(slope_char, "UP");
							slope = !slope;
						}
						state =  Menu_1_Print;
						break;

					case 'x' :
						state = Main_Menu_Print;
						break;

					default:
						sprintf(TxDataBuffer, "\nTry again\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						state = Menu_1_Print;
						break;
				}
				break;


			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			case Menu_2_Print:

				sprintf(TxDataBuffer, "\n             >>> Sine wave <<<        \n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

				sprintf(TxDataBuffer, " ______________________________________________________________\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

				sprintf(TxDataBuffer, " V_high    = %.1f  V  | Press[a]-> '+' | Press[d]-> '-' |\n\r", Vmax_sine);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

				sprintf(TxDataBuffer, " V_low     = %.1f  V  | Press[w]-> '+' | Press[s]-> '-' |\n\r", Vmin_sine);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

				sprintf(TxDataBuffer, " Frequency = %.1f Hz  | Press[q]-> '+' | Press[e]-> '-' |\n\r", freq_sine);
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

				sprintf(TxDataBuffer, " Press -> [x] -> Back              \n\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

				state = Menu_2_Select;
				break;
			case Menu_2_Select :
				switch(inputchar)
				{
					case -1 :

						Amplitude 	= (ADCmax_sin - ADCmin_sin)/2.0;
						ref  		= (ADCmax_sin + ADCmin_sin)/2.0;
						period 		= ( 1000000 / (freq_sine));
						if (micros() - timestamp >=  (1000000) / (freq_sine*(6.28/0.01)) )
				        {
								timestamp = micros();

								if(degree >= 2*3.14)  //2*3.14 = 2pi
								{
									degree = 0;
								}
								else
								{
									degree += 0.01;
								}
			     				dataOut = ( sin(degree)*Amplitude ) + ref;

								if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin) == GPIO_PIN_SET)
								{
									MCP4922SetOutput(DACConfig, dataOut);
								}
				        }
						break;
					case 'a' :
						Vmax_sine += 0.1;
						if(Vmax_sine > 3.3)
						{
							Vmax_sine = 3.3;
							ADCmax_sin = 4096;
							state = Menu_2_Print;
							break;
						}
						ADCmax_sin += (4096*0.1/3.3);
						state = Menu_2_Print;
						break;

					case 'd' : //-Vmax
						Vmax_sine -= 0.1;
						if(Vmax_sine - Vmin_sine < 0.1)
						{
							Vmax_sine  = Vmin_sine + 0.1;
							ADCmax_sin = Vmax_sine*4096/3.3;
							state = Menu_2_Print;
							break;
						}
						ADCmax_sin -= 124.12; //4096*0.1/3.3)
						state = Menu_2_Print;
						break;
					case 'w' : //+Vmin
						Vmin_sine += 0.10;
						if(Vmax_sine - Vmin_sine < 0.1)
						{
							Vmin_sine  = Vmax_sine-0.1;
							ADCmin_sin = Vmin_sine*4096/3.3;
							state = Menu_2_Print;
							break;
						}
						ADCmin_sin += 124.12;
						state = Menu_2_Print;
						break;
					case 's' : //-Vmin
						Vmin_sine -= 0.10;
						if(Vmin_sine<0)
						{
							Vmin_sine = 0.0001;
							ADCmin_sin = 0;
							state = Menu_2_Print;
							break;
						}
						ADCmin_sin -= 124.12;
						state = Menu_2_Print;
						break;
					case 'q' : //+freq
						freq_sine += 0.1;
						if(freq_sine > 10)
						{
							freq_sine = 10;
						}
						state = Menu_2_Print;
						break;
					case 'e' : //-freq
						freq_sine -= 0.1;
						if(freq_sine < 0)
						{
							freq_sine = 0;
						}
						state = Menu_2_Print;
						break;
					case 'x' :
						state = Main_Menu_Print;
						break;
					default:
						sprintf(TxDataBuffer, "\nTry again\n\r");
						HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
						state = Menu_2_Print;
						break;
				}
				break;

			//////////////////////////////////////////////////////////////////////////////////////////////////

				case Menu_3_Print:

					sprintf(TxDataBuffer, "\n             >>> Square wave <<<        \n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

					sprintf(TxDataBuffer, " ______________________________________________________________\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

					sprintf(TxDataBuffer, " V_high    = %.1f  V  | Press[a]-> '+' | Press[d]-> '-' |\n\r", Vmax_sqr);
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

					sprintf(TxDataBuffer, " V_low     = %.1f  V  | Press[w]-> '+' | Press[s]-> '-' |\n\r", Vmin_sqr);
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

					sprintf(TxDataBuffer, " Frequency = %.1f Hz  | Press[q]-> '+' | Press[e]-> '-' |\n\r", freq_sqr);
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

					sprintf(TxDataBuffer, " Dutycycle = %d /100% | Press[z]-> '+' | Press[c]-> '-' |\n\r",Dutycycle);
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

					sprintf(TxDataBuffer, " Press -> [x] -> Back              \n\n\r");
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 50);

					state = Menu_3_Select;
					break;
				case Menu_3_Select :
					switch(inputchar)
					{
						case -1 :
							period = 1000000 / freq_sqr;
							if (micros() - timestamp > 1000000 / (freq_sqr*100))
							{
								timestamp = micros();
								if(ON_time >= 100)
								{
									ON_time = 0;
								}
								else
								{
									ON_time++;
								}

								if(ON_time <= Dutycycle)
								{
									dataOut = ADCmax_sqr*0.09/0.09;
								}
								else
								{
									dataOut = ADCmin_sqr*1;
								}

								if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin) == GPIO_PIN_SET)
								{
									MCP4922SetOutput(DACConfig, dataOut);
								}
							}



							break;
						case 'a' :
							Vmax_sqr += 0.1;
							if(Vmax_sqr > 3.3)
							{
								Vmax_sqr = 3.3;
								ADCmax_sqr = 4096;
								state = Menu_3_Print;
								break;
							}
							ADCmax_sqr += (4096*0.1/3.3);
							state = Menu_3_Print;
							break;

						case 'd' : //-Vmax
							Vmax_sqr -= 0.1;
							if(Vmax_sqr - Vmin_sqr < 0.1)
							{
								Vmax_sqr  = Vmin_sqr + 0.1;
								ADCmax_sqr = Vmax_sqr*4096/3.3;
								state = Menu_3_Print;
								break;
							}
							ADCmax_sqr -= 124.12; //4096*0.1/3.3)
							state = Menu_3_Print;
							break;
						case 'w' : //+Vmin
							Vmin_sqr += 0.10;
							if(Vmax_sqr - Vmin_sqr < 0.1)
							{
								Vmin_sqr  = Vmax_sqr-0.1;
								ADCmin_sqr = Vmin_sqr*4096/3.3;
								state = Menu_3_Print;
								break;
							}
							ADCmin_sqr += 124.12;
							state = Menu_3_Print;
							break;
						case 's' : //-Vmin
							Vmin_sqr -= 0.10;
							if(Vmin_sqr<0)
							{
								Vmin_sqr = 0.0001;
								ADCmin_sqr = 0;
								state = Menu_3_Print;
								break;
							}
							ADCmin_sqr -= 124.12;
							state = Menu_3_Print;
							break;
						case 'q' : //+freq
							freq_sqr += 0.1;
							if(freq_sqr > 10)
							{
								freq_sqr = 10;
							}
							state = Menu_3_Print;
							break;
						case 'e' : //-freq
							freq_sine -= 0.1;
							if(freq_sqr < 0)
							{
								freq_sqr = 0;
							}
							state = Menu_3_Print;
							break;
						case 'z' :
							Dutycycle++;
							if(Dutycycle > 99)
							{
								Dutycycle = 99;
							}
							state = Menu_3_Print;
							break;
						case 'c' :
							Dutycycle--;
							if(Dutycycle < 1)
							{
								Dutycycle = 1;
							}
							state = Menu_3_Print;
							break;
						case 'x' :
							state = Main_Menu_Print;
							break;
						default:
							sprintf(TxDataBuffer, "\nTry again\n\r");
							HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
							state = Menu_3_Print;
							break;
					}
			///////////////////////////
			default:
				break;



		  }
		  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* USER CODE BEGIN 3 */

//
//	  if(inputchar!=-1)
//	  {
//		sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
//		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer),50);
//	  }
//
//	  if(inputchar == 'a')
//	  {
//		  char temp[] = "Uuuuuuuuuu\r\n";
//		  HAL_UART_Transmit_IT(&huart2,(uint8_t*) temp, strlen(temp));
//	  }






//
//
//	  static uint64_t timestamp = 0;
//
//	  if (micros() - timestamp >= 1000)
//	  {
//			timestamp = micros();
////          saw tooth
//			dataOut = dataOut + 1;
//			dataOut %= 4096;
//
//			if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin) == GPIO_PIN_SET)
//			{
//				MCP4922SetOutput(DACConfig, dataOut);
//			}
//	  }




  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
inline uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim11)
	{
		_micro += 65535;
	}
}

////////////////////////////////////

void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}


//////////////////////////////////


int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
