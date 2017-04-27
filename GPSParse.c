/**
  ******************************************************************************
  * File Name     : GPSParse.c
  * PROJECT       : PROG8125 -Assignment #5
  * PROGRAMMER    : Abdelraheem Alkuor (Abdel)
  * FIRST VERSION : 11/Dec/2016
  * Description   :
  * 				* This program parses GPS data of type GPGGA and arrange the parsed data
  * 				   in an array to be displayed on LCD display.
  * 			   	   All the parsed numeric ASCII data is converted to integer
  * 			       except for altitude which is converted to float and for Checksum
  * 			       it gets converted to unsigned 8 bit integer.
  *
  * 			    ** This program also verifies the checksum of GPS DATA by XORing
  * 			       the original GPS Data before it gets parsed.
  * 				*** two pushbuttons are used one to scroll through the parsed data
  * 					(USER pushbutton), and another pushbutton to select GPS data
  * 					set (Connected on PE0 with taking care of debouncing)
  *
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "LCD1602A.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PD */
extern volatile uint8_t dataSelectPushButton;
#define	ASCII_0					48
#define ASCII_9					57
#define APASTROPHE 				39
#define BASE_16					16
#define BASE_10 				10
#define DATA_SCROLL_PUSHBUTTON 	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)
#define ONE_SECOND				1000
#define FIRST_ROW_DISPLAY		0
#define SECOND_ROW_DISPLAY		1
#define FIRST_COLUMN_DISPLAY	0
#define MATCH					0
#define INDEX_0					0
#define INDEX_1					1
#define INDEX_2					2
#define INDEX_3					3
#define INDEX_4					4
#define INDEX_5					5
#define INDEX_6					6
#define INDEX_7					7
#define INDEX_8					8
#define INDEX_9					9
#define INDEX_10				10
#define INDEX_11				11
#define INDEX_12				12
#define INDEX_13				13
#define INDEX_14				14
#define INDEX_15				15
#define INDEX_16				16
#define INDEX_17				17
#define INDEX_18				18
#define INDEX_19				19
#define INDEX_20				20
#define INDEX_21				21
#define DATA_SET_COUNTER_0		0
#define DATA_SET_COUNTER_1		1
#define DATA_SET_COUNTER_2		2
#define DATA_INDEX_1			1
#define DATA_INDEX_3			3
#define DATA_INDEX_5			5
#define DATA_INDEX_7			7
#define DATA_INDEX_9			9
#define DATA_INDEX_11			11
#define DATA_INDEX_13			13
#define DATA_INDEX_15			15
#define DATA_INDEX_17			17
#define DATA_INDEX_19			19
#define DATA_INDEX_21			21
#define RAW_DATA_ROWS			3
#define RAW_DATA_COLS			73
#define PARSED_ROWS				15
#define PARSED_COLS				11
#define SELECTED_SIZE			2
#define CHECKSUM_ROWS			1
#define CHECKSUM_COLS			4
#define FORMATED_ROWS			22
#define FORMATED_COLS			18
/* Private Defines ---------------------------------------------------------*/

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void parseGPSData(char GPSRawData[RAW_DATA_ROWS][RAW_DATA_COLS],char storeGPSParsedData[PARSED_ROWS][PARSED_COLS], uint8_t GPSdataSet );
void convertParsedGPSDataToIntegerFloat(char GPSParsedData[PARSED_ROWS][PARSED_COLS]);
void GPSDataFormat(char GPSParsedData[PARSED_ROWS][PARSED_COLS],char GPSParsedOutput[FORMATED_ROWS][FORMATED_COLS]);
uint8_t verifyCheckSum(char GPSRawData[RAW_DATA_ROWS][RAW_DATA_COLS],uint8_t dataSet);
/* USER CODE END PFP */


int main(void)
{
	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	LCD1602A_init();
	HAL_TIM_Base_Start_IT(&htim2);

	/* USER CODE BEGIN 2 */
	uint8_t calculatedCheckSum=0;
	char checksumString[CHECKSUM_ROWS][CHECKSUM_COLS]={0};

	uint8_t dataScrollCounter=RESET;
	uint8_t dataSetCounter=RESET;
	char selectedData[SELECTED_SIZE];
	char GPSData[RAW_DATA_ROWS][RAW_DATA_COLS]={0};
	//Copy the first set of GPS data
	strcpy(&GPSData[dataSetCounter][INDEX_0],"$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76");
	char GPSDataParsed[PARSED_ROWS][PARSED_COLS]={0}; //Store raw parsed data in this array

	//This array will be used to store parsed GPSDATA values in blank spaces
	char formatedGPSParsedData[FORMATED_ROWS][FORMATED_COLS]={
										 "sentenceID:",
										 " ",// <-------------e.g GPGGA
										 "UTC Time:",
										 " ",// <-------------e.g 09:27:50.0
										 "Latitude:",
										 " ",// <-------------e.g 53 deg 21.0000'N
										 "Longitude:",
										 " ",// <-------------e.g 006deg 30.0000'W
										 "Fix quality:",
										 " ",// <-------------e.g 1
										 "# of Satellites:",
										 " ",// <-------------e.g 8
										 "HDPO:",
										 " ",// <-------------e.g 1
										 "Altitude:",
										 " ",// <-------------e.g 61.70000M
										 "Height of Geoid:",
										 " ",//	<-------------e.g 55M
										 "DGPS Age:",
										 " ",//	<-------------e.g 5s
										 "Checksum:",
										 " "//	<-------------e.g 54
										};


	//By default First GPS data set will be parsed and Checksum will be verified
	LCD1602A_Clear();
	LCD1602A_Print(FIRST_ROW_DISPLAY, FIRST_COLUMN_DISPLAY, "GPS DATA SET:");
	itoa(dataSetCounter + 1, selectedData, BASE_10);// Display GPS data set Starting from 1 instead of 0
	LCD1602A_Print(SECOND_ROW_DISPLAY, FIRST_COLUMN_DISPLAY, selectedData);
	calculatedCheckSum = verifyCheckSum(GPSData, dataSetCounter); //verify checksum so it will be compared with the original checksum later
	itoa(calculatedCheckSum, &checksumString[dataScrollCounter][INDEX_0],BASE_10); // convert verified checksum to string
	parseGPSData(GPSData, GPSDataParsed, dataSetCounter); // parse the first GPS data set
	convertParsedGPSDataToIntegerFloat(GPSDataParsed); // covert all numeric ASCII to integer except for Altitude convert float
	GPSDataFormat(GPSDataParsed, formatedGPSParsedData); // fill out the blanks in the formatedGPSParsedData array
	dataSetCounter++;// prepare for next GPS data set to be parsed when dataSelectPushButton is pressed

	/* USER CODE END 2 */
	while (1)
	{
		/* USER CODE BEGIN 3 */
		if(DATA_SCROLL_PUSHBUTTON==GPIO_PIN_SET)
		{
			LCD1602A_Clear();
			LCD1602A_Print(FIRST_ROW_DISPLAY, FIRST_COLUMN_DISPLAY,&formatedGPSParsedData[dataScrollCounter][INDEX_0]);
			dataScrollCounter++;
			LCD1602A_Print(SECOND_ROW_DISPLAY, FIRST_COLUMN_DISPLAY,&formatedGPSParsedData[dataScrollCounter][INDEX_0]);
			dataScrollCounter++;
			if (dataScrollCounter-1 == INDEX_21)// verify checksum
			{
				HAL_Delay(ONE_SECOND);
				if(strcmp(&checksumString[INDEX_0][INDEX_0],&formatedGPSParsedData[dataScrollCounter-1][INDEX_0])==MATCH)
				{
					LCD1602A_Clear();
					LCD1602A_Print(FIRST_ROW_DISPLAY, FIRST_COLUMN_DISPLAY,"Checksum: Passed");
				}
				else
				{
					LCD1602A_Clear();
					LCD1602A_Print(FIRST_ROW_DISPLAY, FIRST_COLUMN_DISPLAY,"Checksum: Failed");
				}
				dataScrollCounter=RESET;
				calculatedCheckSum=0;
			}
		}

		if(dataSelectPushButton==SET)
		{
			switch (dataSetCounter)
			{
				case DATA_SET_COUNTER_0:
				//Re-copy the data to get parsed again
					strcpy(&GPSData[dataSetCounter][INDEX_0],"$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76");
					break;

				case DATA_SET_COUNTER_1:
			 	//Re-copy the data to get parsed again
					strcpy(&GPSData[dataSetCounter][INDEX_0],"$GPGGA,092751.000,5321.6802,N,00630.3371,W,1,8,1.03,61.7,M,55.3,M,,*75");
					break;

			  	case DATA_SET_COUNTER_2:
			  	//Re-copy the data to get parsed again
			  	default:
			  		 strcpy(&GPSData[dataSetCounter][INDEX_0],"$GPGGA,014729.10,4303.5753,N,08019.0810,W,1,6,1.761,214.682,M,0,M,0,*5D");
			  		 break;
			}
			LCD1602A_Clear();
			LCD1602A_Print(FIRST_ROW_DISPLAY, FIRST_COLUMN_DISPLAY,"GPS DATA SET:");
			itoa(dataSetCounter+1,selectedData,BASE_10);
			LCD1602A_Print(SECOND_ROW_DISPLAY, FIRST_COLUMN_DISPLAY,selectedData);

			calculatedCheckSum=verifyCheckSum(GPSData,dataSetCounter);// Find CheckSum before GPSDATA gets parsed
			itoa(calculatedCheckSum,&checksumString[dataScrollCounter][INDEX_0],BASE_10);
			parseGPSData(GPSData,GPSDataParsed,dataSetCounter);// dataSetCounter sets which GPSDATA gets parsed
			convertParsedGPSDataToIntegerFloat(GPSDataParsed);// covert all numeric ASCII to integer except for Altitude convert float
			GPSDataFormat(GPSDataParsed,formatedGPSParsedData);//Store GPS parsed Data in proper format
			dataSetCounter++;
			dataScrollCounter=RESET;// reset GPS data scroll counter to start displaying data from GPGGA

			if(dataSetCounter>DATA_SET_COUNTER_2)
			{
				dataSetCounter=RESET; //reset GPS data set counter
			}
			dataSelectPushButton=RESET;
		}
		/* USER CODE END 3 */
	}
}



/* USER CODE BEGIN 4 */

/* FUNCTION      : parseGPSData
 * DESCRIPTION   : This function parses GPS DATA with GPGGA data type
 *
 * PARAMETERS    : GPSRawData: data starts with $ and ends with *checksum
 * 				   storeGPSParsedData: array to store parsed GPS data
 * RETURNS       : NULL
*/

void parseGPSData(char GPSRawData[RAW_DATA_ROWS][RAW_DATA_COLS],char storeGPSParsedData[PARSED_ROWS][PARSED_COLS],uint8_t GPSdataSet)
{
	uint8_t index=RESET;
	char *dataParsed;
	dataParsed = strtok(&GPSRawData[GPSdataSet][INDEX_0],",");
	strcpy(&storeGPSParsedData[index][INDEX_0],dataParsed);

	while(*dataParsed!='\0')
	{	index++;
		dataParsed = strtok(NULL,",");
		strcpy(&storeGPSParsedData[index][INDEX_0],dataParsed);
	}
	if(storeGPSParsedData[INDEX_13][INDEX_0]=='*')// if there is * at INDEX_13, add "NULL" instead
	{
		strcpy(&storeGPSParsedData[INDEX_14][INDEX_0],&storeGPSParsedData[INDEX_13][INDEX_0]);
		strcpy(&storeGPSParsedData[INDEX_13][INDEX_0],"NULL");
	}

}
/* FUNCTION      : convertParsedGPSDataToIntegerFloat
 * DESCRIPTION   : This function converts all numeric ASCII to integer type without decimal digits
 * 				   except for Altitude where it converts it to float type.
 * 				   All converted values will get converted back to string type.
 *
 * PARAMETERS    : GPSParsedData: array of data after it got parsed
 *
 * RETURNS       : NULL
*/

void convertParsedGPSDataToIntegerFloat(char GPSParsedData[PARSED_ROWS][PARSED_COLS])
{
	uint32_t GPSFloatAsciiToInteger;//variable to store float in ASCII to integer type
	float GPSFloatAsciiToFloat;//variable to store float in ASCII to float type
	uint8_t index=RESET;
	uint8_t oneZeroFlag=RESET;
	uint8_t twoZeroFlag=RESET;
	char GPSintegerToAscii[0][11];//temporary storage buffer

	for(index=0;index<15;index++)
	{
		if(index==INDEX_9)//Convert Altitude from ASCII to float and convert it back to ASCII
		{
			GPSFloatAsciiToFloat=atof(&GPSParsedData[index][INDEX_0]);
			sprintf(&GPSParsedData[index][INDEX_0],"%f",GPSFloatAsciiToFloat);
		}
		else
		{	// Convert all other numeric ASCII parts to integer
			if((GPSParsedData[index][INDEX_0]>=ASCII_0) && (GPSParsedData[index][INDEX_0]<=ASCII_9)) // extract ASCII numeric
			{
				//taking care of zeros as when ASCII numeric gets converted to integer it removes all zeros..check for zeros
				if((GPSParsedData[index][INDEX_0]==ASCII_0) && (GPSParsedData[index][1]!=ASCII_0))
				{
					oneZeroFlag=SET;
				}
				if((GPSParsedData[index][INDEX_0]==ASCII_0) && (GPSParsedData[index][1]==ASCII_0))
				{
					twoZeroFlag=SET;
				}

				GPSFloatAsciiToInteger=atoi(&GPSParsedData[index][INDEX_0]);// convert numeric ASCII to integer
				itoa(GPSFloatAsciiToInteger,&GPSintegerToAscii[INDEX_0][INDEX_0],BASE_10); // convert integer back to numeric ASCII

				// if there is one zero flag, add '0' at the beginning of the numeric value when it is converted back to string
				if(oneZeroFlag==SET)
				{
					GPSParsedData[index][0]='0';
					strcpy(&GPSParsedData[index][1],&GPSintegerToAscii[INDEX_0][INDEX_0]);
					oneZeroFlag=RESET;
				}
				// if there are two zero flags, add "00" at the beginning of the numeric value when it is converted back to string
				else if (twoZeroFlag==SET)
				{
					strcpy(&GPSParsedData[index][INDEX_0],"00");
					strcpy(&GPSParsedData[index][2],&GPSintegerToAscii[INDEX_0][INDEX_0]);
					twoZeroFlag=RESET;
				}
				else
				{
					strcpy(&GPSParsedData[index][INDEX_0],&GPSintegerToAscii[INDEX_0][INDEX_0]);
				}

			}
		}

	}
}

/* FUNCTION      : verifyCheckSum
 * DESCRIPTION   : This function verifies the Checksum of GPS data
 * 				   and it returns  unsigned integer of checked sum
 *
 * PARAMETERS    : GPSRawData: data starts with $ and ends with *checksum
 * 				   dataSet: select GPS raw data set to be checked
 * RETURNS       : verifiedChecksum
*/
uint8_t verifyCheckSum(char GPSRawData[RAW_DATA_ROWS][RAW_DATA_COLS],uint8_t dataSet)
{
	uint8_t verifiedChecksum=0;
	uint8_t i=0;
	if(GPSRawData[dataSet][0]=='$')// check the beginning of GPSDATA
	{	i++;
		while(GPSRawData[dataSet][i]!='*')// XOR all strings after $ sign until it reaches * sign
		{
			verifiedChecksum ^=GPSRawData[dataSet][i];
			i++;
		}
	}
	return verifiedChecksum;
}

/* FUNCTION      : GPSDataFormat
 * DESCRIPTION   : This function is used to format GPS data as follows:
 * 					x--> integer value
 * 				   __________________
 * 				   |sentenceID:_____|
 *				   |GPGGA___________|
 *			   	   |UTC Time:_______|
 *				   |hh:mm:ss.0______|
 *				   |Latitude:_______|
 *				   |dd deg mm.0000'N|
 *				   |Longitude:______|
 *			   	   |dddeg mm.00000'N|
 *			   	   |Fix quality:____|
 *				   |xx______________|
 *				   |# of Satellites:|
 *				   |xx______________|
 *				   |HDPO:___________|
 *			   	   |xx______________|
 *				   |Altitude:_______|
 *				   |xx.xxxxxM_______|
 *			   	   |Height of Geoid:|
 *				   |xxM_____________|
 *			   	   |DGPS Age:_______|
 *			   	   |xxs_____________|
 *				   |Checksum:_______|
 *			   	   |xx______________|
 *
 * 				***Note: This is an idea how it will be formated the number of x's might change***
 *
 * PARAMETERS    : GPSParsedData: array of data after it got parsed
 * 				   GPSParsedOutput: array of  formated data as shown in the description
 * RETURNS       : NULL
*/

void GPSDataFormat(char GPSParsedData[PARSED_ROWS][PARSED_COLS],char GPSParsedOutput[FORMATED_ROWS][FORMATED_COLS])
{
	uint8_t index=0;
	uint8_t dataIndex=RESET;
	uint8_t checkSumtoInteger=0;
	uint8_t DGPSSize=0;
	uint8_t altitudeSize=0;
	uint8_t geoidSize=0;
	char *unusedString;// used to store as a dumb pointer when strtoul function is used
	for(dataIndex=1 ; dataIndex<22 ; dataIndex+=2)
	{
		switch (dataIndex)
		{
			case DATA_INDEX_1: // add GPGGA part without $ sign
				strcpy(&GPSParsedData[INDEX_0][INDEX_0],&GPSParsedData[INDEX_0][INDEX_1]);//Remove $ sign from $GPGGA
				strcpy(&GPSParsedOutput[DATA_INDEX_1][INDEX_0],&GPSParsedData[INDEX_0][INDEX_0]);
				break;

			case DATA_INDEX_3:// Format time to hh:mm:ss.0
				while(index<=9)
				{
					if(index==INDEX_2)// :mm part
					{
						GPSParsedOutput[DATA_INDEX_3][INDEX_2]=':';
						GPSParsedOutput[DATA_INDEX_3][INDEX_3]=GPSParsedData[INDEX_1][INDEX_2];
						GPSParsedOutput[DATA_INDEX_3][INDEX_4]=GPSParsedData[INDEX_1][INDEX_3];
						index=INDEX_5;
					}
					else if(index==INDEX_5)//:ss part
					{
					GPSParsedOutput[INDEX_3][INDEX_5]=':';
					GPSParsedOutput[INDEX_3][INDEX_6]=GPSParsedData[INDEX_1][INDEX_4];
					GPSParsedOutput[INDEX_3][INDEX_7]=GPSParsedData[INDEX_1][INDEX_5];
					index=INDEX_8;
					}
					else if(index==INDEX_8)// . part
					{
						GPSParsedOutput[INDEX_3][INDEX_8]='.';
						index=INDEX_9;
					}
					else if(index==INDEX_9) // 0 part
					{
						GPSParsedOutput[INDEX_3][INDEX_9]='0';
						index=INDEX_10;
					}
					else
					{
						GPSParsedOutput[INDEX_3][index]=GPSParsedData[INDEX_1][index];// hh part
						index++;
					}

				}
				break;

			case DATA_INDEX_5:// Latitude format dd deg mm.0000'N or S
				GPSParsedOutput[INDEX_5][INDEX_0] = GPSParsedData[INDEX_2][INDEX_0];
				GPSParsedOutput[INDEX_5][INDEX_1] = GPSParsedData[INDEX_2][INDEX_1];
				strcpy(&GPSParsedOutput[INDEX_5][INDEX_2]," deg ");	//<----------------------------||
				GPSParsedOutput[INDEX_5][INDEX_7] = GPSParsedData[INDEX_2][INDEX_2];// Over writing \0 from previous string copy by starting at INDEX_7
				GPSParsedOutput[INDEX_5][INDEX_8] = GPSParsedData[INDEX_2][INDEX_3];
				GPSParsedOutput[INDEX_5][INDEX_9] = '.';
				strcpy(&GPSParsedOutput[INDEX_5][INDEX_10],"0000");
				GPSParsedOutput[INDEX_5][INDEX_14] = APASTROPHE;
				GPSParsedOutput[INDEX_5][INDEX_15] = GPSParsedData[INDEX_3][INDEX_0];
				break;

			case DATA_INDEX_7:// Longitude format ddddeg mm.0000'E or W
				GPSParsedOutput[DATA_INDEX_7][INDEX_0] = GPSParsedData[INDEX_4][INDEX_0];
				GPSParsedOutput[DATA_INDEX_7][INDEX_1] = GPSParsedData[INDEX_4][INDEX_1];
				GPSParsedOutput[DATA_INDEX_7][INDEX_2] = GPSParsedData[INDEX_4][INDEX_2];
				strcpy(&GPSParsedOutput[DATA_INDEX_7][INDEX_3],"deg ");
				GPSParsedOutput[DATA_INDEX_7][INDEX_7] = GPSParsedData[INDEX_4][INDEX_3];
				GPSParsedOutput[DATA_INDEX_7][INDEX_8] = GPSParsedData[INDEX_4][INDEX_4];
				GPSParsedOutput[DATA_INDEX_7][INDEX_9] = '.';
				strcpy(&GPSParsedOutput[DATA_INDEX_7][INDEX_10],"0000");
				GPSParsedOutput[DATA_INDEX_7][INDEX_14] = APASTROPHE;
				GPSParsedOutput[DATA_INDEX_7][INDEX_15] = GPSParsedData[INDEX_5][INDEX_0];
				break;

			case DATA_INDEX_9:// add Fix quality number
				GPSParsedOutput[DATA_INDEX_9][INDEX_0]=GPSParsedData[INDEX_6][INDEX_0];
				break;

			case DATA_INDEX_11://add number of Satellites Used
				GPSParsedOutput[DATA_INDEX_11][INDEX_0] = GPSParsedData[INDEX_7][INDEX_0];
				break;

			case DATA_INDEX_13://add Horizontal dilution of precision(HDOP) value
				GPSParsedOutput[DATA_INDEX_13][INDEX_0] = GPSParsedData[INDEX_8][INDEX_0];
				break;

			case DATA_INDEX_15://add Altitude format xxx.xxxxM
				strcpy(&GPSParsedOutput[DATA_INDEX_15][INDEX_0],&GPSParsedData[INDEX_9][INDEX_0]);
				altitudeSize=strlen(&GPSParsedData[INDEX_9][INDEX_0]);
				GPSParsedOutput[DATA_INDEX_15][altitudeSize-1]=GPSParsedData[INDEX_10][INDEX_0];
				break;

			case DATA_INDEX_17://add Geoid separation value followed by meters unit (M)
				strcpy(&GPSParsedOutput[DATA_INDEX_17][INDEX_0],&GPSParsedData[INDEX_11][0]);
				geoidSize=strlen(&GPSParsedData[INDEX_11][INDEX_0]);
				GPSParsedOutput[DATA_INDEX_17][geoidSize]=GPSParsedData[INDEX_12][0];
				break;

			case DATA_INDEX_19://add Age of DGPS data in seconds
				if(GPSParsedData[INDEX_13][INDEX_0]=='N')// Check for NULL
				{
					strcpy(&GPSParsedOutput[DATA_INDEX_19][INDEX_0],&GPSParsedData[INDEX_13][INDEX_0]);
				}
				else
				{
					strcpy(&GPSParsedOutput[DATA_INDEX_19][INDEX_0],&GPSParsedData[INDEX_13][INDEX_0]);
					DGPSSize=strlen(&GPSParsedData[INDEX_11][INDEX_0]);
					GPSParsedOutput[DATA_INDEX_19][DGPSSize-1]='s';// 's' second time unit for
				}
				break;

			case DATA_INDEX_21://add checksum value without * sign
			default:
				strcpy(&GPSParsedData[INDEX_14][INDEX_0],&GPSParsedData[INDEX_14][INDEX_1]);//Remove * sign from checksum
				checkSumtoInteger=strtoul(&GPSParsedData[INDEX_14][INDEX_0],&unusedString,BASE_16);
				itoa(checkSumtoInteger,&GPSParsedData[INDEX_14][INDEX_0],BASE_10);
				strcpy(&GPSParsedOutput[DATA_INDEX_21][INDEX_0],&GPSParsedData[INDEX_14][INDEX_0]);
				break;

		}
	}

}

/* USER CODE END 4 */
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
