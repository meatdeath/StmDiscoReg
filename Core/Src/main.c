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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define USB_BUF_SIZE		200
#define ADC_DMA_BUF_SIZE	16
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
//----------------------------------------------------- RTC ----------------------
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

// --------------------------------------------------- USB vars ------------------------
uint8_t usb_buf[USB_BUF_SIZE] = {0};
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef hUSB_Host; /* USB Host handle */
WORD adc_dma_buf[ADC_DMA_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, -1);

}

void USB_Error_Handler(FRESULT error_code)
{
  /* USER CODE BEGIN USB_Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
	myprintf("ERROR (code=%d)\r\n", error_code);
	HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,GPIO_PIN_SET);
  /* USER CODE END USB_Error_Handler */
}

void USB_Benchmark_Application(void)
{
	FRESULT res;                                          /* FatFs function common result code */
	uint32_t byteswritten, bytesread;                     /* File write/read counts */
	uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
	uint8_t rtext[100];                                   /* File read buffer */

	HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin,GPIO_PIN_RESET);

	myprintf("Mounting USB... ");
	/* Register the file system object to the FatFs module */
	if((res=f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0)) != FR_OK)
	{
		/* FatFs Initialization Error */
		USB_Error_Handler(res);
	}
	else
	{
		myprintf("ok\r\nOpening file for writing... ");
		if((res = f_open(&MyFile, "Even.TXT", FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
		{
			USB_Error_Handler(res);
		}
		else
		{
			myprintf("ok\r\nWriting file... ");
			res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
			if((byteswritten == 0) || (res != FR_OK))
			{
				USB_Error_Handler(res);
			}
			else
			{
				myprintf("ok\r\nClosing file... ");
				f_close(&MyFile);

				myprintf("ok\r\nOpening file for reading... ");
				if(f_open(&MyFile, "Even.TXT", FA_READ) != FR_OK)
				{
					USB_Error_Handler(res);
				}
				else
				{

					myprintf("ok\r\nReading file... ");
					res = f_read(&MyFile, rtext, sizeof(rtext), (void *)&bytesread);

					if((bytesread == 0) || (res != FR_OK))
					{
						USB_Error_Handler(res);
					}
					else
					{
						myprintf("ok\r\nClosing file.");
						f_close(&MyFile);

						if((bytesread != byteswritten))
						{
							USB_Error_Handler(res);
						}
						else
						{
							FIL fil; 		//File handle
							UINT bytesWritten;

							myprintf("Starting USB write benchmark...\r\n");

							DWORD start_time = HAL_GetTick();
							res = f_open(&fil, "usb_benchmark.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

							if(res == FR_OK) {
								myprintf("I was able to open 'usb_benchmark.txt' for writing\r\n");
							} else {
								USB_Error_Handler(res);
							}

							const uint32_t write_block_size = USB_BUF_SIZE;
							const uint32_t line_size = 12;
							const uint32_t record_sec = 30;
							const uint32_t store_data_size = (line_size*record_sec*10000);

							for( uint32_t i = 0; i < store_data_size; i += write_block_size ) {
								memset(usb_buf,i/write_block_size,write_block_size);
								res = f_write(&fil, usb_buf, write_block_size, &bytesWritten);
								if(res == FR_OK) {
									//myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
								} else {
									USB_Error_Handler(res);
								}
							}

							f_close(&fil);

							DWORD test_duration = HAL_GetTick() - start_time;
							myprintf("Benchmark time: %ul.%uls\r\nWrite speed: %ulKB/s\r\n",
									test_duration/10000,
									test_duration%10000,
									10000*store_data_size/1024/test_duration);
						}
					}
				}
			}
		}
	}

	myprintf("Unlink drive... ");
	/* Unlink the USB disk I/O driver */
	FATFS_UnLinkDriver(USBDISKPath);
	myprintf("done\r\n");
}



#define EVT_VOLTAGE_FAIL	(1<<0)
#define EVT_USB_ATTACH		(1<<1)
#define EVT_USB_ATTACHED	(1<<2)
#define EVT_USB_DETACH		(1<<3)

volatile uint8_t events = 0;

void USB_OnAttach(void) {
	FRESULT res;
	char test_text[] = "Test write into the file.";
	UINT byteswritten;
	// USB_Benchmark_Application();
	events |= EVT_USB_ATTACH;
	myprintf("\r\nUSB stick attached.\r\nCheck FS for access...\r\n");
	myprintf("Mounting USB... ");
	/* Register the file system object to the FatFs module */
	if((res=f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0)) != FR_OK)
	{
		/* FatFs Initialization Error */
		USB_Error_Handler(res);
	}
	else
	{
		myprintf("ok\r\nOpening file for writing... ");
		if((res = f_open(&MyFile, "_test_write.TXT", FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
		{
			USB_Error_Handler(res);
		}
		else
		{
			myprintf("ok\r\nWriting file... ");
			res = f_write(&MyFile, test_text, sizeof(test_text), (void *)&byteswritten);
			if((byteswritten == 0) || (res != FR_OK))
			{
				USB_Error_Handler(res);
			}
			else
			{
				myprintf("ok\r\nClosing file. ");
				f_close(&MyFile);
			}
		}
	}

	myprintf("Unlink drive... ");
	/* Unlink the USB disk I/O driver */
	FATFS_UnLinkDriver(USBDISKPath);
	myprintf("done\r\n");

	if(res == FR_OK) {
		events |= EVT_USB_ATTACHED;
		myprintf("FS access granted successfully.\r\n");
	} else {
		myprintf("ERROR! FS access not granted!\r\n");
	}
	HAL_Delay(2000);
}

void USB_OnDetach(void) {
	events |= EVT_USB_DETACH;
	events &= ~EVT_USB_ATTACHED;
	myprintf("\r\nUSB stick detached.\r\n");
}

#define ADC_AVG_SIZE	16
uint32_t adc_avg_sum = 0;
uint8_t	adc_avg_index = 0;

#define STORE_SECONDS	30
#define STORE_FREQUENCY	10000
#define SDRAM_BUF_SIZE	(STORE_FREQUENCY*STORE_SECONDS)
volatile uint32_t sdram_buf_index = 0;

#define SDRAM_BANK_ADDR     ((uint32_t)0xD0000000)
#define WRITE_READ_ADDR     ((uint32_t)0x100000) //((uint32_t)0x0800)
#define REFRESH_COUNT       ((uint32_t)0x056A)   /* SDRAM refresh counter (90MHz SDRAM clock) */

const int v_max = 600;

#define HIGH_VOLTAGE_ALERT_THRESHOLD	500
#define LOW_VOLTAGE_ALERT_THRESHOLD		-500

#define DISP_BUF_SIZE	50
uint16_t disp_fifo_low_buf[DISP_BUF_SIZE] = {0};
uint16_t disp_fifo_high_buf[DISP_BUF_SIZE] = {0};
volatile uint16_t disp_buf_head = 0;
volatile uint16_t disp_buf_tail = 0;
uint16_t disp_low = 0xFFFF;
uint16_t disp_high = 0;
#define DISP_TIMER_PERIOD	1000
uint16_t disp_timer = 0;


int16_t GetVoltage(uint16_t adc_value) {
	int16_t value = (((int)adc_value-2048) * v_max)/2048;
	return value;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
  UINT adcRawValue = HAL_ADC_GetValue(&hadc1);
  //LD3_TOGGLE();

  adc_avg_sum += adcRawValue;
  adc_avg_index++;

  if(adc_avg_index == ADC_AVG_SIZE) {
	  uint16_t adc_raw_avg_value = (uint16_t)(adc_avg_sum/ADC_AVG_SIZE);
	  *(__IO uint16_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + sizeof(uint16_t)*sdram_buf_index) = adc_raw_avg_value;
	  sdram_buf_index++;
	  if(sdram_buf_index == SDRAM_BUF_SIZE) {
		  sdram_buf_index = 0;
	  }

	  adc_avg_sum = 0;
	  adc_avg_index = 0;

	  if(adc_raw_avg_value < disp_low ) disp_low = adc_raw_avg_value;
	  if(adc_raw_avg_value > disp_high) disp_high = adc_raw_avg_value;

	  // Each 100ms the voltage value should be shown on screen
	  disp_timer++;
	  if(disp_timer == DISP_TIMER_PERIOD) {

		  disp_fifo_low_buf[disp_buf_tail] = disp_low;
		  disp_fifo_high_buf[disp_buf_tail] = disp_high;

		  disp_buf_tail++;
		  if(disp_buf_tail == DISP_BUF_SIZE) {
			  disp_buf_tail = 0;
		  }

		  // skip old values if fifo overflow detected
		  if(disp_buf_tail == disp_buf_head) {
			  disp_buf_head++;
			  if(disp_buf_head == DISP_BUF_SIZE) {
				  disp_buf_head = 0;
			  }
		  }

		  disp_low = 0xFFFF;
		  disp_high = 0;
		  disp_timer = 0;
	  }


	  long adc_real_voltage = GetVoltage(adc_raw_avg_value);
	  if(adc_real_voltage > HIGH_VOLTAGE_ALERT_THRESHOLD || adc_real_voltage < LOW_VOLTAGE_ALERT_THRESHOLD) {
		  // TODO: stop conversions until data will be stored in file
		  // ...

		  HAL_TIM_Base_Stop(&htim2);
		  events |= EVT_VOLTAGE_FAIL;
	  }
  }

}

void StoreData(void) {
	if(events&EVT_USB_ATTACHED) {
		FRESULT res;                                          /* FatFs function common result code */

		myprintf("Mounting USB... ");
		/* Register the file system object to the FatFs module */
		if((res=f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0)) != FR_OK)
		{
			/* FatFs Initialization Error */
			USB_Error_Handler(res);
		}
		else
		{
			FIL fil; 		//File handle
			UINT bytesWritten;
			UINT total_written = 0;

			char filename[20];

			sprintf(filename,"20%02d%02d%02d_%02d%02d%02d.csv",
					sDate.Year, sDate.Month, sDate.Date,
					sTime.Hours, sTime.Minutes, sTime.Seconds);

			myprintf("ok\r\nOpening file '%s' for writing... ", filename);
			DWORD start_time = HAL_GetTick();
			res = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

			if(res == FR_OK) {
				myprintf("ok\r\n");
			} else {
				USB_Error_Handler(res);
			}

			int hour = sTime.Hours;
			int minute = sTime.Minutes;
			int second = sTime.Seconds-30;
			if(second < 0) {
				second += 60;
				if(minute == 0) {
					minute = 60;
					if(hour == 0) {
						hour = 24;
					}
					hour--;
				}
				minute--;
			}
			int fraction = 0;

			for(int i = 0; i < SDRAM_BUF_SIZE; i++) {
				if((i%1000) == 0) {
					char progress_str[60];
					sprintf(progress_str, "Writing... [------------------------------] %d%%\r", (i*100)/SDRAM_BUF_SIZE);

					int j, max = 12+(i/10000);
					for(j = 12; j < max; j++) {
						progress_str[j] = '#';
					}
					int iii = (i/1000)%10;
					progress_str[j] = "-\\|/-\\|/--"[iii];
					myprintf(progress_str);
				}
				char line[30];
				uint32_t address_offset = sdram_buf_index + i;
				if(address_offset >= SDRAM_BUF_SIZE) address_offset -= SDRAM_BUF_SIZE;
				uint16_t adc_value = *(__IO uint16_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + sizeof(uint16_t)*address_offset);
				int16_t voltage = GetVoltage(adc_value);
				sprintf(line, "%02d:%02d:%02d.%03d\t%d\r\n", hour, minute, second, fraction, voltage);
				uint32_t line_size = strlen(line);

				res = f_write(&fil, line, line_size, &bytesWritten);
				if(res == FR_OK) {
					total_written += bytesWritten;
				} else {
					USB_Error_Handler(res);
				}
				fraction++;
				if(fraction == 10000) {
					fraction = 0;
					second++;
					if(second == 60) {
						second = 0;
						minute++;
						if(minute == 60) {
							minute = 0;
							hour++;
							if(hour == 24) {
								hour = 0;
							}
						}
					}
				}
			}
			myprintf("Writing... [##############################] 100%%\r\n");

			f_close(&fil);

			DWORD test_duration = HAL_GetTick() - start_time;
			char report_str[50];
			sprintf(report_str, "Store time: %lu.%lus\r\nWrite speed: %luKB/s\r\n",
					test_duration/10000,
					test_duration%10000,
					total_written/1024/test_duration);
			myprintf(report_str);
		}

		myprintf("Unlink drive... ");
		/* Unlink the USB disk I/O driver */
		FATFS_UnLinkDriver(USBDISKPath);
		myprintf("done\r\n");
	}
}

void RTC_InitTime(void) {
	sTime.Hours = 17;
	sTime.Minutes = 30;
	sTime.Seconds = 0;
	sDate.Date = 22;
	sDate.Month = RTC_MONTH_APRIL;
	sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
	sDate.Year = 21;
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

DWORD GetTimeFromRTC(void) {
	return ((DWORD)(2000 + sDate.Year - 1900) << 25)  // Year 2014
	        | ((DWORD)sDate.Month << 21)            // Month 7
	        | ((DWORD)sDate.Date << 16)           // Mday 10
	        | ((DWORD)sTime.Hours << 11)           // Hour 16
	        | ((DWORD)sTime.Minutes << 5)             // Min 0
	        | ((DWORD)sTime.Seconds >> 1);            // Sec 0
}

void PrintADCValues(void) {
	if(disp_buf_head != disp_buf_tail) {

		/* Disable interrupts */
		__disable_irq();

		uint16_t low  = disp_fifo_low_buf[disp_buf_head];
		uint16_t high = disp_fifo_high_buf[disp_buf_head];
		disp_buf_head++;
		if(disp_buf_head == DISP_BUF_SIZE) {
			disp_buf_head = 0;
		}

		/* Enable interrupts back */
		__enable_irq();

		int16_t low_voltage = GetVoltage(low);
		int16_t high_voltage = GetVoltage(high);

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		myprintf("[20%02d-%02d-%02d %02d:%02d:%02d] Vmin=%dV, Vmax=%dV        \r",
				sDate.Year, sDate.Month, sDate.Date,
				sTime.Hours, sTime.Minutes, sTime.Seconds,
				low_voltage, high_voltage);
	}
}

void DMA_Adc2MemTransferComplete(DMA_HandleTypeDef *hdma) {
	//HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
}



#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
FMC_SDRAM_CommandTypeDef command;

/**
  * @brief  Perform the SDRAM exernal memory inialization sequence
  * @param  hsdram: SDRAM handle
  * @param  Command: Pointer to SDRAM command structure
  * @retval None
  */
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
  __IO uint32_t tmpmrd =0;
  /* Step 3:  Configure a clock configuration enable command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 4: Insert 100 ms delay */
  HAL_Delay(100);

  /* Step 5: Configure a PALL (precharge all) command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget 	     = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 6 : Configure a Auto-Refresh command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 4;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 7: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_3           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT);
}

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
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_FATFS_Init();
  MX_I2C3_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  RTC_InitTime();
  LD3_ON();
  //HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim2);
  //HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		PrintADCValues();
		if(events&EVT_VOLTAGE_FAIL) {
			events &= ~EVT_VOLTAGE_FAIL;

			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			myprintf("\r\nTime to store voltage failure...\r\n");
			StoreData();
			HAL_Delay(2000);
			HAL_TIM_Base_Start(&htim2);
		}
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 432;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV25;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 334;
  hltdc.Init.AccumulatedActiveH = 245;
  hltdc.Init.TotalWidth = 340;
  hltdc.Init.TotalHeigh = 247;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 31;
  hrtc.Init.SynchPrediv = 9999;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 0x1;
  sDate.Year = 0x21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 524;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_3;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  SDRAM_Initialization_Sequence(&hsdram1, &command);
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM2) {
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin); // red led toggle
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
