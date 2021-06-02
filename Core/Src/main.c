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
#include "WM.h"

#ifdef GUI_SUPPORT
#include "FramewinDLG.h"
#include "DTFramewinDLG.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



#define C3_Pin GPIO_PIN_3
#define C3_GPIO_Port GPIOC

#define C3_STATE()	HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)

// ADC, RAM & Display timer
#define SDRAM_BANK_ADDR     ((uint32_t)0xD0000000)
#define WRITE_READ_ADDR     ((uint32_t)0x100000) //((uint32_t)0x0800)

#define ADC_AVG_SIZE	16

#define STORE_SECONDS	30
#define STORE_FREQUENCY	10000
#define SDRAM_BUF_SIZE	(STORE_FREQUENCY*STORE_SECONDS)

#define HIGH_CURRENT_ALERT_THRESHOLD	2000
#define LOW_CURRENT_ALERT_THRESHOLD		-2000

#define DISP_BUF_SIZE	50
#define DISP_TIMER_PERIOD	1000	//20s = 200px
//#define DISP_TIMER_PERIOD	100		//2s = 200px



#ifdef USB_BENCHMARK
#define USB_BUF_SIZE		200
#endif
#ifdef DMAADC
#define ADC_DMA_BUF_SIZE	16
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

IWDG_HandleTypeDef hiwdg;

LTDC_HandleTypeDef hltdc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

// Events
volatile uint8_t events = 0;

// LCD
TIM_HandleTypeDef htim3;
uint32_t uwPrescalerValue = 0;
uint8_t GUI_Initialized = 0;

// RTC
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

// USB
#ifdef USB_BENCHMARK
uint8_t usb_buf[USB_BUF_SIZE] = {0};
#endif
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef hUSB_Host; /* USB Host handle */

// ADC, RAM & Display Timer
#ifdef DMAADC
WORD adc_dma_buf[ADC_DMA_BUF_SIZE];
#endif

volatile uint32_t sdram_buf_index = 0;
const int CURRENT_MAX = 2496;
uint32_t adc_avg_sum = 0;
uint8_t	adc_avg_index = 0;
uint16_t disp_fifo_low_buf[DISP_BUF_SIZE] = {0};
uint16_t disp_fifo_high_buf[DISP_BUF_SIZE] = {0};
volatile uint16_t disp_buf_head = 0;
volatile uint16_t disp_buf_tail = 0;
uint16_t disp_low = 0xFFFF;
uint16_t disp_high = 0;
uint16_t disp_timer = 0;

// UART rx
volatile uint8_t cmd_line[21] = {0};
volatile uint8_t cmd_len = 0;
volatile uint16_t rx_len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA2D_Init(void);
static void MX_LTDC_Init(void);
static void MX_IWDG_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

void DMA2D_XferCpltCallback(DMA2D_HandleTypeDef *hdma2d);
extern void MainTaskFonts(void);
void BSP_Pointer_Update(void);
void BSP_Background(void);

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
	/* USER CODE END USB_Error_Handler */
}

#ifdef USB_BENCHMARK
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
							UINT bytesWritten;

							myprintf("Starting USB write benchmark...\r\n");

							DWORD start_time = HAL_GetTick();
							res = f_open(&MyFile, "usb_benchmark.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

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
								res = f_write(&MyFile, usb_buf, write_block_size, &bytesWritten);
								if(res == FR_OK) {
									//myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
								} else {
									USB_Error_Handler(res);
								}
							}

							f_close(&MyFile);

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
#endif //USB_BENCHMARK

FRESULT CheckFileWrite(void) {
	FRESULT res;
	UINT byteswritten;
	char test_text[] = "Test write into the file.";
	const char filename[] = "_test_write.TXT";

	// USB_Benchmark_Application();

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
		if((res = f_open(&MyFile, filename, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
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
				myprintf("ok\r\nClosing and delete... ");
				f_close(&MyFile);
				if((res = f_unlink(filename)) != FR_OK)
				{
					USB_Error_Handler(res);
				}
				else
				{
					myprintf("ok\r\n");
				}
			}
		}
	}

	myprintf("Unlink drive... ");
	/* Unlink the USB disk I/O driver */
	FATFS_UnLinkDriver(USBDISKPath);
	myprintf("done\r\n");
	return res;
}

void USB_OnAttach(void) {
	events |= EVT_USB_ATTACH;
}

void USB_OnDetach(void) {
	events |= EVT_USB_DETACH;
}

void USB_Connecting(void) {
	events |= EVT_USB_CONNECTING;
}


int16_t GetCurrent(uint16_t adc_value) {
	return (((int)adc_value-2048) * CURRENT_MAX)/2048;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
  UINT adcRawValue = HAL_ADC_GetValue(&hadc1);
  LD4_TOGGLE();

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


//	  long adc_real_voltage = GetCurrent(adc_raw_avg_value);
//	  if(adc_real_voltage > HIGH_CURRENT_ALERT_THRESHOLD || adc_real_voltage < LOW_CURRENT_ALERT_THRESHOLD) {
//		  // TODO: stop conversions until data will be stored in file
//		  // ...
//
//		  HAL_TIM_Base_Stop(&htim2);
//		  events |= EVT_VOLTAGE_FAIL;
//	  }
  }

}

void StoreData(void) {
	if(events&EVT_USB_ATTACHED) {
		FRESULT res;                                          /* FatFs function common result code */
#ifdef GUI_SUPPORT
		UpdateUsbStatusText("Writing...", GUI_BLUE);
#endif
		myprintf("Mounting USB... ");
		/* Register the file system object to the FatFs module */
		if((res=f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0)) != FR_OK)
		{
			/* FatFs Initialization Error */
			USB_Error_Handler(res);
		}
		else
		{
			UINT bytesWritten;
			UINT total_written = 0;

			char filename[20];

			sprintf(filename,"20%02d%02d%02d_%02d%02d%02d.csv",
					sDate.Year, sDate.Month, sDate.Date,
					sTime.Hours, sTime.Minutes, sTime.Seconds);

			myprintf("ok\r\nOpening file '%s' for writing... ", filename);
			DWORD start_time = HAL_GetTick();
			res = f_open(&MyFile, filename, FA_WRITE | FA_CREATE_ALWAYS);

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
			char line[30];

			for(int i = 0; i < SDRAM_BUF_SIZE; i++) {
				if((i%1000) == 0) {
					char progress_str[60];

					HAL_IWDG_Refresh(&hiwdg);
					uint32_t percentage = (i*100)/SDRAM_BUF_SIZE;
					sprintf(progress_str, "Writing... [______________________________] %ld%% \r", percentage);

					int j, max = 12+(i/10000);
					for(j = 12; j < max; j++) {
						progress_str[j] = '#';
					}
					int iii = (i/1000)%10;
					progress_str[j] = ".:|/-\\|/-\\|"[iii];
					myprintf(progress_str);
#ifdef GUI_SUPPORT
					UpdateProgressBar(percentage);
					DialogProcess();
#endif
				}
				uint32_t address_offset = sdram_buf_index + i;
				if(address_offset >= SDRAM_BUF_SIZE) address_offset -= SDRAM_BUF_SIZE;
				uint16_t adc_value = *(__IO uint16_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + sizeof(uint16_t)*address_offset);
				int16_t voltage = GetCurrent(adc_value);
				sprintf(line, "%02d:%02d:%02d.%04d\t%d\r\n", hour, minute, second, fraction, voltage);
				uint32_t line_size = strlen(line);

				res = f_write(&MyFile, line, line_size, &bytesWritten);
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
#ifdef GUI_SUPPORT
			UpdateProgressBar(100);
#endif

			f_close(&MyFile);

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
#ifdef GUI_SUPPORT
		HAL_Delay(300);
		UpdateProgressBar(0);
		UpdateUsbStatusText("Ready", GUI_BLACK);
#endif
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
	return ((DWORD)(2000 + sDate.Year - 1900) << 25)
	        | ((DWORD)sDate.Month << 21)
	        | ((DWORD)sDate.Date << 16)
	        | ((DWORD)sTime.Hours << 11)
	        | ((DWORD)sTime.Minutes << 5)
	        | ((DWORD)sTime.Seconds >> 1);
}

const uint8_t skip_prints = 0;//10;
uint8_t skip_prints_cnt = 0;
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

		int16_t low_current = GetCurrent(low);
		int16_t high_current = GetCurrent(high);

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		if( skip_prints_cnt++ == skip_prints ) {
			myprintf("[20%02d-%02d-%02d %02d:%02d:%02d.%03d] Vmin=%dV, Vmax=%dV        \r",
					sDate.Year, sDate.Month, sDate.Date,
					sTime.Hours, sTime.Minutes, sTime.Seconds, (uint32_t)(sTime.SubSeconds/1.024),
					low_current, high_current);
			skip_prints_cnt = 0;
		}
#ifdef GUI_SUPPORT
		UpdateDateTimeEdit(sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);
		AddGraphData(low_current, high_current);
		if( low_current <= 0 && high_current >= 0 ) {
			if( high_current >= (-low_current) ) {
				UpdateCurrentEdit(high_current);
			} else {
				UpdateCurrentEdit(low_current);
			}
		} else if( high_current > 0 ) {
			UpdateCurrentEdit(high_current);
		} else {
			UpdateCurrentEdit(low_current);
		}
#endif
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

#ifdef GUI_SUPPORT
/**
  * @brief  Initializes the STM32F429I-DISCO's LCD and LEDs resources.
  * @param  None
  * @retval None
  */
static void BSP_Config(void)
{
  /* Initialize STM32F429I-DISCO's LEDs */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  /* Initializes the SDRAM device */
  BSP_SDRAM_Init();

  /* Initialize the Touch screen */
  BSP_TS_Init(240, 320);

  /* Enable the CRC Module */
  __HAL_RCC_CRC_CLK_ENABLE();
}
#endif

void TIM3_Init(void) {
	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM3_Init 1 */

	  /* USER CODE END TIM3_Init 1 */
	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 1000-1;
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 8400-1;
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
	  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if( huart->Instance == USART1 ) {
		events |= EVT_UART_RX;
		//cmd_line[rx_len++] = (uint8_t)(huart->Instance->DR & (uint8_t)0xFF);
	}
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
  MX_FMC_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_FATFS_Init();
  MX_I2C3_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  MX_DMA2D_Init();
  MX_LTDC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  	myprintf("\r\n\r\n\r\nStarting...");

	#ifdef GUI_SUPPORT
  	  	/* Initialize LCD and LEDs */
		BSP_Config();
		/* Init the STemWin GUI Library */
		GUI_Init();
		WM_MULTIBUF_Enable(1);
		/* Activate the use of memory device feature */
		WM_SetCreateFlags(WM_CF_MEMDEV);
		GUI_SetBkColor(GUI_BLACK);
		GUI_Clear();
		int result = GUI_SetOrientation(GUI_ROTATION_180);
		CreateDialog();
#ifdef TOUCH
		TIM3_Init();
#endif
		GUI_Initialized = 1;
	#endif

	//RTC_InitTime();
	LD3_ON();
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_IT(&hadc1);

	//HAL_TIM_Base_Start_IT(&htim2);
	//HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	myprintf("ok\r\n");

	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg);
		PrintADCValues();
//		if(events&EVT_VOLTAGE_FAIL) {
//			events &= ~EVT_VOLTAGE_FAIL;
		if (B1_STATE() == GPIO_PIN_SET || REC_START_STATE() == GPIO_PIN_RESET ) {
			HAL_TIM_Base_Stop(&htim2);

			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			myprintf("\r\nTime to store voltage failure...\r\n");
			StoreData();
			//HAL_Delay(2000);
			HAL_TIM_Base_Start(&htim2);
		}
		//BSP_Background();
		//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin); // red led toggle
#ifdef GUI_SUPPORT
		DialogProcess();
#endif

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

		if(events&EVT_VOLTAGE_FAIL){
			events &= ~EVT_VOLTAGE_FAIL;
		}

		if(events&EVT_USB_ATTACH){
			events &= ~EVT_USB_ATTACH;

			myprintf("\r\nUSB stick attached.\r\nCheck FS for access...\r\n");
			#ifdef GUI_SUPPORT
				UpdateUsbStatusText("Attaching...", GUI_ORANGE);
			#endif

			FRESULT res = CheckFileWrite();

			if(res == FR_OK) {
				events |= EVT_USB_ATTACHED;
				myprintf("FS access granted successfully.\r\n");
				#ifdef GUI_SUPPORT
					UpdateUsbStatusText("Ready", GUI_BLACK);
				#endif
			} else {
				myprintf("ERROR! FS access not granted!\r\n");
				#ifdef GUI_SUPPORT
					UpdateUsbStatusText("Error", GUI_RED);
				#endif
			}
			//HAL_Delay(2000);
		}

		if(events&EVT_USB_CONNECTING){
			events &= ~EVT_USB_CONNECTING;
			myprintf("\r\nConnecting USB device...\r\n");
			#ifdef GUI_SUPPORT
				UpdateUsbStatusText("Connecting...", GUI_ORANGE);
			#endif
		}

		if(events&EVT_USB_DETACH){
			events &= ~EVT_USB_DETACH;
			events &= ~EVT_USB_ATTACHED;
			myprintf("\r\nUSB stick detached.\r\n");
			#ifdef GUI_SUPPORT
				UpdateUsbStatusText("Not connected", GUI_RED);
			#endif
		}

		{
#ifdef GUI_SUPPORT
			if( events&EVT_DATE_TIME_DLG ) {

				CreateDTDialog();

				while(events&EVT_DATE_TIME_DLG) {
					HAL_IWDG_Refresh(&hiwdg);
					DTDialogProcess();
				}

				if( events&EVT_DATE_TIME_SETUP ) {
					events &= ~EVT_DATE_TIME_SETUP;
					HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
					HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				}

				DTDialogClose();
			}
#endif
			//if( HAL_UARTEx_ReceiveToIdle(&huart1, cmd_line, 1, &rx_len, 1) == HAL_OK && rx_len > 0 && (cmd_line[0] == '~' || cmd_line[0] == '`') )
			if( events&EVT_UART_RX )
			{
				events &= ~EVT_UART_RX;
				if( cmd_line[0] == '~' || cmd_line[0] == '`' )
				{
					myprintf("\r\n");
					myprintf("Command mode activated\r\n");
					HAL_Delay(2000);


					while(1)
					{
						HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
						myprintf("Current date & time: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
											sDate.Year, sDate.Month, sDate.Date,
											sTime.Hours, sTime.Minutes, sTime.Seconds);
						myprintf("Available commands:\r\n");
						myprintf("\tY=xx - set year (0...99)\r\n");
						myprintf("\tM=xx - set month (1-Jan, ..., 12-Dec)\r\n");
						myprintf("\tD=xx - set date (1...31)\r\n");
						myprintf("\th=xx - set hours (0...23)\r\n");
						myprintf("\tm=xx - set minutes (0...59)\r\n");
						myprintf("\ts=xx - set seconds (0...59)\r\n");
						myprintf("\texit - Exit from command mode\r\n");
						myprintf("CMD> ");
						cmd_len = 0;
						memset( cmd_line, 0, 21 );
						uint8_t ch = 0;
						while( ch != 13 ) {
							HAL_IWDG_Refresh(&hiwdg);
							//if( HAL_UART_ReceiveToIdle(&huart1, &ch, 1, &rx_len, 1) == HAL_OK && rx_len > 0 )
							if( HAL_UART_Receive(&huart1, &ch, 1, 50) == HAL_OK )
							{
								if( ch == 127 && cmd_len > 0 ) {
									HAL_UART_Transmit(&huart1, &ch, 1, 1);
									cmd_len--;
								}
								else if ( ch == 13 ) // enter
								{
									cmd_line[cmd_len] = 0;
								}
								else if ( (ch >= 'a' && ch <= 'z') ||
										  (ch >= 'A' && ch <= 'Z') ||
										  (ch >= '0' && ch <= '9') ||
										  ch == '=' )
								{
									HAL_UART_Transmit(&huart1, &ch, 1, 1);
									cmd_line[cmd_len++] = ch;
								}
							}
						}
						myprintf("\r\n");
						if( strcmp((char*)cmd_line,"exit") == 0 )
						{
							break;
						}
						if( cmd_line[1] == '=' && cmd_len > 2 )
						{
							HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
							HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
							switch( cmd_line[0] )
							{
							case 'Y':
								{
									int year = 2021;
									sscanf( (char*)&cmd_line[2], "%d", &year );
									if( year >= 2000 ) year -= 2000;
									if( year > 99 )
									{
										myprintf("Command error\r\n");
										break;
									}
									sDate.Year = year;
									HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
								}
								break;
							case 'M':
							{
								int month = 1;
								sscanf( (char*)&cmd_line[2], "%d", &month );
								if( month == 0 || month > 12 )
								{
									myprintf("Command error\r\n");
									break;
								}
								sDate.Month = month;
								HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
							}
								break;
							case 'D':
							{
								int date = 1;
								sscanf( (char*)&cmd_line[2], "%d", &date );
								if( date == 0 || date > 31 )
								{
									myprintf("Command error\r\n");
									break;
								}
								sDate.Date = date;
								HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
							}
								break;
							case 'h':
							{
								int hour = 0;
								sscanf( (char*)&cmd_line[2], "%d", &hour );
								if( hour > 23 )
								{
									myprintf("Command error\r\n");
									break;
								}
								sTime.Hours = hour;
								HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
							}
								break;
							case 'm':
							{
								int minute = 0;
								sscanf( (char*)&cmd_line[2], "%d", &minute );
								if( minute > 59 )
								{
									myprintf("Command error\r\n");
									break;
								}
								sTime.Minutes = minute;
								HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
							}
								break;
							case 's':
							{
								int second = 0;
								sscanf( (char*)&cmd_line[2], "%d", &second );
								if( second > 59 )
								{
									myprintf("Command error\r\n");
									break;
								}
								sTime.Seconds = second;
								HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
							}
								break;
							default:
								myprintf("Command error\r\n");
							}
						}
						else
						{
							myprintf("Command error\r\n");
						}
					}
				}
				else
				{
					if( rx_len )
					{
						myprintf("\r\nUnknown command key pressed (code=0x%02X)\r\n", cmd_line[0]);
					}
				}
				rx_len = 0;
			}

			HAL_UART_Receive_IT(&huart1, cmd_line, 1);

		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */
  hdma2d.XferCpltCallback = DMA2D_XferCpltCallback;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 29;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
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
  hrtc.Init.SynchPrediv = 1023;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if(0) {
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

  }
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

  /*Configure GPIO pin : REC_START_Pin */
  GPIO_InitStruct.Pin = REC_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(REC_START_GPIO_Port, &GPIO_InitStruct);

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

/**
  * @brief  BSP_Background.
  * @param  None
  * @retval None
  */
void BSP_Background(void)
{
//  BSP_LED_Toggle(LED3);
//  BSP_LED_Toggle(LED4);

  /* Capture input event and update cursor */
  if(GUI_Initialized == 1)
  {
    BSP_Pointer_Update();
  }
}

/**
  * @brief  Provide the GUI with current state of the touch screen
  * @param  None
  * @retval None
  */
void BSP_Pointer_Update(void)
{
  GUI_PID_STATE TS_State;
  static TS_StateTypeDef prev_state;
  TS_StateTypeDef  ts;
  uint16_t xDiff, yDiff;

  BSP_TS_GetState(&ts);

  TS_State.Pressed = ts.TouchDetected;

  xDiff = (prev_state.X > ts.X) ? (prev_state.X - ts.X) : (ts.X - prev_state.X);
  yDiff = (prev_state.Y > ts.Y) ? (prev_state.Y - ts.Y) : (ts.Y - prev_state.Y);

  if((prev_state.TouchDetected != ts.TouchDetected )||
     (xDiff > 3 )||
       (yDiff > 3))
  {
    prev_state.TouchDetected = ts.TouchDetected;

    if((ts.X != 0) &&  (ts.Y != 0))
    {
      prev_state.X= ts.X;
      prev_state.Y= ts.Y;
    }

      TS_State.Layer = 0;
      TS_State.x = prev_state.X;
      TS_State.y = prev_state.Y;

    GUI_TOUCH_StoreStateEx(&TS_State);
  }
}
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

#ifdef TOUCH
	if (htim->Instance == TIM3) {
		BSP_Background();
//		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin); // red led toggle
	}
#endif
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
