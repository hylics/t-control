/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 22/06/2015 11:08:53
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "crc.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "main.h"
//#include "assert.h"
//#define USE_FULL_ASSERT 1
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//__IO uint8_t dma_t_cplt=1, dma_r_cplt=1;
extern AD7792_HandleTypeDef adi1;
extern SavedDomain_t EepromDomain;
SavedDomain_t Options_rw;
TIM_OC_InitTypeDef sConfigPWM;
HAL_StatusTypeDef sts;
HD44780 lcd;
HD44780_STM32F0xx_GPIO_Driver lcd_pindriver;
volatile uint32_t systick_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
void init_lcd(void);
void delay_lcd(uint16_t ms);
uint32_t uint32_time_diff(uint32_t now, uint32_t before);
void hd44780_assert_failure_handler(const char *filename, unsigned long line);
static HAL_StatusTypeDef out_pwm_jitter(float32_t pwr) __attribute__((used));
static HAL_StatusTypeDef out_pwm_simple(float32_t pwr) __attribute__((used));
static HAL_StatusTypeDef out_pwm_bresenham(float32_t pwr) __attribute__((used));
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef (*pf_output[N_FUNC_PWR])(float32_t pwr) = {out_pwm_jitter, out_pwm_simple, out_pwm_bresenham};
static uint16_t counter = 0; //counter for out_pwm_jitter
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	counter++;
}

HAL_StatusTypeDef out_pwm_jitter(float32_t pwr) {
	//use mod
	// tc1 6s, 6000 ms, 600 halfcycles 10ms cycle
	// tc2 60s, 10*tc1
	//uint16_t tmp = (uint16_t)(pwr / Options_rw.pwm_scale_f);
	
	//uint16_t tmp = (uint16_t)pwr;
	
	
	uint16_t pwm_val;
	
	if(counter == 0) {
		pwm_val = (uint16_t)pwr + 10*((uint16_t)pwr%10);
	}
	else {
		pwm_val = (uint16_t)pwr;
	}
	if(counter > 9) {
		counter = 0;
	}
	
	sConfigPWM.Pulse = pwm_val;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigPWM, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	
	return HAL_OK;
}
HAL_StatusTypeDef out_pwm_simple(float32_t pwr){
	return HAL_OK;
}

HAL_StatusTypeDef out_pwm_bresenham(float32_t pwr) {
	return HAL_OK;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	/*float32_t  cosOutput, sinOutput, sincos;  
	float32_t x=0.941943896490312180;
	cosOutput = arm_cos_f32(x);
	sinOutput = arm_sin_f32(x);
	sincos=sinOutput+cosOutput;*/
	
	/** Rx=((A21+A22)/65536)*Re+(RL2-RL1)
	*  switch Iout1 and Iout2
	*/
	
	__IO uint32_t conf[5] = {0};
	sConfigPWM.OCMode = TIM_OCMODE_PWM1;
  sConfigPWM.Pulse = 0;
  sConfigPWM.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigPWM.OCFastMode = TIM_OCFAST_DISABLE;
	
	Options_rw = EepromDomain;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	//keep commented while default config changed
	//uint32_t temp_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&Options_rw, OPT_CRC_LEN);
	/*if(Options_rw.crc != temp_crc) {
		//TODO: display CRC error on LCD
		while(1) {
		  ;
		}
	}*/
	//Options_rw.cnt_fw++;
	//ee_format(&EepromDomain); 
	/*if(EepromDomain.header != 0xABAB) {
		ee_format(&EepromDomain);
	}*/
	init_lcd();
	
	for(uint32_t i=0; i<1000; i++) {
		static uint32_t counter2 = 5;
		const size_t buf_size = lcd.columns_amount + 1;
		char buf[buf_size];
		snprintf(buf, buf_size, "%d", counter2);
		++counter2;
		hd44780_clear(&lcd);
		hd44780_write_string(&lcd, buf);
		HAL_Delay(300);
	}

	//Options_rw.offset[0] = 0x7000;
	//Options_rw.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&Options_rw, OPT_CRC_LEN); //crc32
	//sts = SaveOptToFlash(&Options_rw, &EepromDomain); // 100% work

	AD7792_Reset();
	ADI_Init();
  
		conf[0] = AD7792_GetRegisterValue(AD7792_REG_CONF, 2, 1);
		conf[1] = AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1);
		conf[2] = AD7792_GetRegisterValue(AD7792_REG_IO, 1, 1);
		conf[3] = AD7792_GetRegisterValue(AD7792_REG_OFFSET, 2, 1);
		conf[4] = AD7792_GetRegisterValue(AD7792_REG_FULLSCALE, 2, 1);
		
		//AD7792_Calibrate(&adi1, AD7792_MODE_CAL_SYS_ZERO, AD7792_CH_AIN2P_AIN2M);
		conf[4] = AD7792_GetRegisterValue(AD7792_REG_FULLSCALE, 2, 1);
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
			/*reading data unstable, some times its read only first byte of data*/
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void init_lcd(void)
{
  /* Распиновка дисплея */
  const HD44780_STM32F0xx_Pinout lcd_pinout =
  {
    {
      /* RS        */  { GPIOC, GPIO_PIN_9 },
      /* ENABLE    */  { GPIOC, GPIO_PIN_8 },
      /* RW        */  { NULL, 0 },
      /* Backlight */  { NULL, 0 },
      /* DP0       */  { NULL, 0 },
      /* DP1       */  { NULL, 0 },
      /* DP2       */  { NULL, 0 },
      /* DP3       */  { NULL, 0 },
      /* DP4       */  { GPIOA, GPIO_PIN_8 },
      /* DP5       */  { GPIOA, GPIO_PIN_9 },
      /* DP6       */  { GPIOA, GPIO_PIN_10 },
      /* DP7       */  { GPIOA, GPIO_PIN_11 },
    }
  };

  /* Настраиваем драйвер: указываем интерфейс драйвера (стандартный),
     указанную выше распиновку и обработчик ошибок GPIO (необязателен). */
  lcd_pindriver.interface = HD44780_STM32F0XX_PINDRIVER_INTERFACE;
  /* Если вдруг захотите сами вручную настраивать GPIO для дисплея
     (зачем бы вдруг), напишите здесь ещё (библиотека учтёт это):*/
  //lcd_pindriver.interface.configure = NULL;
  lcd_pindriver.pinout = lcd_pinout;
  lcd_pindriver.assert_failure_handler = hd44780_assert_failure_handler; //hd44780_assert_failure_handler;

  /* �?, наконец, создаём конфигурацию дисплея: указываем наш драйвер,
     функцию задержки, обработчик ошибок дисплея (необязателен) и опции.
     На данный момент доступны две опции - использовать или нет
     вывод RW дисплея (в последнем случае его нужно прижать к GND),
     и то же для управления подсветкой. */
  const HD44780_Config lcd_config =
  {
    (HD44780_GPIO_Interface*)&lcd_pindriver,
    delay_lcd,
    hd44780_assert_failure_handler, //hd44780_assert_failure_handler,
    //HD44780_OPT_USE_RW
  };

  /* Ну, а теперь всё стандартно: подаём тактирование на GPIO,
     инициализируем дисплей: 16x2, 4-битный интерфейс, символы 5x8 точек. */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  hd44780_init(&lcd, HD44780_MODE_4BIT, &lcd_config, 20, 4, HD44780_CHARSIZE_5x8);
}

void delay_lcd(uint16_t ms) {
	HAL_Delay((uint32_t)ms);
//	SysTick->VAL = SysTick->LOAD;
//  const uint32_t systick_ms_start = systick_ms;//increment in Systick handler
//  uint16_t us = ms * 1000;
//  while (1)
//  {
//    uint32_t diff = uint32_time_diff(systick_ms, systick_ms_start);

//    if (diff >= ((uint32_t)us / 1000) + (us % 1000 ? 1 : 0))
//      break;
//  }
	//cast types for supress warnings
	// вызов до старта диспетчера приводит к ошибке
	//osDelay((uint32_t)ms);
}

//uint32_t uint32_time_diff(uint32_t now, uint32_t before)
//{
//  return (now >= before) ? (now - before) : (UINT32_MAX - before + now);
//}

void hd44780_assert_failure_handler(const char *filename, unsigned long line)
{
	__IO static char st_file[30];
	for (uint32_t i=0; i<30; i++) {
		st_file[i]=*(filename+i);
	}
	__IO static unsigned long st_line;
	st_line	= line;
  //(void)filename; (void)line;
  do {} while (1);
}

 /*void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	 //change state
	 dma_t_cplt = 0;
 }
 
 void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	 //
	 dma_r_cplt = 0;
 }*/
/* USER CODE END 4 */

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
