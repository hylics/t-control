/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/06/2015 10:01:22
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
#define ARM_MATH_CM0

#include "soft_spi.h"
#include "eeprom.h"
#include "adi.h"
#include "arm_math.h"
#include "rtd_linearization.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//__IO uint8_t dma_t_cplt=1, dma_r_cplt=1;
extern AD7792_HandleTypeDef adi1;
extern SavedDomain_t SavedDomain;
HAL_StatusTypeDef sts;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
	
	__IO uint32_t raw_conv[8] = {0};
	__IO uint32_t sum_conv[4] = {0};
	__IO float32_t t_rtd[4] = {0.0f};
	__IO uint32_t conf[5] = {0};
	
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
	//SavedDomain_t Copy_Options = SavedDomain;
	//ee_format(&SavedDomain); //hardfault
	/*if(SavedDomain.header != 0xABAB) {
		ee_format(&SavedDomain);
	}*/
	
	//Copy_Options.offset[0] = 0x7000;
	//sts = SaveOptToFlash(&Copy_Options, &SavedDomain);

	AD7792_Reset();
	ADI_Init();
  
		conf[0] = AD7792_GetRegisterValue(AD7792_REG_CONF, 2, 1);
		conf[1] = AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1);
		conf[2] = AD7792_GetRegisterValue(AD7792_REG_IO, 1, 1);
		conf[3] = AD7792_GetRegisterValue(AD7792_REG_OFFSET, 2, 1);
		conf[4] = AD7792_GetRegisterValue(AD7792_REG_FULLSCALE, 2, 1);
		
		__IO uint8_t temp_state=1;
		while (temp_state)
    {
			/*reading data unstable, some times its read only first byte of data*/
		//__IO uint32_t t_read;
		for(uint32_t i=0; i<8; i++) {
			//uint32_t command = AD7792_IEXCEN(AD7792_EN_IXCEN_210uA);
			
			if( i % 2 ) {
				adi1.io &= ~AD7792_IEXCDIR(0x3);
        adi1.io |= AD7792_IEXCDIR(AD7792_DIR_IEXC1_IOUT2_IEXC2_IOUT1);
        AD7792_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
			}
			else {
				adi1.io &= ~AD7792_IEXCDIR(0x3);
        adi1.io |= AD7792_IEXCDIR(AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2);
        AD7792_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
			}
			conf[2] = AD7792_GetRegisterValue(AD7792_REG_IO, 1, 1);
			/*if(t_read < 0x60) {
				while(1) {}
				}*/
			raw_conv[i] = AD7792_SingleConversion(&adi1);
			HAL_Delay(200);
		}
		
		/*sum of A21+A22 measurement*/
		for( uint32_t i=0; i<4; i++) {
			sum_conv[i] = raw_conv[2*i] + raw_conv[(2*i + 1)];
			t_rtd[i] = rtd_get_temp(sum_conv[i], a375, r1000);
		}

		HAL_Delay(200);
		conf[0] = AD7792_GetRegisterValue(AD7792_REG_CONF, 2, 1);
		conf[1] = AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1);
		conf[2] = AD7792_GetRegisterValue(AD7792_REG_IO, 1, 1);
		conf[3] = AD7792_GetRegisterValue(AD7792_REG_OFFSET, 2, 1);
		conf[4] = AD7792_GetRegisterValue(AD7792_REG_FULLSCALE, 2, 1);
	
  }
		
	
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
