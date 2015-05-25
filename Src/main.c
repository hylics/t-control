/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 25/05/2015 10:51:17
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
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#define ARM_MATH_CM0

#include "AD7792.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint8_t dma_t_cplt=1, dma_r_cplt=1;
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
	
	__IO uint32_t raw_conv[5] = {0};
	__IO uint32_t conf[5] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

	AD7792_Reset();
  if(AD7792_Init() == 1) {
		//
		AD7792_conf(AD7792_GAIN_1, AD7792_CH_AIN2P_AIN2M, AD7792_EN_IXCEN_210uA);
		
    /*		
		//AD7792_SetGain(AD7792_GAIN_1);
		//AD7792_EnableBuf();
		//AD7792_SetIntReference(AD7792_REFSEL_INT);
		//AD7792_SetChannel(AD7792_CH_AIN2P_AIN2M);
		//AD7792_SetUnipolar();
		
		//AD7792_IexDir(AD7792_DIR_IEXC1_IEXC2_IOUT1);
		//AD7792_IexEn(AD7792_EN_IXCEN_210uA);
		*/
		// check config
		conf[0] = AD7792_GetRegisterValue(AD7792_REG_CONF, 2, 1);
		conf[1] = AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1);
		conf[2] = AD7792_GetRegisterValue(AD7792_REG_IO, 1, 1);
		conf[3] = AD7792_GetRegisterValue(AD7792_REG_OFFSET, 2, 1);
		conf[4] = AD7792_GetRegisterValue(AD7792_REG_FULLSALE, 2, 1);
		
		//AD7792_SetMode(AD7792_MODE_SINGLE);
		HAL_Delay(200);
	  while(1) {
			
			//
			/*reading data unstable, some times its read only first byte of data*/
		  __IO uint32_t t_read;
		  for(uint32_t i=0; i<5; i++) {
			  t_read = AD7792_SingleConversion();
		  	if(t_read < 0x60) {
			  	while(1) {}
		  		}
			  raw_conv[i] = t_read;
			  HAL_Delay(200);
	  	} //end for(uint32_t i=0; i<5; i++)

		//raw_conv = AD7792_SingleConversion();
		HAL_Delay(200);
		conf[0] = AD7792_GetRegisterValue(AD7792_REG_CONF, 2, 1);
		conf[1] = AD7792_GetRegisterValue(AD7792_REG_MODE, 2, 1);
		conf[2] = AD7792_GetRegisterValue(AD7792_REG_IO, 1, 1);
		conf[3] = AD7792_GetRegisterValue(AD7792_REG_OFFSET, 2, 1);
		conf[4] = AD7792_GetRegisterValue(AD7792_REG_FULLSALE, 2, 1);
		} //end while(1)
	} //end if(AD7792_Init() == 1)
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
		/**/
			
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
 void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	 //change state
	 dma_t_cplt = 0;
 }
 
 void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	 //
	 dma_r_cplt = 0;
 }
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
