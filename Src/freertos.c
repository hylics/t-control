/**
  ******************************************************************************
  * File Name          : freertos.c
  * Date               : 22/06/2015 11:08:50
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "main.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId adcTaskHandle;
osThreadId pidTaskHandle;
osThreadId LcdTaskHandle;
osMutexId Mutex_T_Handle;

/* USER CODE BEGIN Variables */
extern AD7792_HandleTypeDef adi1;
//extern SavedDomain_t EepromDomain;
extern SavedDomain_t Options_rw;
extern TIM_HandleTypeDef htim3;
extern HAL_StatusTypeDef (*pf_output[N_FUNC_PWR])(float32_t pwr);
extern HD44780 lcd;
//extern HD44780_STM32F0xx_GPIO_Driver lcd_pindriver;
__IO static Temperature_t temp_handle = {0.0f};
arm_pid_instance_f32 pid_instance_1;
//__IO static float32_t out_tr;
size_t fre=0;

//unsigned int la_adc_task = 0;
//unsigned int la_pid_task = 0;
//fre=xPortGetFreeHeapSize();
//__IO static float32_t t_rtd = 0.0f;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartAdcTask(void const * argument);
void StartPidTask(void const * argument);
void StartLcdTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
extern void init_lcd(void);
/* USER CODE END FunctionPrototypes */
/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of Mutex_T_ */
  osMutexDef(Mutex_T_);
  Mutex_T_Handle = osMutexCreate(osMutex(Mutex_T_));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of adcTask */
  osThreadDef(adcTask, StartAdcTask, osPriorityHigh, 0, 128);
  adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);

  /* definition and creation of pidTask */
  osThreadDef(pidTask, StartPidTask, osPriorityAboveNormal, 0, 128);
  pidTaskHandle = osThreadCreate(osThread(pidTask), NULL);

  /* definition and creation of LcdTask */
  osThreadDef(LcdTask, StartLcdTask, osPriorityNormal, 0, 128);
  LcdTaskHandle = osThreadCreate(osThread(LcdTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartAdcTask function */
void StartAdcTask(void const * argument)
{

  /* USER CODE BEGIN StartAdcTask */
	TickType_t LastWakeTime;
	//const uint32_t adc_delay = 1000; //milliseconds
	const uint32_t mutex_T_wait = 500; //milliseconds
	static uint32_t filt_conv_rtd;
	
	LastWakeTime = xTaskGetTickCount();
	
  /* Infinite loop */
  for(;;)
  {
		/*reading data unstable, some times its read only first byte of data*/
		__IO uint32_t raw_conv_rtd = 0;
		
		//la_adc_task = 1;
		
		adi1.io &= ~AD7792_IEXCDIR(0x3);
		adi1.io |= AD7792_IEXCDIR(AD7792_DIR_IEXC1_IOUT2_IEXC2_IOUT1);
		
		taskENTER_CRITICAL();
		AD7792_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
		raw_conv_rtd = AD7792_SingleConversion(&adi1);
		taskEXIT_CRITICAL();
		
		adi1.io &= ~AD7792_IEXCDIR(0x3);
		adi1.io |= AD7792_IEXCDIR(AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2);
		
		taskENTER_CRITICAL();
		AD7792_conf(&adi1, reg_io); // CS is modified by SPI read/write functions.
		raw_conv_rtd += AD7792_SingleConversion(&adi1);
		taskEXIT_CRITICAL();
		
		filt_conv_rtd = rec_filter(raw_conv_rtd, 55, 8); // 45=30s, 55=20s
		
		//access through semaphores to temperature state;
		osStatus status = osMutexWait(Mutex_T_Handle, mutex_T_wait);
		if(status == osOK) {
			temp_handle.rtd = rtd_get_temp(filt_conv_rtd, Pt375, r1000);
			temp_handle.setpoint = 25.0f;
			osMutexRelease(Mutex_T_Handle);
		}
		else {
			//do something when error occuring
		}
		
		fre=xPortGetFreeHeapSize();
		
		//la_adc_task = 0;
		
		//osDelay(1000);
		vTaskDelayUntil(&LastWakeTime, 2000);//2000
  }
	//vTaskDelete(NULL);
  /* USER CODE END StartAdcTask */
}

/* StartPidTask function */
void StartPidTask(void const * argument)
{
  /* USER CODE BEGIN StartPidTask */
	TickType_t LastWakeTime;
	//const uint32_t pid_delay = 6000; //milliseconds
	const uint32_t mutex_T_wait = 2000; //milliseconds
	pid_instance_1.Kp = Options_rw.Kp;
	pid_instance_1.Ki = Options_rw.Ki;
	pid_instance_1.Kd = Options_rw.Kd;
	
	arm_pid_init_f32(&pid_instance_1, 1);
	
	//where the best place for this?
	HAL_TIM_Base_Start_IT(&htim3);
	
  /* Infinite loop */
  for(;;)
  {
		float32_t delta_t = 0.0f;//, out_f32 = 0.0f;
		
		//la_pid_task = 1;
		
		osStatus status = osMutexWait(Mutex_T_Handle, mutex_T_wait);
		
		if(status == osOK) {
			//calculate difference of setpoint T and measured T
			if(Options_rw.input == in_rtd) {
				delta_t = temp_handle.setpoint - temp_handle.rtd;
			}
			else {
				delta_t = temp_handle.setpoint - temp_handle.thermocouple;
			}//Options_rw.input
			osMutexRelease(Mutex_T_Handle);
		}//status
		else {
			//do something when error occuring
		}
		
		arm_pid_f32(&pid_instance_1, delta_t);
		
		if(pid_instance_1.state[2] > PID_MAX_FLT) {
			//overflow protection
			pid_instance_1.state[2] = PID_MAX_FLT;
		}
		else if(pid_instance_1.state[2] < PID_MIN_FLT) {
			//underflow protection
			pid_instance_1.state[2] = PID_MIN_FLT;
		}
		//set output power
		pf_output[Options_rw.pf_out](pid_instance_1.state[2]);//use Eeprom or Options_rw to choice output function?
		//set_output(pid_instance_1.state[2], TIM_CHANNEL_1);
		//set_output(out_f32);
		//out_tr = pid_instance_1.state[2];
		
		//la_pid_task = 0;
		
		//osDelay(1000);
    vTaskDelayUntil(&LastWakeTime, 2000);//2000
  }
	//vTaskDelete(NULL);
  /* USER CODE END StartPidTask */
}

/* StartLcdTask function */
void StartLcdTask(void const * argument)
{
  /* USER CODE BEGIN StartLcdTask */
//	TickType_t LastWakeTime;
//	static uint32_t counter2 = 5;

//	const size_t buf_size = lcd.columns_amount + 1;
//	char buf[buf_size];
  /* Infinite loop */
  for(;;)
  {
//		snprintf(buf, buf_size, "%d", counter2);
//		
//    vTaskDelayUntil(&LastWakeTime, 1000);//
//		
//		++counter2;
//    taskENTER_CRITICAL();
//		hd44780_clear(&lcd);
//		hd44780_write_string(&lcd, buf);
//		taskEXIT_CRITICAL();
    osDelay(1);
  }
  /* USER CODE END StartLcdTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
