/**
  ******************************************************************************
  * File Name          : freertos.c
  * Date               : 27/07/2015 13:18:06
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
osThreadId ProgramTaskHandle;
osMutexId Mutex_T_Handle;

/* USER CODE BEGIN Variables */
extern AD7792_HandleTypeDef adi1;
//extern SavedDomain_t EepromDomain;
extern SavedDomain_t Options_rw;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart4;
extern HAL_StatusTypeDef (*pf_output[N_FUNC_PWR])(float32_t pwr);
extern HD44780 lcd;
//extern HD44780_STM32F0xx_GPIO_Driver lcd_pindriver;
__IO static Temperature_t temp_handle = {0.0f};
arm_pid_instance_f32 pid_instance_1;
//__IO static float32_t out_tr;
size_t fre=0;
__IO static uint8_t CPU_IDLE = 0, malloc_err=0;


//unsigned int la_adc_task = 0;
//unsigned int la_pid_task = 0;
//fre=xPortGetFreeHeapSize();
//__IO static float32_t t_rtd = 0.0f;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartAdcTask(void const * argument);
void StartPidTask(void const * argument);
void StartLcdTask(void const * argument);
void StartProgramTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
extern void init_lcd(void);
/* USER CODE END FunctionPrototypes */
/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	 static portTickType LastTick; 
        static uint32_t count;             //наш трудяга счетчик
        static uint32_t max_count ;                //максимальное значение счетчика, вычисляется при калибровке и соответствует 100% CPU idle

        count++;                                                  //приращение счетчика

        if((xTaskGetTickCount() - LastTick ) > 1000)    { //если прошло 1000 тиков (1 сек для моей платфрмы)
                LastTick = xTaskGetTickCount();
                if(count > max_count) max_count = count;          //это калибровка
                CPU_IDLE = (100 * count) / max_count;               //вычисляем текущую загрузку
                count = 0;                                        //обнуляем счетчик
        }

}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
__IO	static int8_t taskname[30];
	for(uint32_t i=0; i<30; i++) {
		taskname[i]=*(pcTaskName+i);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	malloc_err++;
}
/* USER CODE END 5 */

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

  /* definition and creation of ProgramTask */
  osThreadDef(ProgramTask, StartProgramTask, osPriorityNormal, 0, 128);
  ProgramTaskHandle = osThreadCreate(osThread(ProgramTask), NULL);

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
	TickType_t LastWakeTime = xTaskGetTickCount();
	const TickType_t adc_delay = 2000; //milliseconds
	const uint32_t mutex_T_wait = 500; //milliseconds
	static uint32_t filt_conv_rtd;
	osDelay(1000); //for maxcount calibration
	//LastWakeTime = xTaskGetTickCount();
	
	uint8_t msg1[]="adc_task\n";
	
  /* Infinite loop */
  for(;;)
  {
		/*reading data unstable, some times its read only first byte of data*/
		__IO uint32_t raw_conv_rtd = 0;
		
		//la_adc_task = 1;
		HAL_UART_Transmit(&huart4, msg1, sizeof(msg1), 5);
		
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
		
		//osDelay(2000);
		vTaskDelayUntil(&LastWakeTime, adc_delay);//2000
  }
	//vTaskDelete(NULL);
  /* USER CODE END StartAdcTask */
}

/* StartPidTask function */
void StartPidTask(void const * argument)
{
  /* USER CODE BEGIN StartPidTask */
	TickType_t LastWakeTime = xTaskGetTickCount();
	const TickType_t pid_delay = 2000; //milliseconds
	const uint32_t mutex_T_wait = 2000; //milliseconds
	pid_instance_1.Kp = Options_rw.Kp;
	pid_instance_1.Ki = Options_rw.Ki;
	pid_instance_1.Kd = Options_rw.Kd;
	osDelay(1000); //for maxcount calibration
	
	arm_pid_init_f32(&pid_instance_1, 1);
	
	uint8_t msg1[]="pid_task\n";
	
	//where the best place for this?
	HAL_TIM_Base_Start_IT(&htim3);
	
  /* Infinite loop */
  for(;;)
  {
		float32_t delta_t = 0.0f;//, out_f32 = 0.0f;
		
		//la_pid_task = 1;
		HAL_UART_Transmit(&huart4, msg1, sizeof(msg1), 5);
		
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
		
		//osDelay(2000);
    vTaskDelayUntil(&LastWakeTime, pid_delay);//2000
  }
	//vTaskDelete(NULL);
  /* USER CODE END StartPidTask */
}

/* StartLcdTask function */
void StartLcdTask(void const * argument)
{
  /* USER CODE BEGIN StartLcdTask */
	static osStatus status;
	TickType_t LastWakeTime = xTaskGetTickCount();
	const TickType_t lcd_delay = 1000;
	const uint32_t mutex_T_wait = 2000; //milliseconds
//	//static uint32_t counter2 = 5;
  osDelay(1000); //for maxcount calibration
	const size_t buf_size = lcd.columns_amount + 1;
	char buf[buf_size];
	uint8_t msg1[]="lcd_task\n";
  /* Infinite loop */
  for(;;)
  {
		HAL_UART_Transmit(&huart4, msg1, sizeof(msg1), 5);
		
		status = osMutexWait(Mutex_T_Handle, mutex_T_wait);
		if(status == osOK) {
			snprintf(buf, buf_size, "%5.2f", temp_handle.rtd);
			osMutexRelease(Mutex_T_Handle);
//			//++counter2;
			hd44780_clear(&lcd);
			hd44780_write_string(&lcd, buf);
		}
//		snprintf(buf, buf_size, "%d", counter2);
//		
    vTaskDelayUntil(&LastWakeTime, lcd_delay);//
//		
//		++counter2;
//    taskENTER_CRITICAL();
//		hd44780_clear(&lcd);
//		hd44780_write_string(&lcd, buf);
//		taskEXIT_CRITICAL();
    //osDelay(1000);
  }
  /* USER CODE END StartLcdTask */
}

/* StartProgramTask function */
void StartProgramTask(void const * argument)
{
  /* USER CODE BEGIN StartProgramTask */
	static osStatus status;
	TickType_t LastWakeTime = xTaskGetTickCount();
	const TickType_t program_delay = 1000;
	const uint32_t mutex_T_wait = 2000; //milliseconds
	static uint32_t time=0;
	static float32_t alpha=0, beta=0;
	uint32_t prog=0;
  uint32_t	step=0;
	osDelay(30000);
	status = osMutexWait(Mutex_T_Handle, mutex_T_wait);
	if(status == osOK) {
		if (Options_rw.input == in_rtd) {
			Options_rw.prog[prog][0].y = temp_handle.rtd;
		}
		else {
			Options_rw.prog[prog][0].y = temp_handle.thermocouple;
		}
		osMutexRelease(Mutex_T_Handle);
	}
	else {
		//do something when error occuring
	}//status
	
	osDelay(1000); //for maxcount calibration
  /* Infinite loop */
  for(;;)
  { 
		//update coefficients
		if (time >= Options_rw.prog[prog][step].time) {
		  
		  alpha = (Options_rw.prog[prog][step+1].y - Options_rw.prog[prog][step].y)
			          /
			  		  (Options_rw.prog[prog][step+1].time - Options_rw.prog[prog][step].time);
			beta = Options_rw.prog[prog][step+1].y - Options_rw.prog[prog][step+1].time * alpha;
			
			if(step<N_STEP) { //not end program?
				//next step is has valid time?
				if(Options_rw.prog[prog][step+1].time > Options_rw.prog[prog][step].time) {
					step++;
			  }
			}//(step<N_STEP)
		}//update coefficients
		
		status = osMutexWait(Mutex_T_Handle, mutex_T_wait);
		if(status == osOK) {
			//calculate setpoint by a linear equation
			temp_handle.setpoint = (float32_t)(alpha * time) + beta;
			osMutexRelease(Mutex_T_Handle);
		}
		else {
			//do something when error occuring
		}
		
		time++;
    vTaskDelayUntil(&LastWakeTime, program_delay);
  }
  /* USER CODE END StartProgramTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
