  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/


/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <math.h> // for the sqrt function
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"

#include <stdio.h>
#include "string.h"
#include <sys/stat.h>

static void UART1_Init(void);
static void User_Button_Init();
extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART
// void Update_Distress_Characteristic(uint8_t status);
extern int mov_avg(int N, int* accel_buff); // asm implementation

int mov_avg_C(int N, int* accel_buff); // Reference C implementation

UART_HandleTypeDef huart1;


int main(void)
{
	const int N=4;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* UART initialization  */
	UART1_Init();

	/* Initialise the button for PC13*/
	User_Button_Init();

	/* Peripheral initializations using BSP functions */
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_PSENSOR_Init();

	/*Set the initial LED state to off*/
	BSP_LED_Off(LED2);

	int accel_buff_x[4]={0};
	int accel_buff_y[4]={0};
	int accel_buff_z[4]={0};
	int i=0;

	// Timing: non-blocking sampling + non-blocking LED blink
	uint32_t last_sample_tick = 0;
	uint32_t last_led_tick = 0;
	const uint32_t sample_period_ms = 50;

	int delay_ms=1000; //change delay time to suit your code

	// Barometer baseline / confirmation
	float pressure_baseline = BSP_PSENSOR_ReadPressure();
	float current_pressure = pressure_baseline;
	bool pressure_confirmed = false;

	// Fall detection state machine + timers
	// States: 0 = NORMAL, 1 = FREEFALL, 2 = IMPACT_WAIT_STILL, 3 = FALL_ALERT
	static int fall_state = 0;
	static uint32_t freefall_start_tick = 0;
	static uint32_t impact_tick = 0;
	static uint32_t fall_alert_start_tick = 0;

	// Thresholds (tune later with UART prints)
	const float free_fall_threshold = 7.5f;   // m/s^2  (below this indicates near freefall)
	const float impact_threshold    = 15.0f;  // m/s^2  (above this indicates impact spike)
	const float still_gyro_threshold = 80.0f; // dps-ish (must be low to indicate lying still)
	const float pressure_rise_threshold = 0.10f; // hPa increase indicates lower altitude

	const uint32_t freefall_timeout_ms = 1200;      // must see impact within this window
	const uint32_t stillness_window_ms = 2000;      // time after impact to look for stillness
	const uint32_t fall_alert_duration_ms = 10000;  // fast blink duration then reset to normal, this will be set to 10 seconds

	while (1)
	{
		uint32_t now = HAL_GetTick();

		// Non-blocking LED blink: toggle based on delay_ms
		if ((now - last_led_tick) >= (uint32_t)delay_ms)
		{
			BSP_LED_Toggle(LED2);		// This function helps to toggle the current LED state
			last_led_tick = now;
		}

		// Run the sensor + detection logic every 50ms (non-blocking)
		if ((now - last_sample_tick) < sample_period_ms)
		{
			continue;
		}
		last_sample_tick = now;

		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings of accelerometer
		/********Function call to read accelerometer values*********/
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);

		//Copy the values over to a circular style buffer
		accel_buff_x[i%4]=accel_data_i16[0]; //acceleration along X-Axis
		accel_buff_y[i%4]=accel_data_i16[1]; //acceleration along Y-Axis
		accel_buff_z[i%4]=accel_data_i16[2]; //acceleration along Z-Axis


		// ********* Read gyroscope values *********/
		float gyro_data[3]={0.0};
		float* ptr_gyro=gyro_data;
		BSP_GYRO_GetXYZ(ptr_gyro);

		//The output of gyro has been made to display in dps(degree per second)
		float gyro_velocity[3]={0.0};
		gyro_velocity[0]=(gyro_data[0]*9.8/(1000));
		gyro_velocity[1]=(gyro_data[1]*9.8/(1000));
		gyro_velocity[2]=(gyro_data[2]*9.8/(1000));

		// ********* Read barometer values *********/
		current_pressure = BSP_PSENSOR_ReadPressure();

		//Preprocessing the filtered outputs  The same needs to be done for the output from the C program as well
		float accel_filt_asm[3]={0}; // final value of filtered acceleration values

		accel_filt_asm[0]= (float)mov_avg(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_asm[1]= (float)mov_avg(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_asm[2]= (float)mov_avg(N,accel_buff_z) * (9.8/1000.0f);


		//Preprocessing the filtered outputs  The same needs to be done for the output from the assembly program as well
		float accel_filt_c[3]={0};

		accel_filt_c[0]=(float)mov_avg_C(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_c[1]=(float)mov_avg_C(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_c[2]=(float)mov_avg_C(N,accel_buff_z) * (9.8/1000.0f);

		// ********* Fall detection *********/
		// write your program from here:

		// calculate the magnitude of the acceleration (sqrt(ax^2 + ay^2 + az^2))
		float accelerationMagnitude = sqrt((accel_filt_asm[0] * accel_filt_asm[0]) + (accel_filt_asm[1] * accel_filt_asm[1])
				+ accel_filt_asm[2] * accel_filt_asm[2]);

		// calculate the magnitude of the gyro readings
		float gyroMagnitude = sqrt((gyro_velocity[0] * gyro_velocity[0]) + (gyro_velocity[1] * gyro_velocity[1])
				+ gyro_velocity[2] * gyro_velocity[2]);

		// State machine:
		// 0 NORMAL -> 1 FREEFALL when accel drops
		// 1 FREEFALL -> 2 IMPACT_WAIT_STILL when impact spike occurs
		// 2 IMPACT_WAIT_STILL -> 3 FALL_ALERT when (stillness OR barometer confirm) within window
		// 3 FALL_ALERT -> 0 NORMAL after duration (auto reset)

		if (fall_state == 0)
		{
			pressure_confirmed = false;
			delay_ms = 1000; // slow blinking to indicate normal activity

			if (accelerationMagnitude < free_fall_threshold)
			{
				fall_state = 1;
				freefall_start_tick = now;
				pressure_baseline = current_pressure; // capture baseline at free-fall start
			}
		}
		else if (fall_state == 1)
		{
			// If no impact soon, reset
			if ((now - freefall_start_tick) > freefall_timeout_ms)
			{
				fall_state = 0;
			}
			else
			{
				// Barometer can confirm drop (pressure increases when lower)
				if ((current_pressure - pressure_baseline) > pressure_rise_threshold)
				{
					pressure_confirmed = true;
				}

				if (accelerationMagnitude > impact_threshold)
				{
					fall_state = 2;
					impact_tick = now;
				}
			}
		}
		else if (fall_state == 2)
		{
			// We do NOT require high gyro here (because lifting causes high gyro).
			// Instead, after impact, we look for "stillness" (gyro low) typical of lying on floor.
			if ((now - impact_tick) <= stillness_window_ms)
			{
				if (gyroMagnitude < still_gyro_threshold)
				{
					// Confirm fall (impact + then still)
					fall_state = 3;
					fall_alert_start_tick = now;
					delay_ms = 100; // fast blinking to indicate fall
					char gyro_alert[] = "\r\n*** FALL DETECTED (Gyro Stillness Confirmed)! ***\r\n--- Waiting 10 seconds for User OK Button... ---\r\n\n";
					HAL_UART_Transmit(&huart1, (uint8_t*)gyro_alert, strlen(gyro_alert), HAL_MAX_DELAY);
				}
				else if (pressure_confirmed)
				{
					// Alternate confirm: impact + pressure rise sustained
					fall_state = 3;
					fall_alert_start_tick = now;
					delay_ms = 100; // fast blinking to indicate fall
					char pressure_alert[] = "\r\n*** FALL DETECTED (Barometer Drop Confirmed)! ***\r\n--- Waiting 10 seconds for User OK Button... ---\r\n\n";
					HAL_UART_Transmit(&huart1, (uint8_t*)pressure_alert, strlen(pressure_alert), HAL_MAX_DELAY);

				}
			}
			else
			{
				// If we didn't get confirmation soon after impact, reset
				fall_state = 0;
			}
		}
		else if (fall_state == 3)
		{
			delay_ms = 100; // fast blinking to indicate fall

//			// Auto reset after alert duration so it doesn't blink forever
//			if ((now - fall_alert_start_tick) > fall_alert_duration_ms)
//			{
//				fall_state = 0;
//			}

			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
				fall_state = 0; // that means that the user has indicated that he is ok
				delay_ms = 1000; // go back to blinking the LED slowly

				char cancel_msg[] = "\r\n--- USER PRESSED 'I AM OK'. ALARM CANCELLED! HE IS ALRIGHT! ---\r\n\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)cancel_msg, strlen(cancel_msg), HAL_MAX_DELAY);
				HAL_Delay(300);

			} else if (now - fall_alert_start_tick > fall_alert_duration_ms){ // if the duration exceeds 10 seconds
				char emergency_msg[] = "\r\n!!! NO RESPONSE. INITIATING EMERGENCY PROTOCOL !!!\r\n\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)emergency_msg, strlen(emergency_msg), HAL_MAX_DELAY);
				fall_state = 0;
			}
		}

		/***************************UART transmission*******************************************/
		char buffer[200]; // Create a buffer large enough to hold the text

		if(i>=3)
		{
			sprintf(buffer,
					"AccelMag:%f GyroMag:%f Pressure:%f State:%d PressureOK:%d\r\n",
					accelerationMagnitude, gyroMagnitude, current_pressure, fall_state, pressure_confirmed ? 1 : 0);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		}

		i++;
	}
}



int mov_avg_C(int N, int* accel_buff)
{ 	// The implementation below is inefficient and meant only for verifying your results.
	int result=0;
	for(int i=0; i<N;i++)
	{
		result+=accel_buff[i];
	}

	result=result/N;

	return result;
}

static void User_Button_Init() {
	// Enable the clock for GPIO Port C
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// Configure PC13 as an input pin
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void UART1_Init(void)
{
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
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
          while(1);
        }

}


// Do not modify these lines of code. They are written to supress UART related warnings
int _write(int file, char *ptr, int len) { return len; }
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
