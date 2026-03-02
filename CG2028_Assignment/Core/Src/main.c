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
static void User_Button_Init(void);

extern int mov_avg(int N, int* accel_buff); // asm implementation
int mov_avg_C(int N, int* accel_buff);      // Reference C implementation

UART_HandleTypeDef huart1;

int main(void)
{
    const int N = 4;

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* UART initialization */
    UART1_Init();

    /* Initialise the button for PC13 */
    User_Button_Init();

    /* Peripheral initializations using BSP functions */
    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_PSENSOR_Init();
    BSP_TSENSOR_Init();

    /* Set the initial LED state to off */
    BSP_LED_Off(LED2);

    int accel_buff_x[4] = {0};
    int accel_buff_y[4] = {0};
    int accel_buff_z[4] = {0};
    int i = 0;

    /* Timing: non-blocking sampling + non-blocking LED blink */
    uint32_t last_sample_tick = 0;
    uint32_t last_led_tick = 0;
    const uint32_t sample_period_ms = 50;

    int delay_ms = 1000; // slow blink default

    /* Barometer baseline / confirmation */
    float pressure_baseline = BSP_PSENSOR_ReadPressure();
    float current_pressure = pressure_baseline;
    bool pressure_confirmed = false;

    /* Temperature reading (only used in emergency msg) */
    float ambient_temp_c = 0.0f;

    /* Fall detection state machine + timers
       States: 0 = NORMAL, 1 = FREEFALL, 2 = IMPACT_WAIT_STILL, 3 = FALL_ALERT */
    static int fall_state = 0;
    static uint32_t freefall_start_tick = 0;
    static uint32_t impact_tick = 0;
    static uint32_t fall_alert_start_tick = 0;

    /* Thresholds */
    const float free_fall_threshold = 5.0f;      // m/s^2
    const float impact_threshold    = 20.0f;     // m/s^2
    const float still_gyro_threshold = 80.0f;    // dps-ish
    const float pressure_rise_threshold = 0.10f; // hPa

    const uint32_t freefall_timeout_ms = 600;       // must see impact within this window
    const uint32_t stillness_window_ms = 800;       // time after impact to look for stillness
    const uint32_t fall_alert_duration_ms = 10000;  // 10 seconds to press "I AM OK"

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* Non-blocking LED blink: toggle based on delay_ms */
        if ((now - last_led_tick) >= (uint32_t)delay_ms)
        {
            BSP_LED_Toggle(LED2);
            last_led_tick = now;
        }

        /* Run the sensor + detection logic every 50ms (non-blocking) */
        if ((now - last_sample_tick) < sample_period_ms)
        {
            continue;
        }
        last_sample_tick = now;

        /* Read accelerometer */
        int16_t accel_data_i16[3] = {0};
        BSP_ACCELERO_AccGetXYZ(accel_data_i16);

        accel_buff_x[i % 4] = accel_data_i16[0];
        accel_buff_y[i % 4] = accel_data_i16[1];
        accel_buff_z[i % 4] = accel_data_i16[2];

        /* Read gyroscope */
        float gyro_data[3] = {0.0f};
        BSP_GYRO_GetXYZ(gyro_data);

        float gyro_velocity[3] = {0.0f};
        gyro_velocity[0] = (gyro_data[0] * 9.8f / 1000.0f);
        gyro_velocity[1] = (gyro_data[1] * 9.8f / 1000.0f);
        gyro_velocity[2] = (gyro_data[2] * 9.8f / 1000.0f);

        /* Read barometer */
        current_pressure = BSP_PSENSOR_ReadPressure();

        /* Filtered accelerometer (assembly) */
        float accel_filt_asm[3] = {0};
        accel_filt_asm[0] = (float)mov_avg(N, accel_buff_x) * (9.8f / 1000.0f);
        accel_filt_asm[1] = (float)mov_avg(N, accel_buff_y) * (9.8f / 1000.0f);
        accel_filt_asm[2] = (float)mov_avg(N, accel_buff_z) * (9.8f / 1000.0f);

        /* Magnitudes */
        float accelerationMagnitude = sqrtf(
            accel_filt_asm[0] * accel_filt_asm[0] +
            accel_filt_asm[1] * accel_filt_asm[1] +
            accel_filt_asm[2] * accel_filt_asm[2]
        );

        float gyroMagnitude = sqrtf(
            gyro_velocity[0] * gyro_velocity[0] +
            gyro_velocity[1] * gyro_velocity[1] +
            gyro_velocity[2] * gyro_velocity[2]
        );

        /* ========================== FALL STATE MACHINE ========================== */

        if (fall_state == 0)
        {
            pressure_confirmed = false;
            delay_ms = 1000;

            if (accelerationMagnitude < free_fall_threshold)
            {
                fall_state = 1;
                freefall_start_tick = now;
                pressure_baseline = current_pressure;
            }
        }
        else if (fall_state == 1)
        {
            if ((now - freefall_start_tick) > freefall_timeout_ms)
            {
                fall_state = 0;
            }
            else
            {
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
            if ((now - impact_tick) <= stillness_window_ms)
            {
                if (gyroMagnitude < still_gyro_threshold || pressure_confirmed)
                {
                    fall_state = 3;
                    fall_alert_start_tick = now;
                    delay_ms = 100;
                }
            }
            else
            {
                fall_state = 0;
            }
        }
        else if (fall_state == 3)
        {
            delay_ms = 100;

            /* PC13 user button is ACTIVE-LOW: pressed -> GPIO_PIN_RESET */
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
            {
                fall_state = 0;
                delay_ms = 1000;

                char cancel_msg[] = "\r\n--- USER PRESSED 'I AM OK'. ALARM CANCELLED ---\r\n\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)cancel_msg, strlen(cancel_msg), HAL_MAX_DELAY);

                /* Debounce delay */
                HAL_Delay(200);
            }
            else if ((now - fall_alert_start_tick) > fall_alert_duration_ms)
            {
                /* Read temperature when escalating */
                ambient_temp_c = BSP_TSENSOR_ReadTemp();

                char emergency_msg[220];
                snprintf(emergency_msg, sizeof(emergency_msg),
                         "\r\n!!! NO RESPONSE. INITIATING EMERGENCY PROTOCOL !!!\r\n"
                         "Ambient Temp: %.2f C | Pressure: %.2f hPa\r\n\n",
                         ambient_temp_c, current_pressure);

                HAL_UART_Transmit(&huart1, (uint8_t*)emergency_msg, strlen(emergency_msg), HAL_MAX_DELAY);

                fall_state = 0;
                delay_ms = 1000;
            }
        }

        /* ========================== UART DEBUG PRINT ========================== */
        char buffer[200];

        if (i >= 3)
        {
            snprintf(buffer, sizeof(buffer),
                     "AccelMag:%f GyroMag:%f Pressure:%f State:%d PressureOK:%d\r\n",
                     accelerationMagnitude, gyroMagnitude, current_pressure, fall_state,
                     pressure_confirmed ? 1 : 0);

            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        }

        i++;
    }
}

/* Reference moving average (C) */
int mov_avg_C(int N, int* accel_buff)
{
    int result = 0;
    for (int i = 0; i < N; i++)
    {
        result += accel_buff[i];
    }
    result = result / N;
    return result;
}

static void User_Button_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    /* PC13 typically needs pull-up if board doesn't have it externally */
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
        while (1);
    }
}

/* Do not modify these lines of code. They are written to suppress UART related warnings */
int _write(int file, char *ptr, int len) { return len; }
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }