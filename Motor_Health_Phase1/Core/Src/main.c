/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Motor Health Telemetry & Diagnostic System (Phase 7 Final)
  *                   - Closed-Loop PI Speed Control Locked at 150.0 RPM
  *                   - 50 ms Sampling Window to Eliminate Encoder Jitter
  *                   - Signed ACS712 Current Sensing with On-Demand Re-Zero
  *                   - 200 Hz TIM7 Vibration Sampling with Backlog Tick Counter
  *                   - 256-Point Real Radix-2 FFT with Hardware DLPF & DWT Profiling
  *                   - Non-Blocking Sliced Serial Dashboard & 16x2 LCD Telemetry
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define ENCODER_PPR             334.0f
#define CONTROL_DT_MS           50       /* 50 ms window: smooth RPM */
#define PWM_MAX_LIMIT           999.0f
#define PWM_MIN_LIMIT           0.0f
#define CPU_HZ                  170000000.0f

/* Closed-Loop Mode: Set to 0 to strictly enforce target 150 RPM PI control */
#define OPEN_LOOP_TEST_PWM      0

#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_CONFIG          0x1A
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B

#define FFT_SAMPLES             256
#define SAMPLING_FREQ_HZ        200.0f

/* ACS712 Current Sensing Settings */
#define ACS712_SENS_V_PER_A     0.185f   /* 5A module = 185 mV/A */
#define CURRENT_SIGN            1.0f     /* Invert to -1.0f if current reads negative */
#define CURRENT_READ_MS         20       /* 50 Hz dedicated current loop */
#define CURRENT_AVG_SAMPLES     256      /* 256-sample averaging filter */
#define CURRENT_DEADBAND_A      0.008f   /* 8 mA noise suppressor */

/* Stall / Motor-Off Guard Settings */
#define STALL_RPM_THRESHOLD     3.0f
#define STALL_PWM_THRESHOLD     900.0f
#define STALL_TIME_MS           2000
#define STALL_PWM_LIMIT         250.0f   /* Soft restart clamp */
#define STALL_CLEAR_RPM         10.0f

#ifndef M_PI
#define M_PI                    3.14159265358979323846f
#endif

/* LCD GPIO Pin Assignments:
   RS -> PC0, EN -> PC1, D4 -> PC2, D5 -> PC3, D6 -> PB0, D7 -> PB1 */
#define LCD_RS_PORT             GPIOC
#define LCD_RS_PIN              GPIO_PIN_0
#define LCD_EN_PORT             GPIOC
#define LCD_EN_PIN              GPIO_PIN_1

#define LCD_D4_PORT             GPIOC
#define LCD_D4_PIN              GPIO_PIN_2
#define LCD_D5_PORT             GPIOC
#define LCD_D5_PIN              GPIO_PIN_3
#define LCD_D6_PORT             GPIOB
#define LCD_D6_PIN              GPIO_PIN_0
#define LCD_D7_PORT             GPIOB
#define LCD_D7_PIN              GPIO_PIN_1

typedef enum {
    HEALTH_NORMAL = 0,
    HEALTH_WARNING,
    HEALTH_CRITICAL_FAULT
} HealthState_t;

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

/* Speed Control Loop Variables (Strict 150 RPM Target) */
float Kp = 0.5f;
float Ki = 2.0f;
float target_rpm = 150.0f;
float raw_rpm = 0.0f;
float filtered_rpm = 0.0f;
float speed_error = 0.0f;
float integral_accum = 0.0f;
float control_output = 0.0f;

volatile int32_t last_encoder_counter = 0;
int32_t last_delta_pulses = 0;
float   last_dt_ms = 0.0f;
uint32_t last_pi_cyc = 0;
uint32_t last_control_tick = 0;
uint32_t last_telemetry_tick = 0;
uint32_t last_current_tick = 0;
uint32_t last_lcd_toggle = 0;

/* Stall Guard State */
uint8_t  stall_flag = 0;
uint32_t stall_start_tick = 0;
uint8_t  stall_timing = 0;

/* Current Sensor Variables */
uint32_t adc_raw = 0;
float adc_voltage = 0.0f;
float motor_current_A = 0.0f;
float zero_current_v = 2.515f;
volatile uint8_t recalibrate_request = 0;

/* MPU6050 Acquisition & Buffer */
uint8_t mpu_ok = 0;
uint8_t active_mpu_addr = 0x68;
volatile uint8_t sample_pending = 0;

float ax_buf[FFT_SAMPLES];
float ay_buf[FFT_SAMPLES];
float az_buf[FFT_SAMPLES];
uint16_t sample_idx = 0;
uint8_t buffer_ready = 0;

/* Time-Domain & Spectral Features */
float vib_rms_x = 0.0f;
float vib_rms_y = 0.0f;
float vib_rms_z = 0.0f;
float total_vib_rms = 0.0f;
float peak_to_peak_g = 0.0f;

float fft_real[FFT_SAMPLES];
float fft_imag[FFT_SAMPLES];
float fft_mag[FFT_SAMPLES / 2];
float dominant_freq_hz = 0.0f;
float dominant_mag = 0.0f;
float expected_shaft_freq_hz = 0.0f;

/* Profiling Metrics */
uint32_t fft_cycles = 0;
float fft_execution_time_us = 0.0f;

/* Stage 2 Diagnostics & Scoring */
float pen_i = 0.0f;
float pen_v = 0.0f;
float pen_r = 0.0f;
float health_score = 100.0f;
HealthState_t current_health_state = HEALTH_NORMAL;

uint8_t lcd_page = 0;

/* Non-Blocking Dashboard State */
static uint8_t print_active = 0;
static uint8_t print_step = 0;

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);

static void DWT_Init(void);
static void Update_PI_Controller(void);
static float ADC_Read_Voltage_Avg(uint16_t n);
static void Calibrate_Zero_Current(void);
static void Read_ACS712_Current(void);
static void Acquire_Sensor_Sample(void);
static void Process_Vibration_DSP(void);
static void Execute_Real_FFT(void);
static void Calculate_Health_Score(void);
static void Dashboard_Print_Step(void);

/* LCD Driver Prototypes */
static void LCD_Delay_us(uint32_t us);
static void LCD_Pulse_Enable(void);
static void LCD_Send_4Bits(uint8_t val);
static void LCD_Send_Cmd(uint8_t cmd);
static void LCD_Send_Data(uint8_t data);
static void LCD_Init(void);
static void LCD_SetCursor(uint8_t row, uint8_t col);
static void LCD_Print(const char *str);
static void LCD_Update_Display(void);

/* Bit-Bang I2C Functions */
static void I2C_Bus_Delay(void);
static void I2C_Bus_Clear(void);
static void I2C_Start(void);
static void I2C_Stop(void);
static uint8_t I2C_WriteByte(uint8_t byte);
static uint8_t I2C_ReadByte(uint8_t send_ack);
static uint8_t MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t val);
static uint8_t MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
static void Init_MPU6050(void);

/* Main Entry Point ----------------------------------------------------------*/
int main(void)
{
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2)); /* Enable Hardware FPU */
  #endif

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();

  DWT_Init();

  /* PWM Pin Configuration */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Motor Driver Direction/Enable Pins PC8, PC9 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  /* Bring Up 16x2 HD44780 LCD */
  LCD_Init();

  I2C_Bus_Clear();
  HAL_Delay(50);
  Init_MPU6050();

  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Boot-time zero-current calibration */
  Calibrate_Zero_Current();
  LCD_Update_Display();

  BSP_LED_On(LED_GREEN);
  printf("\033[2J\033[H");
  fflush(stdout);

  /* Start 200 Hz Sampling Timer */
  HAL_TIM_Base_Start_IT(&htim7);

  last_encoder_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  last_pi_cyc = DWT->CYCCNT;
  {
    uint32_t t = HAL_GetTick();
    last_control_tick   = t;
    last_current_tick   = t;
    last_telemetry_tick = t;
    last_lcd_toggle     = t;
  }

  while (1)
  {
    /* On-demand recalibration requested by Blue User Button */
    if (recalibrate_request)
    {
      recalibrate_request = 0;
      Calibrate_Zero_Current();

      filtered_rpm = 0.0f;
      stall_flag = 0;
      stall_timing = 0;
      sample_idx = 0;
      sample_pending = 0;
      last_encoder_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
      last_pi_cyc = DWT->CYCCNT;

      uint32_t t = HAL_GetTick();
      last_control_tick   = t;
      last_current_tick   = t;
      last_telemetry_tick = t;
      last_lcd_toggle     = t;
    }

    uint32_t now = HAL_GetTick();

    /* 200 Hz Sensor Ingestion (Drain Pending Backlog) */
    if (sample_pending)
    {
      __disable_irq();
      sample_pending--;
      __enable_irq();
      Acquire_Sensor_Sample();
    }

    /* 20 Hz (50 ms) PI Speed Control Loop */
    if ((now - last_control_tick) >= CONTROL_DT_MS)
    {
      last_control_tick = now;
      Update_PI_Controller();
    }

    /* 50 Hz Dedicated Current Sampling Loop */
    if ((now - last_current_tick) >= CURRENT_READ_MS)
    {
      last_current_tick = now;
      Read_ACS712_Current();
    }

    /* 256-Point FFT & Feature Pipeline */
    if (buffer_ready)
    {
      buffer_ready = 0;
      Process_Vibration_DSP();

      uint32_t c_start = DWT->CYCCNT;
      Execute_Real_FFT();
      uint32_t c_end = DWT->CYCCNT;

      fft_cycles = c_end - c_start;
      fft_execution_time_us = ((float)fft_cycles / 170.0f);
    }

    /* 5 Hz Display Refresh (200 ms) */
    if ((now - last_telemetry_tick) >= 200)
    {
      last_telemetry_tick = now;
      Calculate_Health_Score();

      expected_shaft_freq_hz = filtered_rpm / 60.0f;

      if ((now - last_lcd_toggle) >= 3000)
      {
        last_lcd_toggle = now;
        lcd_page = !lcd_page;
      }
      LCD_Update_Display();

      if (!print_active)
      {
        print_active = 1;
        print_step = 0;
      }
    }

    /* Non-blocking UART output (1 line per loop pass) */
    if (print_active)
    {
      Dashboard_Print_Step();
    }
  }
}

/* Non-blocking dashboard: prints one line per call --------------------------*/
static void Dashboard_Print_Step(void)
{
  switch (print_step)
  {
    case 0:
      printf("\033[H=================================================================\r\n");
      break;
    case 1:
      printf("           PHASE 7 & 8: REAL-TIME DSP & HEALTH ENGINE            \r\n");
      break;
    case 2:
      printf("=================================================================\r\n");
      break;
    case 3:
      printf(" SPEED CTRL    : TARGET: %5.1f RPM  | ACTUAL: %5.1f RPM%s\r\n",
             target_rpm, filtered_rpm, stall_flag ? " [STALL]" : "        ");
      break;
    case 4:
      printf(" ACTUATION     : PWM: %4lu/999 | I_ACC: %6.1f\r\n", (uint32_t)control_output, integral_accum);
      break;
    case 5:
      printf(" ENCODER DEBUG : CNT: %10lu | DELTA: %5ld | DT: %5.2f ms\r\n",
             (uint32_t)__HAL_TIM_GET_COUNTER(&htim2), (long)last_delta_pulses, last_dt_ms);
      break;
    case 6:
      printf(" HEALTH ENGINE : SCORE : %3.0f / 100   | STATUS: [%-14s]\r\n",
             health_score,
             (current_health_state == HEALTH_NORMAL) ? "NORMAL" :
             (current_health_state == HEALTH_WARNING) ? "WARNING" : "CRITICAL FAULT");
      break;
    case 7:
      printf(" ARMATURE CURR : %+7.1f mA | ADC V: %5.3f V (Zero: %5.3f V)\r\n",
             motor_current_A * 1000.0f, adc_voltage, zero_current_v);
      break;
    case 8:
      printf(" PENALTIES     : CURR: -%4.1f | VIB: -%4.1f | RPM: -%4.1f\r\n", pen_i, pen_v, pen_r);
      break;
    case 9:
      printf("-----------------------------------------------------------------\r\n");
      break;
    case 10:
      if (mpu_ok)
        printf(" MPU6050       : ONLINE (0x%02X)    | N=%d BINS (df=%.3f Hz)\r\n",
               active_mpu_addr, FFT_SAMPLES, (SAMPLING_FREQ_HZ / FFT_SAMPLES));
      else
      {
        printf(" MPU6050       : OFFLINE / BUS FAULT DETECTED\r\n");
        print_step = 16;
      }
      break;
    case 11:
      printf(" AXIS RMS (g)  : X: %5.3f  | Y: %5.3f  | Z: %5.3f\r\n", vib_rms_x, vib_rms_y, vib_rms_z);
      break;
    case 12:
      printf(" VIB METRICS   : TOTAL RMS: %5.3f g | PK-TO-PK: %5.3f g\r\n", total_vib_rms, peak_to_peak_g);
      break;
    case 13:
      printf("-----------------------------------------------------------------\r\n");
      break;
    case 14:
      printf(" SHAFT FUND. f0: %5.2f Hz  (RPM/60)\r\n", expected_shaft_freq_hz);
      printf(" DOMINANT PEAK : %5.2f Hz  (Spectral Mag: %6.2f)\r\n", dominant_freq_hz, dominant_mag);
      break;
    case 15:
      printf(" DWT PROFILING : FFT RUN: %lu CYCLES (%.2f us @ 170MHz)\r\n", fft_cycles, fft_execution_time_us);
      break;
    case 16:
      printf("=================================================================\r\n");
      printf("\033[J");
      fflush(stdout);
      print_active = 0;
      print_step = 0;
      return;
    default:
      print_active = 0;
      print_step = 0;
      return;
  }
  print_step++;
}

/* Calibrated Health Scoring Function ----------------------------------------*/
static void Calculate_Health_Score(void)
{
  float current_ma = fabsf(motor_current_A) * 1000.0f;
  pen_i = 0.0f;
  pen_v = 0.0f;
  pen_r = 0.0f;

  /* 1. Speed Error Penalty (Max 30 pts) */
  float abs_speed_err = fabsf(target_rpm - filtered_rpm);
  if (abs_speed_err > 5.0f)
  {
    pen_r = ((abs_speed_err - 5.0f) / 45.0f) * 30.0f;
  }

  /* 2. Armature Current Penalty (Baseline 300 mA, Limit 400 mA, Max 35 pts) */
  if (current_ma > 400.0f)
  {
    pen_i = ((current_ma - 400.0f) / 300.0f) * 35.0f;
  }

  /* 3. Vibration RMS Penalty (Max 35 pts) */
  if (total_vib_rms > 0.03f)
  {
    pen_v = ((total_vib_rms - 0.03f) / 0.07f) * 35.0f;
  }

  float total_penalty = pen_i + pen_v + pen_r;
  if (total_penalty > 100.0f) total_penalty = 100.0f;

  health_score = 100.0f - total_penalty;

  if (health_score >= 60.0f)      current_health_state = HEALTH_NORMAL;
  else if (health_score >= 30.0f) current_health_state = HEALTH_WARNING;
  else                            current_health_state = HEALTH_CRITICAL_FAULT;
}

/* DWT Hardware Cycle Counter Init -------------------------------------------*/
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}

/* TIM7 Periodic Interrupt Handler (200 Hz) ----------------------------------*/
void TIM7_IRQHandler(void)
{
  if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
      if (sample_pending < 4) sample_pending++;
    }
  }
}

/* 170 MHz Safe HD44780 LCD Timing Drivers -----------------------------------*/
static void LCD_Delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < ticks) {}
}

static void LCD_Pulse_Enable(void)
{
  LCD_Delay_us(2);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_SET);
  LCD_Delay_us(3);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);
  LCD_Delay_us(3);
}

static void LCD_Send_4Bits(uint8_t val)
{
  HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (val & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (val & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (val & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (val & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  LCD_Pulse_Enable();
}

static void LCD_Send_Cmd(uint8_t cmd)
{
  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
  LCD_Delay_us(2);
  LCD_Send_4Bits(cmd >> 4);
  LCD_Send_4Bits(cmd & 0x0F);
  if (cmd == 0x01 || cmd == 0x02) LCD_Delay_us(3000);
  else                            LCD_Delay_us(60);
}

static void LCD_Send_Data(uint8_t data)
{
  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);
  LCD_Delay_us(2);
  LCD_Send_4Bits(data >> 4);
  LCD_Send_4Bits(data & 0x0F);
  LCD_Delay_us(50);
}

static void LCD_Init(void)
{
  HAL_Delay(150);

  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);

  LCD_Send_4Bits(0x03); HAL_Delay(10);
  LCD_Send_4Bits(0x03); HAL_Delay(5);
  LCD_Send_4Bits(0x03); HAL_Delay(5);
  LCD_Send_4Bits(0x02); HAL_Delay(5);

  LCD_Send_Cmd(0x28);
  HAL_Delay(5);
  LCD_Send_Cmd(0x08);
  HAL_Delay(5);
  LCD_Send_Cmd(0x01);
  HAL_Delay(5);
  LCD_Send_Cmd(0x06);
  HAL_Delay(5);
  LCD_Send_Cmd(0x0C);
  HAL_Delay(5);
}

static void LCD_SetCursor(uint8_t row, uint8_t col)
{
  uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
  LCD_Send_Cmd(addr);
}

static void LCD_Print(const char *str)
{
  while (*str)
  {
    LCD_Send_Data((uint8_t)*str++);
  }
}

static void LCD_Update_Display(void)
{
  char line1[17];
  char line2[17];

  if (lcd_page == 0)
  {
    const char *tag = (current_health_state == HEALTH_NORMAL) ? "NORM" :
                      (current_health_state == HEALTH_WARNING) ? "WARN" : "CRIT";

    snprintf(line1, sizeof(line1), "RPM:%-4.0f T:%-4.0f ", filtered_rpm, target_rpm);
    snprintf(line2, sizeof(line2), "I:%4.0fmA H:%-2.0f%-4s", fabsf(motor_current_A) * 1000.0f, health_score, tag);
  }
  else
  {
    if (mpu_ok)
    {
      snprintf(line1, sizeof(line1), "RMS:%4.2f PK:%4.2f", total_vib_rms, peak_to_peak_g);
      snprintf(line2, sizeof(line2), "f0:%3.1f  fP:%4.1f ", filtered_rpm / 60.0f, dominant_freq_hz);
    }
    else
    {
      snprintf(line1, sizeof(line1), "RMS:---- PK:----");
      snprintf(line2, sizeof(line2), "MPU OFFLINE     ");
    }
  }

  LCD_SetCursor(0, 0);
  LCD_Print(line1);
  LCD_SetCursor(1, 0);
  LCD_Print(line2);
}

/* Sensor Acquisition Engine -------------------------------------------------*/
static void Acquire_Sensor_Sample(void)
{
  if (!mpu_ok)
  {
    static uint32_t last_retry = 0;
    uint32_t t = HAL_GetTick();
    if ((t - last_retry) >= 500)
    {
      last_retry = t;
      Init_MPU6050();
    }
    return;
  }

  uint8_t raw[6];
  if (MPU6050_Read(active_mpu_addr, MPU6050_ACCEL_XOUT_H, raw, 6))
  {
    int16_t rx = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ry = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t rz = (int16_t)((raw[4] << 8) | raw[5]);

    ax_buf[sample_idx] = (float)rx / 16384.0f;
    ay_buf[sample_idx] = (float)ry / 16384.0f;
    az_buf[sample_idx] = (float)rz / 16384.0f;

    sample_idx++;
    if (sample_idx >= FFT_SAMPLES)
    {
      sample_idx = 0;
      buffer_ready = 1;
    }
  }
  else
  {
    mpu_ok = 0;
  }
}

/* Time-Domain Feature Computation -------------------------------------------*/
static void Process_Vibration_DSP(void)
{
  float mean_x = 0.0f, mean_y = 0.0f, mean_z = 0.0f;
  float min_val = 999.0f, max_val = -999.0f;

  for (int i = 0; i < FFT_SAMPLES; i++)
  {
    mean_x += ax_buf[i];
    mean_y += ay_buf[i];
    mean_z += az_buf[i];
  }
  mean_x /= (float)FFT_SAMPLES;
  mean_y /= (float)FFT_SAMPLES;
  mean_z /= (float)FFT_SAMPLES;

  float sum_sq_x = 0.0f, sum_sq_y = 0.0f, sum_sq_z = 0.0f;
  for (int i = 0; i < FFT_SAMPLES; i++)
  {
    float dx = ax_buf[i] - mean_x;
    float dy = ay_buf[i] - mean_y;
    float dz = az_buf[i] - mean_z;

    sum_sq_x += dx * dx;
    sum_sq_y += dy * dy;
    sum_sq_z += dz * dz;

    float vector_mag = sqrtf(dx * dx + dy * dy + dz * dz);
    if (vector_mag < min_val) min_val = vector_mag;
    if (vector_mag > max_val) max_val = vector_mag;

    float hanning = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(FFT_SAMPLES - 1)));
    fft_real[i] = dz * hanning;
    fft_imag[i] = 0.0f;
  }

  vib_rms_x = sqrtf(sum_sq_x / (float)FFT_SAMPLES);
  vib_rms_y = sqrtf(sum_sq_y / (float)FFT_SAMPLES);
  vib_rms_z = sqrtf(sum_sq_z / (float)FFT_SAMPLES);

  total_vib_rms = sqrtf(vib_rms_x * vib_rms_x + vib_rms_y * vib_rms_y + vib_rms_z * vib_rms_z);
  peak_to_peak_g = (max_val > min_val) ? (max_val - min_val) : 0.0f;
}

/* 256-Point Radix-2 Real FFT ------------------------------------------------*/
static void Execute_Real_FFT(void)
{
  int n = FFT_SAMPLES;
  int j = 0;
  for (int i = 0; i < n - 1; i++)
  {
    if (i < j)
    {
      float tr = fft_real[i]; fft_real[i] = fft_real[j]; fft_real[j] = tr;
      float ti = fft_imag[i]; fft_imag[i] = fft_imag[j]; fft_imag[j] = ti;
    }
    int k = n >> 1;
    while (k <= j)
    {
      j -= k;
      k >>= 1;
    }
    j += k;
  }

  for (int len = 2; len <= n; len <<= 1)
  {
    float angle = -2.0f * (float)M_PI / (float)len;
    float wlen_r = cosf(angle);
    float wlen_i = sinf(angle);

    for (int i = 0; i < n; i += len)
    {
      float w_r = 1.0f;
      float w_i = 0.0f;
      for (int k = 0; k < len / 2; k++)
      {
        float u_r = fft_real[i + k];
        float u_i = fft_imag[i + k];
        float v_r = fft_real[i + k + len / 2] * w_r - fft_imag[i + k + len / 2] * w_i;
        float v_i = fft_real[i + k + len / 2] * w_i + fft_imag[i + k + len / 2] * w_r;

        fft_real[i + k] = u_r + v_r;
        fft_imag[i + k] = u_i + v_i;
        fft_real[i + k + len / 2] = u_r - v_r;
        fft_imag[i + k + len / 2] = u_i - v_i;

        float next_w_r = w_r * wlen_r - w_i * wlen_i;
        float next_w_i = w_r * wlen_i + w_i * wlen_r;
        w_r = next_w_r;
        w_i = next_w_i;
      }
    }
  }

  dominant_mag = 0.0f;
  uint16_t dominant_bin = 1;
  float bin_resolution = SAMPLING_FREQ_HZ / (float)FFT_SAMPLES;

  uint16_t max_bin = (uint16_t)(50.0f / bin_resolution);
  if (max_bin >= (FFT_SAMPLES / 2)) max_bin = (FFT_SAMPLES / 2) - 1;

  for (int i = 1; i <= max_bin; i++)
  {
    fft_mag[i] = sqrtf(fft_real[i] * fft_real[i] + fft_imag[i] * fft_imag[i]);
    if (fft_mag[i] > dominant_mag)
    {
      dominant_mag = fft_mag[i];
      dominant_bin = i;
    }
  }

  dominant_freq_hz = (float)dominant_bin * bin_resolution;
}

/* Speed Control Loop Locked at 150 RPM --------------------------------------*/
static void Update_PI_Controller(void)
{
  uint32_t cyc = DWT->CYCCNT;
  float dt = (float)(cyc - last_pi_cyc) / CPU_HZ;
  last_pi_cyc = cyc;
  if (dt < 0.005f) dt = 0.005f;
  if (dt > 0.200f) dt = 0.200f;
  last_dt_ms = dt * 1000.0f;

  int32_t current_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  int32_t delta_pulses = current_counter - last_encoder_counter;
  last_encoder_counter = current_counter;
  last_delta_pulses = delta_pulses;

  raw_rpm = ((float)abs(delta_pulses) / ENCODER_PPR) * 60.0f / dt;
  filtered_rpm = (0.25f * raw_rpm) + (0.75f * filtered_rpm);

  speed_error = target_rpm - filtered_rpm;
  float P_term = Kp * speed_error;

  /* Conditional Anti-Windup */
  float unsat = P_term + integral_accum;
  uint8_t sat_high = (unsat >= PWM_MAX_LIMIT) && (speed_error > 0.0f);
  uint8_t sat_low  = (unsat <= PWM_MIN_LIMIT) && (speed_error < 0.0f);
  if (!sat_high && !sat_low)
  {
    integral_accum += (Ki * speed_error * dt);
  }

  if (integral_accum > PWM_MAX_LIMIT) integral_accum = PWM_MAX_LIMIT;
  else if (integral_accum < 0.0f)     integral_accum = 0.0f;

  control_output = P_term + integral_accum;

  /* Stall Guard Logic */
  uint32_t now = HAL_GetTick();
  if (!stall_flag)
  {
    if ((control_output >= STALL_PWM_THRESHOLD) && (filtered_rpm < STALL_RPM_THRESHOLD))
    {
      if (!stall_timing)
      {
        stall_timing = 1;
        stall_start_tick = now;
      }
      else if ((now - stall_start_tick) >= STALL_TIME_MS)
      {
        stall_flag = 1;
        integral_accum = 0.0f;
      }
    }
    else
    {
      stall_timing = 0;
    }
  }
  else
  {
    if (control_output > STALL_PWM_LIMIT) control_output = STALL_PWM_LIMIT;
    if (integral_accum > STALL_PWM_LIMIT) integral_accum = STALL_PWM_LIMIT;
    if (filtered_rpm > STALL_CLEAR_RPM)
    {
      stall_flag = 0;
      stall_timing = 0;
    }
  }

  if (control_output > PWM_MAX_LIMIT) control_output = PWM_MAX_LIMIT;
  else if (control_output < PWM_MIN_LIMIT) control_output = PWM_MIN_LIMIT;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)control_output);
}

/* Multi-Sample Oversampled ADC Read -----------------------------------------*/
static float ADC_Read_Voltage_Avg(uint16_t n)
{
  uint32_t sum = 0;
  uint16_t good = 0;

  for (uint16_t i = 0; i < n; i++)
  {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 2) == HAL_OK)
    {
      sum += HAL_ADC_GetValue(&hadc2);
      good++;
    }
    HAL_ADC_Stop(&hadc2);
  }

  if (good == 0) return adc_voltage;
  adc_raw = sum / good;
  return ((float)sum / (float)good) * 3.3f / 4095.0f;
}

/* Settle-and-Calibrate Zero-Current Baseline ---------------------------------*/
/* Measure the true zero-current voltage with the motor PWM forced OFF (Silent / Direct Boot) */
static void Calibrate_Zero_Current(void)
{
  /* Force motor PWM off */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

  /* Background sampling without blocking LCD with text messages */
  float acc = 0.0f;
  for (int k = 0; k < 20; k++)           /* 20 x 256 samples */
  {
    acc += ADC_Read_Voltage_Avg(256);
    HAL_Delay(2);
  }
  zero_current_v  = acc / 20.0f;
  adc_voltage     = zero_current_v;
  motor_current_A = 0.0f;
  integral_accum  = 0.0f;
  control_output  = 0.0f;
}

/* Signed 50 Hz Current Sensing Engine ---------------------------------------*/
static void Read_ACS712_Current(void)
{
  static float current_filt_A = 0.0f;

  adc_voltage = ADC_Read_Voltage_Avg(CURRENT_AVG_SAMPLES);

  float raw_current = CURRENT_SIGN * (adc_voltage - zero_current_v) / ACS712_SENS_V_PER_A;

  current_filt_A  = (0.10f * raw_current) + (0.90f * current_filt_A);
  motor_current_A = (fabsf(current_filt_A) < CURRENT_DEADBAND_A) ? 0.0f : current_filt_A;
}

/* Bit-Bang I2C Functions ----------------------------------------------------*/
static void I2C_Bus_Delay(void)
{
  for (volatile int i = 0; i < 180; i++) __NOP();
}

static inline void SCL_1(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); }
static inline void SCL_0(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); }
static inline void SDA_1(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); }
static inline void SDA_0(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); }
static inline uint8_t SDA_READ(void) { return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) ? 1 : 0; }

static void I2C_Bus_Clear(void)
{
  SDA_1(); SCL_1(); I2C_Bus_Delay();
  for (uint8_t i = 0; i < 9; i++)
  {
    SCL_0(); I2C_Bus_Delay();
    SCL_1(); I2C_Bus_Delay();
  }
  SDA_0(); I2C_Bus_Delay();
  SCL_1(); I2C_Bus_Delay();
  SDA_1(); I2C_Bus_Delay();
}

static void I2C_Start(void)
{
  SDA_1(); SCL_1(); I2C_Bus_Delay();
  SDA_0(); I2C_Bus_Delay();
  SCL_0(); I2C_Bus_Delay();
}

static void I2C_Stop(void)
{
  SDA_0(); SCL_0(); I2C_Bus_Delay();
  SCL_1(); I2C_Bus_Delay();
  SDA_1(); I2C_Bus_Delay();
}

static uint8_t I2C_WriteByte(uint8_t byte)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (byte & 0x80) SDA_1(); else SDA_0();
    byte <<= 1;
    I2C_Bus_Delay();
    SCL_1(); I2C_Bus_Delay();
    SCL_0(); I2C_Bus_Delay();
  }
  SDA_1(); I2C_Bus_Delay();
  SCL_1(); I2C_Bus_Delay();
  uint8_t ack = (SDA_READ() == 0) ? 1 : 0;
  SCL_0(); I2C_Bus_Delay();
  return ack;
}

static uint8_t I2C_ReadByte(uint8_t send_ack)
{
  uint8_t byte = 0;
  SDA_1(); I2C_Bus_Delay();
  for (uint8_t i = 0; i < 8; i++)
  {
    byte <<= 1;
    SCL_1(); I2C_Bus_Delay();
    if (SDA_READ()) byte |= 0x01;
    SCL_0(); I2C_Bus_Delay();
  }
  if (send_ack) SDA_0(); else SDA_1();
  I2C_Bus_Delay();
  SCL_1(); I2C_Bus_Delay();
  SCL_0(); I2C_Bus_Delay();
  SDA_1(); I2C_Bus_Delay();
  return byte;
}

static uint8_t MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t val)
{
  I2C_Start();
  if (!I2C_WriteByte(addr << 1)) { I2C_Stop(); return 0; }
  if (!I2C_WriteByte(reg))       { I2C_Stop(); return 0; }
  if (!I2C_WriteByte(val))       { I2C_Stop(); return 0; }
  I2C_Stop();
  return 1;
}

static uint8_t MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
  I2C_Start();
  if (!I2C_WriteByte(addr << 1)) { I2C_Stop(); return 0; }
  if (!I2C_WriteByte(reg))       { I2C_Stop(); return 0; }
  I2C_Start();
  if (!I2C_WriteByte((addr << 1) | 0x01)) { I2C_Stop(); return 0; }
  for (uint8_t i = 0; i < len; i++)
  {
    buf[i] = I2C_ReadByte((i == len - 1) ? 0 : 1);
  }
  I2C_Stop();
  return 1;
}

static void Init_MPU6050(void)
{
  uint8_t whoami = 0;
  uint8_t addrs[2] = {0x68, 0x69};

  for (uint8_t i = 0; i < 2; i++)
  {
    uint8_t candidate = addrs[i];
    if (MPU6050_Read(candidate, 0x75, &whoami, 1))
    {
      active_mpu_addr = candidate;
      MPU6050_Write(active_mpu_addr, MPU6050_PWR_MGMT_1, 0x00);
      HAL_Delay(10);
      MPU6050_Write(active_mpu_addr, MPU6050_ACCEL_CONFIG, 0x00);
      HAL_Delay(5);
      MPU6050_Write(active_mpu_addr, MPU6050_CONFIG, 0x03);
      HAL_Delay(5);
      mpu_ok = 1;
      return;
    }
  }
  mpu_ok = 0;
}

/* Peripheral Initialization -------------------------------------------------*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_TIM7_Init(void)
{
  __HAL_RCC_TIM7_CLK_ENABLE();
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1699;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK) Error_Handler();

  HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Filter = 6;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim3);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim4);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* User Button Callback */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
    recalibrate_request = 1;
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
