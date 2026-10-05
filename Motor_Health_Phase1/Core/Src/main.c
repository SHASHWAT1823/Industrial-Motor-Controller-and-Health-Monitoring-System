/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Phase 7 - Monolithic Edge DSP & Vibration Engine
  *                   Cooley-Tukey Radix-2/4 FFT, Hanning Window, Vector RMS,
  *                   Closed-Loop PI Control & 5-Second Auto-Rotating 16x2 LCD
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
#define CONTROL_DT_MS           10
#define CONTROL_DT_S            0.010f
#define PWM_MAX_LIMIT           999.0f
#define PWM_MIN_LIMIT           0.0f

#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B

/* DSP & FFT Configuration (Radix-2 Power of 2) */
#define FFT_SIZE                64
#define SAMPLING_FREQ_HZ        100.0f  /* 10 ms interval = 100 Hz */
#ifndef M_PI
#define M_PI                    3.14159265358979323846f
#endif

/* LCD GPIO Pin Mapping (PC0:RS, PC1:EN, PC2:D4, PC3:D5, PB0:D6, PB1:D7) */
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

/* Complex Data Structure for Cooley-Tukey FFT */
typedef struct {
    float real;
    float imag;
} Complex_t;

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Speed Control Loop Variables */
float Kp = 1.2f;
float Ki = 3.0f;
float target_rpm = 150.0f;
float raw_rpm = 0.0f;
float filtered_rpm = 0.0f;
float speed_error = 0.0f;
float integral_accum = 0.0f;
float control_output = 0.0f;

volatile int32_t last_encoder_counter = 0;
uint32_t last_control_tick = 0;
uint32_t last_sample_tick = 0;
uint32_t last_telemetry_tick = 0;

/* Current Sensor Variables */
uint32_t adc_raw = 0;
float adc_voltage = 0.0f;
float motor_current_A = 0.0f;
float zero_current_v = 2.515f;

/* MPU6050 & Vibration Variables */
uint8_t mpu_ok = 0;
uint8_t active_mpu_addr = 0x68;
float ax_buf[FFT_SIZE];
float ay_buf[FFT_SIZE];
float az_buf[FFT_SIZE];
uint8_t vib_idx = 0;

/* In-Source DSP Engine Variables */
Complex_t fft_buffer[FFT_SIZE];
float fft_mag[FFT_SIZE / 2];
float hanning_window[FFT_SIZE];

float vib_rms_x = 0.0f;
float vib_rms_y = 0.0f;
float vib_rms_z = 0.0f;
float total_vib_rms = 0.0f;
float peak_to_peak_g = 0.0f;
float dominant_freq_hz = 0.0f;
float dominant_peak_mag = 0.0f;

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

static void Update_PI_Controller(void);
static void Read_ACS712_Current(void);
static void Read_Vibration_Sample(void);

/* Monolithic DSP Math Engine */
static void Init_Hanning_Window(void);
static void Fast_CooleyTukey_FFT(Complex_t *data, uint16_t n);
static void Compute_Vibration_Edge_DSP(void);

/* HD44780 4-bit Parallel Driver */
static void LCD_Delay_us(uint32_t us);
static void LCD_Pulse_Enable(void);
static void LCD_Send_4Bits(uint8_t val);
static void LCD_Send_Cmd(uint8_t cmd);
static void LCD_Send_Data(uint8_t data);
static void LCD_Init(void);
static void LCD_SetCursor(uint8_t row, uint8_t col);
static void LCD_Print(const char *str);
static void LCD_Update_Dashboard(void);

/* Bit-Bang I2C Engine (PB8: SCL, PB9: SDA) */
static void I2C_Bus_Delay(void);
static void I2C_Bus_Clear(void);
static void I2C_Start(void);
static void I2C_Stop(void);
static uint8_t I2C_WriteByte(uint8_t byte);
static uint8_t I2C_ReadByte(uint8_t send_ack);
static uint8_t MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t val);
static uint8_t MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
static void Init_MPU6050(void);

/* Application Entry Point ---------------------------------------------------*/
int main(void)
{
  /* Hardware FPU Activation (CP10 and CP11 Full Access) */
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
  #endif

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* BTS7960 Alternate Function Configuration */
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

  /* Motor Driver Enablers Driven HIGH */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  /* LCD Bring-Up */
  LCD_Init();
  LCD_SetCursor(0, 0);
  LCD_Print("PHASE 7 EDGE DSP");
  LCD_SetCursor(1, 0);
  LCD_Print("FFT READY (M4)  ");
  HAL_Delay(1000);
  LCD_Send_Cmd(0x01);
  HAL_Delay(5);

  Init_Hanning_Window();

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

  BSP_LED_On(LED_GREEN);
  printf("\033[2J\033[H");
  fflush(stdout);

  last_encoder_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  last_control_tick = HAL_GetTick();
  last_sample_tick = HAL_GetTick();
  last_telemetry_tick = HAL_GetTick();

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* 100 Hz MPU6050 Acceleration Sampling (10 ms) */
    if ((now - last_sample_tick) >= 10)
    {
      last_sample_tick = now;
      Read_Vibration_Sample();
    }

    /* 100 Hz PI Speed Control Loop (10 ms) */
    if ((now - last_control_tick) >= CONTROL_DT_MS)
    {
      last_control_tick = now;
      Update_PI_Controller();
    }

    /* 4 Hz Display & Telemetry Refresh (250 ms) */
    if ((now - last_telemetry_tick) >= 250)
    {
      last_telemetry_tick = now;

      Read_ACS712_Current();
      Compute_Vibration_Edge_DSP();
      LCD_Update_Dashboard();

      printf("\033[H");
      printf("====================================================================\r\n");
      printf("      PHASE 7: DIRECT HARDWARE FPU REAL FFT & EDGE TELEMETRY       \r\n");
      printf("====================================================================\r\n");
      printf(" TARGET SPEED : %5.1f RPM   | ACTUAL SPEED : %5.1f RPM\r\n", target_rpm, filtered_rpm);
      printf(" PWM DRIVE    : %4lu / 999   | ARMATURE CURR: %+6.1f mA\r\n",
              (uint32_t)control_output, motor_current_A * 1000.0f);
      printf("--------------------------------------------------------------------\r\n");
      if (mpu_ok)
      {
        printf(" MPU6050      : ONLINE (0x%02X)\r\n", active_mpu_addr);
        printf(" 3-AXIS RMS(g): X: %5.3f | Y: %5.3f | Z: %5.3f\r\n", vib_rms_x, vib_rms_y, vib_rms_z);
        printf(" TOTAL RMS    : %5.3f g   | PEAK-TO-PEAK : %5.3f g\r\n", total_vib_rms, peak_to_peak_g);
        printf(" DOMINANT FFT : %5.2f Hz  | HARMONIC MAG : %5.3f\r\n", dominant_freq_hz, dominant_peak_mag);
        printf(" SHAFT FUND.  : %5.2f Hz  (1xRPM expected)\r\n", filtered_rpm / 60.0f);
      }
      else
      {
        printf(" MPU6050      : OFFLINE / BUS FAULT\r\n");
      }
      printf("====================================================================\r\n");
      printf("\033[J");
      fflush(stdout);
    }
  }
}

/* In-Source DSP Engine (Hanning, Cooley-Tukey Radix-2 FFT) -------------------*/
static void Init_Hanning_Window(void)
{
  for (int i = 0; i < FFT_SIZE; i++)
  {
    hanning_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(FFT_SIZE - 1)));
  }
}

static void Fast_CooleyTukey_FFT(Complex_t *data, uint16_t n)
{
  /* Bit-Reversal Permutation */
  uint16_t j = 0;
  for (uint16_t i = 0; i < n - 1; i++)
  {
    if (i < j)
    {
      Complex_t temp = data[i];
      data[i] = data[j];
      data[j] = temp;
    }
    uint16_t k = n >> 1;
    while (k <= j)
    {
      j -= k;
      k >>= 1;
    }
    j += k;
  }

  /* Butterfly Computations */
  for (uint16_t len = 2; len <= n; len <<= 1)
  {
    float angle = -2.0f * (float)M_PI / (float)len;
    Complex_t wlen = { cosf(angle), sinf(angle) };

    for (uint16_t i = 0; i < n; i += len)
    {
      Complex_t w = { 1.0f, 0.0f };
      for (uint16_t k = 0; k < len / 2; k++)
      {
        Complex_t u = data[i + k];
        Complex_t v = {
          data[i + k + len / 2].real * w.real - data[i + k + len / 2].imag * w.imag,
          data[i + k + len / 2].real * w.imag + data[i + k + len / 2].imag * w.real
        };

        data[i + k].real = u.real + v.real;
        data[i + k].imag = u.imag + v.imag;
        data[i + k + len / 2].real = u.real - v.real;
        data[i + k + len / 2].imag = u.imag - v.imag;

        float next_w_real = w.real * wlen.real - w.imag * wlen.imag;
        w.imag = w.real * wlen.imag + w.imag * wlen.real;
        w.real = next_w_real;
      }
    }
  }
}

static void Compute_Vibration_Edge_DSP(void)
{
  float mean_x = 0.0f, mean_y = 0.0f, mean_z = 0.0f;
  float sum_sq_x = 0.0f, sum_sq_y = 0.0f, sum_sq_z = 0.0f;
  float max_val = -999.0f, min_val = 999.0f;

  /* 1. Time-Domain Vector Math */
  for (int i = 0; i < FFT_SIZE; i++)
  {
    mean_x += ax_buf[i];
    mean_y += ay_buf[i];
    mean_z += az_buf[i];
  }
  mean_x /= FFT_SIZE;
  mean_y /= FFT_SIZE;
  mean_z /= FFT_SIZE;

  for (int i = 0; i < FFT_SIZE; i++)
  {
    float dx = ax_buf[i] - mean_x;
    float dy = ay_buf[i] - mean_y;
    float dz = az_buf[i] - mean_z;

    sum_sq_x += dx * dx;
    sum_sq_y += dy * dy;
    sum_sq_z += dz * dz;

    if (dz > max_val) max_val = dz;
    if (dz < min_val) min_val = dz;

    /* Windowing dynamic AC vibration on Z-Axis */
    fft_buffer[i].real = dz * hanning_window[i];
    fft_buffer[i].imag = 0.0f;
  }

  vib_rms_x = sqrtf(sum_sq_x / FFT_SIZE);
  vib_rms_y = sqrtf(sum_sq_y / FFT_SIZE);
  vib_rms_z = sqrtf(sum_sq_z / FFT_SIZE);
  total_vib_rms = sqrtf(vib_rms_x * vib_rms_x + vib_rms_y * vib_rms_y + vib_rms_z * vib_rms_z);
  peak_to_peak_g = (max_val > min_val) ? (max_val - min_val) : 0.0f;

  /* 2. Cooley-Tukey Real FFT Execution */
  Fast_CooleyTukey_FFT(fft_buffer, FFT_SIZE);

  /* Calculate Magnitudes (Single-Sided Spectrum) */
  fft_mag[0] = 0.0f; /* Ignore DC offset */
  dominant_peak_mag = 0.0f;
  uint16_t dominant_bin = 1;

  for (uint16_t i = 1; i < (FFT_SIZE / 2); i++)
  {
    fft_mag[i] = sqrtf(fft_buffer[i].real * fft_buffer[i].real + fft_buffer[i].imag * fft_buffer[i].imag);
    if (fft_mag[i] > dominant_peak_mag)
    {
      dominant_peak_mag = fft_mag[i];
      dominant_bin = i;
    }
  }

  /* Frequency Resolution = Fs / N */
  dominant_freq_hz = (float)dominant_bin * (SAMPLING_FREQ_HZ / (float)FFT_SIZE);
}

/* Control & Sensing Logic ---------------------------------------------------*/
static void Update_PI_Controller(void)
{
  int32_t current_counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  int32_t delta_pulses = current_counter - last_encoder_counter;
  last_encoder_counter = current_counter;

  raw_rpm = ((float)abs(delta_pulses) * 6000.0f) / ENCODER_PPR;
  filtered_rpm = (0.30f * raw_rpm) + (0.70f * filtered_rpm);

  speed_error = target_rpm - filtered_rpm;
  float P_term = Kp * speed_error;

  integral_accum += (Ki * speed_error * CONTROL_DT_S);

  if (integral_accum > PWM_MAX_LIMIT) integral_accum = PWM_MAX_LIMIT;
  else if (integral_accum < 0.0f)     integral_accum = 0.0f;

  control_output = P_term + integral_accum;

  if (control_output > PWM_MAX_LIMIT) control_output = PWM_MAX_LIMIT;
  else if (control_output < PWM_MIN_LIMIT) control_output = PWM_MIN_LIMIT;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)control_output);
}

static void Read_ACS712_Current(void)
{
  uint32_t adc_sum = 0;
  const uint16_t SAMPLES = 64;

  for (uint16_t i = 0; i < SAMPLES; i++)
  {
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 2) == HAL_OK)
    {
      adc_sum += HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);
  }

  adc_raw = adc_sum / SAMPLES;
  adc_voltage = ((float)adc_raw * 3.3f) / 4095.0f;

  float raw_current = fabsf(adc_voltage - zero_current_v) / 0.185f;
  motor_current_A = (0.15f * raw_current) + (0.85f * motor_current_A);

  if (motor_current_A < 0.003f)
  {
    motor_current_A = 0.0f;
  }
}

static void Read_Vibration_Sample(void)
{
  if (!mpu_ok)
  {
    Init_MPU6050();
    return;
  }

  uint8_t raw[6];
  if (MPU6050_Read(active_mpu_addr, MPU6050_ACCEL_XOUT_H, raw, 6))
  {
    int16_t raw_x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_y = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_z = (int16_t)((raw[4] << 8) | raw[5]);

    ax_buf[vib_idx] = (float)raw_x / 16384.0f;
    ay_buf[vib_idx] = (float)raw_y / 16384.0f;
    az_buf[vib_idx] = (float)raw_z / 16384.0f;

    vib_idx = (vib_idx + 1) % FFT_SIZE;
  }
  else
  {
    mpu_ok = 0;
  }
}

/* Auto-Rotating 5-Second Paged LCD Dashboard --------------------------------*/
static void LCD_Delay_us(uint32_t us)
{
  uint32_t count = us * 17;
  for (volatile uint32_t i = 0; i < count; i++)
  {
    __NOP();
  }
}

static void LCD_Pulse_Enable(void)
{
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_SET);
  LCD_Delay_us(5);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);
  LCD_Delay_us(50);
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
  LCD_Send_4Bits(cmd >> 4);
  LCD_Send_4Bits(cmd & 0x0F);
  if (cmd == 0x01 || cmd == 0x02)
  {
    HAL_Delay(2);
  }
}

static void LCD_Send_Data(uint8_t data)
{
  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);
  LCD_Send_4Bits(data >> 4);
  LCD_Send_4Bits(data & 0x0F);
}

static void LCD_Init(void)
{
  HAL_Delay(50);

  HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_EN_PORT, LCD_EN_PIN, GPIO_PIN_RESET);

  LCD_Send_4Bits(0x03);
  HAL_Delay(5);
  LCD_Send_4Bits(0x03);
  LCD_Delay_us(150);
  LCD_Send_4Bits(0x03);
  LCD_Delay_us(150);
  LCD_Send_4Bits(0x02);
  LCD_Delay_us(150);

  LCD_Send_Cmd(0x28);
  LCD_Send_Cmd(0x08);
  LCD_Send_Cmd(0x01);
  HAL_Delay(3);
  LCD_Send_Cmd(0x06);
  LCD_Send_Cmd(0x0C);
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

static void LCD_Update_Dashboard(void)
{
  static uint8_t lcd_page = 0;
  static uint32_t last_page_swap_tick = 0;
  char line_buf[17];
  uint32_t current_tick = HAL_GetTick();

  /* 5-second automatic page flip */
  if ((current_tick - last_page_swap_tick) >= 5000)
  {
    last_page_swap_tick = current_tick;
    lcd_page ^= 1;
    LCD_Send_Cmd(0x01);
    HAL_Delay(2);
  }

  if (lcd_page == 0)
  {
    /* Page 0: Speed & Electrical Parameters */
    snprintf(line_buf, sizeof(line_buf), "RPM:%5.1f T:%3.0f", filtered_rpm, target_rpm);
    LCD_SetCursor(0, 0);
    LCD_Print(line_buf);

    snprintf(line_buf, sizeof(line_buf), "CURR:  %4d mA", (int)(motor_current_A * 1000.0f));
    LCD_SetCursor(1, 0);
    LCD_Print(line_buf);
  }
  else
  {
    /* Page 1: Vibration Metrics & Dominant FFT Harmonic */
    if (mpu_ok)
    {
      snprintf(line_buf, sizeof(line_buf), "VIB RMS:%5.3fg", total_vib_rms);
      LCD_SetCursor(0, 0);
      LCD_Print(line_buf);

      snprintf(line_buf, sizeof(line_buf), "DOM FFT:%4.1fHz", dominant_freq_hz);
      LCD_SetCursor(1, 0);
      LCD_Print(line_buf);
    }
    else
    {
      LCD_SetCursor(0, 0);
      LCD_Print("VIBRATION SENSOR");
      LCD_SetCursor(1, 0);
      LCD_Print("MPU6050 OFFLINE ");
    }
  }
}

/* Bit-Bang I2C Engine -------------------------------------------------------*/
static void I2C_Bus_Delay(void)
{
  for (volatile int i = 0; i < 180; i++)
  {
    __NOP();
  }
}

static inline void SCL_1(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); }
static inline void SCL_0(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); }
static inline void SDA_1(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); }
static inline void SDA_0(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); }
static inline uint8_t SDA_READ(void) { return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) ? 1 : 0; }

static void I2C_Bus_Clear(void)
{
  SDA_1();
  SCL_1();
  I2C_Bus_Delay();

  for (uint8_t i = 0; i < 9; i++)
  {
    SCL_0();
    I2C_Bus_Delay();
    SCL_1();
    I2C_Bus_Delay();
  }

  SDA_0();
  I2C_Bus_Delay();
  SCL_1();
  I2C_Bus_Delay();
  SDA_1();
  I2C_Bus_Delay();
}

static void I2C_Start(void)
{
  SDA_1();
  SCL_1();
  I2C_Bus_Delay();
  SDA_0();
  I2C_Bus_Delay();
  SCL_0();
  I2C_Bus_Delay();
}

static void I2C_Stop(void)
{
  SDA_0();
  SCL_0();
  I2C_Bus_Delay();
  SCL_1();
  I2C_Bus_Delay();
  SDA_1();
  I2C_Bus_Delay();
}

static uint8_t I2C_WriteByte(uint8_t byte)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (byte & 0x80) SDA_1();
    else             SDA_0();
    byte <<= 1;
    I2C_Bus_Delay();
    SCL_1();
    I2C_Bus_Delay();
    SCL_0();
    I2C_Bus_Delay();
  }

  SDA_1();
  I2C_Bus_Delay();
  SCL_1();
  I2C_Bus_Delay();
  uint8_t ack = (SDA_READ() == 0) ? 1 : 0;
  SCL_0();
  I2C_Bus_Delay();
  return ack;
}

static uint8_t I2C_ReadByte(uint8_t send_ack)
{
  uint8_t byte = 0;
  SDA_1();
  I2C_Bus_Delay();

  for (uint8_t i = 0; i < 8; i++)
  {
    byte <<= 1;
    SCL_1();
    I2C_Bus_Delay();
    if (SDA_READ()) byte |= 0x01;
    SCL_0();
    I2C_Bus_Delay();
  }

  if (send_ack) SDA_0();
  else          SDA_1();
  I2C_Bus_Delay();
  SCL_1();
  I2C_Bus_Delay();
  SCL_0();
  I2C_Bus_Delay();
  SDA_1();
  I2C_Bus_Delay();

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
      MPU6050_Write(active_mpu_addr, 0x1C, 0x00);
      mpu_ok = 1;
      return;
    }
  }
  mpu_ok = 0;
}

/* System Clock & Peripherals ------------------------------------------------*/
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;

  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* BTS7960 Enable Pins (PC8, PC9) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* LCD Control and Low Data Pins (PC0:RS, PC1:EN, PC2:D4, PC3:D5) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* LCD High Data Pins (PB0:D6, PB1:D7) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* MPU6050 Bit-Bang I2C Pins (PB8:SCL, PB9:SDA) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
