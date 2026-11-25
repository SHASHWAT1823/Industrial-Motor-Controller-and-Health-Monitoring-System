/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Industrial Motor Health Monitoring & Control (Phase 10 Final)
  *                   - Deterministic Time-Sliced Task Architecture
  *                   - Closed-Loop 20 Hz PI Speed Control with Anti-Windup
  *                   - Signed ACS712 Current Sensing + 200 Hz MPU6050 FFT
  *                   - Composite 0-100 Health Engine & Flash Page 255 Black-Box
  *                   - Hardware IWDG Supervisor (500 ms) + Safe-State Recovery
  *                   - Non-Blocking Interactive UART CLI (115200 Baud)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define ENCODER_PPR             334.0f
#define CONTROL_DT_MS           50       /* 20 Hz PI Speed Control Task */
#define PWM_MAX_LIMIT           999.0f
#define PWM_MIN_LIMIT           0.0f
#define CPU_HZ                  170000000.0f

#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_CONFIG          0x1A
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B

#define FFT_SAMPLES             256
#define SAMPLING_FREQ_HZ        200.0f

/* ACS712 Current Sensing Settings */
#define ACS712_SENS_V_PER_A     0.185f   /* 5A module = 185 mV/A */
#define CURRENT_SIGN            1.0f
#define CURRENT_READ_MS         20       /* 50 Hz Current Sensing Task */
#define CURRENT_AVG_SAMPLES     256
#define CURRENT_DEADBAND_A      0.008f

/* Safety Protection Thresholds */
#define TRIP_OVERCURRENT_A      1.20f    /* 1200 mA hard cutoff */
#define TRIP_HIGH_VIB_G         0.25f    /* 0.25 g severe vibration cutoff */

/* Internal Flash Black-Box Storage (STM32G491RE Page 255: 0x0807F800) */
#define FLASH_LOG_PAGE          255U
#define FLASH_LOG_ADDR          0x0807F800UL
#define FLASH_LOG_MAGIC         0xFA17B00CUL

/* Task Supervisor Heartbeat Flags (for Hardware IWDG) */
#define HB_TASK_CONTROL         (1U << 0)
#define HB_TASK_CURRENT         (1U << 1)
#define HB_TASK_TELEMETRY       (1U << 2)
#define HB_ALL_TASKS            (HB_TASK_CONTROL | HB_TASK_CURRENT | HB_TASK_TELEMETRY)

/* UART CLI Buffer */
#define CLI_BUF_SIZE            64

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

typedef enum {
    FAULT_NONE = 0x00,
    FAULT_ROTOR_STALL = 0x01,
    FAULT_OVERCURRENT = 0x02,
    FAULT_HIGH_VIBRATION = 0x03,
    FAULT_WATCHDOG_RESET = 0x04
} FaultCode_t;

/* 16-Byte Aligned Non-Volatile Fault Record */
typedef struct __attribute__((packed, aligned(8))) {
    uint32_t magic;            /* 0xFA17B00C */
    uint32_t timestamp_ms;     /* HAL_GetTick() at trip */
    uint16_t trip_count;       /* Cumulative non-volatile trip counter */
    uint8_t  fault_code;       /* FaultCode_t */
    uint8_t  health_score;     /* Health score (0-100) at trip */
    int16_t  rpm_at_trip;      /* Measured RPM x10 */
    uint16_t current_ma;       /* Armature current in mA */
} FaultLogRecord_t;

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
IWDG_HandleTypeDef hiwdg;

/* Speed Control Loop Variables */
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

/* Safety Trip & Watchdog Supervisor State */
uint8_t  fault_latched = 0;
FaultCode_t active_fault = FAULT_NONE;
uint32_t trip_latency_us = 0;
uint8_t  task_heartbeat_mask = 0;
uint32_t iwdg_kick_count = 0;
uint8_t  wdg_recovery_boot = 0;

/* Stored Black-Box Record in RAM */
FaultLogRecord_t last_saved_fault;
uint8_t has_saved_fault = 0;

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

/* Diagnostics & Scoring */
float pen_i = 0.0f;
float pen_v = 0.0f;
float pen_r = 0.0f;
float health_score = 100.0f;
HealthState_t current_health_state = HEALTH_NORMAL;

uint8_t lcd_page = 0;
static uint8_t print_active = 0;
static uint8_t print_step = 0;
static uint8_t dashboard_paused = 0;

/* UART CLI State */
static char cli_rx_buf[CLI_BUF_SIZE];
static uint8_t cli_rx_idx = 0;
static char cli_last_reply[64] = "READY (TYPE 'HELP' + ENTER)";

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_IWDG_Init(void);

static void DWT_Init(void);
static void Trigger_Emergency_Cutoff(FaultCode_t code, uint32_t detect_start_cyc);
static void Flash_Load_Last_Fault(void);
static void Flash_Save_Fault_Record(const FaultLogRecord_t *rec);
static const char* Fault_Code_To_String(uint8_t code);
static void System_Perform_Reset_Rearm(void);

static void Task_CLI_Poll(void);
static void CLI_Execute_Command(char *cmd);
static void Task_Supervisor_IWDG(void);

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
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
  #endif

  /* Check if MCU rebooted due to Hardware Independent Watchdog (IWDG) */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    wdg_recovery_boot = 1;
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }

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

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  /* Load previous Black-Box Fault Record from Internal Flash Page 255 */
  Flash_Load_Last_Fault();

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

  /* Safe-State Recovery: If booted from Watchdog Reset, latch safe stop */
  if (wdg_recovery_boot)
  {
    uint32_t t_det = DWT->CYCCNT;
    Trigger_Emergency_Cutoff(FAULT_WATCHDOG_RESET, t_det);
    snprintf(cli_last_reply, sizeof(cli_last_reply), "SAFE RECOVERY: IWDG RESET LATCHED");
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  }

  LCD_Update_Display();

  BSP_LED_On(LED_GREEN);
  printf("\033[2J\033[H");
  fflush(stdout);

  /* Start 200 Hz Sampling Timer */
  HAL_TIM_Base_Start_IT(&htim7);

  /* Enable Hardware Independent Watchdog (IWDG: 500 ms Supervisor) */
  MX_IWDG_Init();

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
    /* Poll UART CLI RX (Non-blocking) */
    Task_CLI_Poll();

    /* Blue User Button (B1): Clear Latched Fault & Re-Zero Current */
    if (recalibrate_request)
    {
      recalibrate_request = 0;
      System_Perform_Reset_Rearm();
      snprintf(cli_last_reply, sizeof(cli_last_reply), "B1 PRESSED: SYSTEM RE-ARMED & ZEROED");
    }

    uint32_t now = HAL_GetTick();

    /* Task 1: 200 Hz IMU Sensor Ingestion */
    if (sample_pending)
    {
      __disable_irq();
      sample_pending--;
      __enable_irq();
      Acquire_Sensor_Sample();
    }

    /* Task 2: 20 Hz (50 ms) Closed-Loop PI Speed Controller */
    if ((now - last_control_tick) >= CONTROL_DT_MS)
    {
      last_control_tick = now;
      Update_PI_Controller();
      task_heartbeat_mask |= HB_TASK_CONTROL;
    }

    /* Task 3: 50 Hz (20 ms) ACS712 Current Sensing & Overcurrent Guard */
    if ((now - last_current_tick) >= CURRENT_READ_MS)
    {
      last_current_tick = now;
      Read_ACS712_Current();

      float inst_curr = fabsf(CURRENT_SIGN * (adc_voltage - zero_current_v) / ACS712_SENS_V_PER_A);
      if (!fault_latched && ((fabsf(motor_current_A) >= TRIP_OVERCURRENT_A) || (inst_curr >= TRIP_OVERCURRENT_A)))
      {
        uint32_t t_det = DWT->CYCCNT;
        Trigger_Emergency_Cutoff(FAULT_OVERCURRENT, t_det);
      }
      task_heartbeat_mask |= HB_TASK_CURRENT;
    }

    /* Task 4: Edge CMSIS-DSP Vibration RMS & 256-Point Real FFT */
    if (buffer_ready)
    {
      buffer_ready = 0;
      Process_Vibration_DSP();

      if (!fault_latched && (filtered_rpm > 20.0f) && (total_vib_rms >= TRIP_HIGH_VIB_G))
      {
        uint32_t t_det = DWT->CYCCNT;
        Trigger_Emergency_Cutoff(FAULT_HIGH_VIBRATION, t_det);
      }

      uint32_t c_start = DWT->CYCCNT;
      Execute_Real_FFT();
      uint32_t c_end = DWT->CYCCNT;

      fft_cycles = c_end - c_start;
      fft_execution_time_us = ((float)fft_cycles / 170.0f);
    }

    /* Task 5: 5 Hz Health Scoring, LCD & Serial Telemetry */
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

      if (!print_active && !dashboard_paused)
      {
        print_active = 1;
        print_step = 0;
      }
      task_heartbeat_mask |= HB_TASK_TELEMETRY;
    }

    /* Task 6: Hardware Watchdog (IWDG) Supervisor */
    Task_Supervisor_IWDG();

    /* Non-blocking UART dashboard output */
    if (print_active && !dashboard_paused)
    {
      Dashboard_Print_Step();
    }
  }
}

/* Task 6: Hardware IWDG Supervisor (Kicks only when all tasks check in) ----*/
static void Task_Supervisor_IWDG(void)
{
  if ((task_heartbeat_mask & HB_ALL_TASKS) == HB_ALL_TASKS)
  {
    task_heartbeat_mask = 0;
    HAL_IWDG_Refresh(&hiwdg);
    iwdg_kick_count++;
  }
}

/* Re-arm & Zero Calibration Helper -----------------------------------------*/
static void System_Perform_Reset_Rearm(void)
{
  fault_latched = 0;
  active_fault = FAULT_NONE;
  trip_latency_us = 0;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  Calibrate_Zero_Current();

  filtered_rpm = 0.0f;
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

/* Non-Blocking UART CLI Polling & Command Parser ----------------------------*/
static void Task_CLI_Poll(void)
{
  uint8_t ch = 0;
  if (HAL_UART_Receive(&hcom_uart[COM1], &ch, 1, 0) == HAL_OK)
  {
    if (ch == '\r' || ch == '\n')
    {
      if (cli_rx_idx > 0)
      {
        cli_rx_buf[cli_rx_idx] = '\0';
        CLI_Execute_Command(cli_rx_buf);
        cli_rx_idx = 0;
      }
    }
    else if (ch == '\b' || ch == 127)
    {
      if (cli_rx_idx > 0) cli_rx_idx--;
    }
    else if (isprint((int)ch) && (cli_rx_idx < (CLI_BUF_SIZE - 1)))
    {
      cli_rx_buf[cli_rx_idx++] = (char)toupper((int)ch);
    }
  }
}

static void CLI_Execute_Command(char *cmd)
{
  while (*cmd == ' ') cmd++;

  if (strncmp(cmd, "HELP", 4) == 0)
  {
    snprintf(cli_last_reply, sizeof(cli_last_reply),
             "CMDS: SET RPM <0-350> | SET KP <v> | SET KI <v> | RESET");
  }
  else if (strncmp(cmd, "SET RPM ", 8) == 0)
  {
    float val = strtof(cmd + 8, NULL);
    if (val >= 0.0f && val <= 350.0f)
    {
      target_rpm = val;
      snprintf(cli_last_reply, sizeof(cli_last_reply), "OK: TARGET_RPM = %.1f RPM", target_rpm);
    }
    else
    {
      snprintf(cli_last_reply, sizeof(cli_last_reply), "ERR: RPM MUST BE 0..350");
    }
  }
  else if (strncmp(cmd, "SET KP ", 7) == 0)
  {
    float val = strtof(cmd + 7, NULL);
    if (val >= 0.0f && val <= 20.0f)
    {
      Kp = val;
      snprintf(cli_last_reply, sizeof(cli_last_reply), "OK: Kp = %.3f", Kp);
    }
    else
    {
      snprintf(cli_last_reply, sizeof(cli_last_reply), "ERR: Kp OUT OF RANGE (0..20)");
    }
  }
  else if (strncmp(cmd, "SET KI ", 7) == 0)
  {
    float val = strtof(cmd + 7, NULL);
    if (val >= 0.0f && val <= 50.0f)
    {
      Ki = val;
      snprintf(cli_last_reply, sizeof(cli_last_reply), "OK: Ki = %.3f", Ki);
    }
    else
    {
      snprintf(cli_last_reply, sizeof(cli_last_reply), "ERR: Ki OUT OF RANGE (0..50)");
    }
  }
  else if (strncmp(cmd, "RESET", 5) == 0)
  {
    System_Perform_Reset_Rearm();
    snprintf(cli_last_reply, sizeof(cli_last_reply), "OK: FAULT CLEARED & RE-ZEROED");
  }
  else if (strncmp(cmd, "PAUSE", 5) == 0)
  {
    dashboard_paused = 1;
    printf("\r\n[CLI] Dashboard Paused. Type RESUME + Enter to continue.\r\n> ");
    fflush(stdout);
  }
  else if (strncmp(cmd, "RESUME", 6) == 0)
  {
    dashboard_paused = 0;
    printf("\033[2J\033[H");
    fflush(stdout);
    snprintf(cli_last_reply, sizeof(cli_last_reply), "OK: DASHBOARD RESUMED");
  }
  else if (strncmp(cmd, "FAULTLOG", 8) == 0)
  {
    if (has_saved_fault)
      snprintf(cli_last_reply, sizeof(cli_last_reply),
               "LOG #%u: %s (%u mA @ %.1f RPM)",
               last_saved_fault.trip_count,
               Fault_Code_To_String(last_saved_fault.fault_code),
               last_saved_fault.current_ma,
               (float)last_saved_fault.rpm_at_trip / 10.0f);
    else
      snprintf(cli_last_reply, sizeof(cli_last_reply), "FLASH LOG EMPTY (0 TRIPS)");
  }
  else
  {
    snprintf(cli_last_reply, sizeof(cli_last_reply), "UNKNOWN CMD '%s' (TYPE HELP)", cmd);
  }
}

/* Autonomous Emergency Cutoff & Non-Volatile Flash Logging ------------------*/
static void Trigger_Emergency_Cutoff(FaultCode_t code, uint32_t detect_start_cyc)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  uint32_t cutoff_cyc = DWT->CYCCNT;
  trip_latency_us = (cutoff_cyc - detect_start_cyc) / 170U;

  float inst_curr_A = fabsf(CURRENT_SIGN * (adc_voltage - zero_current_v) / ACS712_SENS_V_PER_A);
  float log_curr_A  = (inst_curr_A > fabsf(motor_current_A)) ? inst_curr_A : fabsf(motor_current_A);

  control_output = 0.0f;
  integral_accum = 0.0f;
  fault_latched = 1;
  active_fault = code;
  current_health_state = HEALTH_CRITICAL_FAULT;
  health_score = 0.0f;

  FaultLogRecord_t rec;
  rec.magic        = FLASH_LOG_MAGIC;
  rec.timestamp_ms = HAL_GetTick();
  rec.trip_count   = has_saved_fault ? (last_saved_fault.trip_count + 1U) : 1U;
  rec.fault_code   = (uint8_t)code;
  rec.health_score = 0;
  rec.rpm_at_trip  = (int16_t)(filtered_rpm * 10.0f);
  rec.current_ma   = (uint16_t)(log_curr_A * 1000.0f);

  Flash_Save_Fault_Record(&rec);
  last_saved_fault = rec;
  has_saved_fault = 1;
}

static void Flash_Load_Last_Fault(void)
{
  const FaultLogRecord_t *flash_rec = (const FaultLogRecord_t *)FLASH_LOG_ADDR;
  if (flash_rec->magic == FLASH_LOG_MAGIC)
  {
    last_saved_fault = *flash_rec;
    has_saved_fault = 1;
  }
  else
  {
    memset(&last_saved_fault, 0, sizeof(FaultLogRecord_t));
    has_saved_fault = 0;
  }
}

static void Flash_Save_Fault_Record(const FaultLogRecord_t *rec)
{
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef eraseInit = {0};
  uint32_t pageError = 0;
  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.Banks     = FLASH_BANK_1;
  eraseInit.Page      = FLASH_LOG_PAGE;
  eraseInit.NbPages   = 1;

  if (HAL_FLASHEx_Erase(&eraseInit, &pageError) == HAL_OK)
  {
    const uint64_t *dw_ptr = (const uint64_t *)rec;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_LOG_ADDR, dw_ptr[0]);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_LOG_ADDR + 8U, dw_ptr[1]);
  }

  HAL_FLASH_Lock();
}

static const char* Fault_Code_To_String(uint8_t code)
{
  switch (code)
  {
    case FAULT_ROTOR_STALL:    return "ROTOR STALL";
    case FAULT_OVERCURRENT:    return "OVERCURRENT";
    case FAULT_HIGH_VIBRATION: return "HIGH VIBRATION";
    case FAULT_WATCHDOG_RESET: return "IWDG RECOVERY";
    default:                   return "NONE";
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
      printf("    PHASE 10 FINAL: EDGE MOTOR HEALTH, IWDG & UART CLI SYSTEM    \r\n");
      break;
    case 2:
      printf("=================================================================\r\n");
      break;
    case 3:
      printf(" SPEED CTRL    : TARGET: %5.1f RPM  | ACTUAL: %5.1f RPM%s\r\n",
             target_rpm, filtered_rpm, fault_latched ? " [TRIPPED]" : "          ");
      break;
    case 4:
      printf(" PI GAINS      : Kp: %4.2f | Ki: %4.2f | PWM: %4lu/999\r\n",
             Kp, Ki, (uint32_t)control_output);
      break;
    case 5:
      printf(" IWDG SUPERVIS : ARMED (500 ms)   | KICKS: %lu | BOOT: %s\r\n",
             iwdg_kick_count, wdg_recovery_boot ? "WDG_REC" : "NORMAL ");
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
      if (has_saved_fault)
        printf(" FLASH BLACKBOX: TRIP #%u | %s | %u mA | %.1f RPM @ %lu ms\r\n",
               last_saved_fault.trip_count,
               Fault_Code_To_String(last_saved_fault.fault_code),
               last_saved_fault.current_ma,
               (float)last_saved_fault.rpm_at_trip / 10.0f,
               last_saved_fault.timestamp_ms);
      else
        printf(" FLASH BLACKBOX: NO FAULTS LOGGED IN PAGE 255 (0x0807F800)\r\n");
      break;
    case 11:
      printf("-----------------------------------------------------------------\r\n");
      break;
    case 12:
      if (mpu_ok)
        printf(" MPU6050       : ONLINE (0x%02X)    | N=%d BINS (df=%.3f Hz)\r\n",
               active_mpu_addr, FFT_SAMPLES, (SAMPLING_FREQ_HZ / FFT_SAMPLES));
      else
      {
        printf(" MPU6050       : OFFLINE / BUS FAULT DETECTED\r\n");
        print_step = 16;
      }
      break;
    case 13:
      printf(" AXIS RMS (g)  : X: %5.3f  | Y: %5.3f  | Z: %5.3f\r\n", vib_rms_x, vib_rms_y, vib_rms_z);
      break;
    case 14:
      printf(" VIB METRICS   : TOTAL RMS: %5.3f g | PK-TO-PK: %5.3f g\r\n", total_vib_rms, peak_to_peak_g);
      break;
    case 15:
      printf(" SPECTRAL FFT  : f0: %5.2f Hz | fP: %5.2f Hz | RUN: %.1f us\r\n",
             expected_shaft_freq_hz, dominant_freq_hz, fft_execution_time_us);
      break;
    case 16:
      printf("-----------------------------------------------------------------\r\n");
      break;
    case 17:
      printf(" UART CLI      : %s\r\n", cli_last_reply);
      break;
    case 18:
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
  if (fault_latched)
  {
    health_score = 0.0f;
    current_health_state = HEALTH_CRITICAL_FAULT;
    return;
  }

  float current_ma = fabsf(motor_current_A) * 1000.0f;
  pen_i = 0.0f;
  pen_v = 0.0f;
  pen_r = 0.0f;

  float abs_speed_err = fabsf(target_rpm - filtered_rpm);
  if (abs_speed_err > 5.0f)
  {
    pen_r = ((abs_speed_err - 5.0f) / 45.0f) * 30.0f;
  }

  if (current_ma > 400.0f)
  {
    pen_i = ((current_ma - 400.0f) / 300.0f) * 35.0f;
  }

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

  if (fault_latched)
  {
    snprintf(line1, sizeof(line1), "FAULT: %-9s", Fault_Code_To_String((uint8_t)active_fault));
    snprintf(line2, sizeof(line2), "PRESS B1 TO RST ");
  }
  else if (lcd_page == 0)
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

/* Closed-Loop PI Speed Controller -------------------------------------------*/
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

  if (fault_latched)
  {
    control_output = 0.0f;
    integral_accum = 0.0f;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    return;
  }

  speed_error = target_rpm - filtered_rpm;
  float P_term = Kp * speed_error;

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

static void Calibrate_Zero_Current(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);

  float acc = 0.0f;
  for (int k = 0; k < 20; k++)
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

/* Independent Watchdog Initialization (~500 ms Supervisor Timeout) ---------*/
static void MX_IWDG_Init(void)
{
  __HAL_DBGMCU_FREEZE_IWDG(); /* Freeze watchdog when debugger hits a breakpoint */

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32; /* 32 kHz / 32 = 1 kHz (1 ms tick) */
  hiwdg.Init.Window    = 500;
  hiwdg.Init.Reload    = 500;               /* 500 ms hardware timeout */
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
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
void assert_failed(uint8_t *file, uint8_t line) {}
#endif
