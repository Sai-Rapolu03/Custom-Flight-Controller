/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef  hspi1;
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ── Timing ────────────────────────────────────────────────────────────────
 * TIM2: APB1 timer clock = 90 MHz, PSC = 8999 → tick = 10 kHz,
 *        ARR = 19  → ISR fires at 500 Hz, DT = 2 ms.
 */
#define SAMPLE_FREQ  500.0f
#define DT           (1.0f / SAMPLE_FREQ)

/* Complementary filter weight for gyro vs accel (0.98 = 98 % gyro) */
#define CF_ALPHA     0.98f

/* ── iBUS ──────────────────────────────────────────────────────────────── */
#define IBUS_FRAME_LENGTH  32
#define IBUS_HEADER_BYTE0  0x20
#define IBUS_HEADER_BYTE1  0x40
#define IBUS_NUM_CHANNELS  14

uint8_t  ibus_raw_buf[IBUS_FRAME_LENGTH];
uint8_t  ibus_frame[IBUS_FRAME_LENGTH];
uint16_t ibus_channels[IBUS_NUM_CHANNELS];
uint8_t  ibus_rx_index = 0;

/*
 * FIX-ARMING: ibus_frame_ready MUST be volatile.
 * The UART ISR sets it to 1; if it is not volatile the compiler may cache
 * it as 0 in a register and the main loop never sees the ISR write,
 * so channels are never decoded and arm_input stays at 1500 → never arms.
 */
volatile uint8_t ibus_frame_ready = 0;

/* ── IMU ────────────────────────────────────────────────────────────────── */
int16_t accel[3], gyro[3];
volatile uint8_t imu_flag = 0;   /* set by TIM2 ISR at 500 Hz */

/* Low-pass filter states — kept in physical units (g and deg/s) */
float ax_f = 0, ay_f = 0, az_f = 0;
float gx_f = 0, gy_f = 0, gz_f = 0;

/* Gyro static bias (removed during calibration) */
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

/* Level-calibration offsets (mounting tilt correction) */
float roll_offset  = 0.0f;
float pitch_offset = 0.0f;

/* ── PID ─────────────────────────────────────────────────────────────────
 * D-on-measurement: pid = Kp * angle_error  -  Kd * gyro_rate
 * Kd acts directly on the gyro rate (deg/s) so no numerical differentiation
 * of the noisy angle estimate is needed.
 */
float Kp       = 3.0f,  Kd       = 0.585f;   /* roll  */
float Kp_pitch = 2.8f,  Kd_pitch = 0.72f;   /* pitch */
float Ki = 0.05f; // Small baseline to start
float roll_i = 0.0f;
float pitch_i = 0.0f;
#define I_LIMIT 50.0f // Keep it safely capped to avoid windup
float Kp_yaw   = 2.0f,  Kd_yaw   = 0.02f;   /* yaw (rate mode) */
float yaw_rate_setpoint = 0;
float prev_yaw_error    = 0;
float derivative_yaw_f  = 0;

/* Setpoints (computed from RC sticks each loop) */
float roll_setpoint  = 0;
float pitch_setpoint = 0;

/* ── Motor trim ─────────────────────────────────────────────────────────── */
float m1_trim = 0;
float m2_trim = 0;
float m3_trim = 0;
float m4_trim = 0;

/* ── Arm state ──────────────────────────────────────────────────────────── */
int armed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void    IBUS_Init(void);
void    IBUS_ProcessByte(uint8_t byte);
uint8_t IBUS_ValidateFrame(uint8_t *buf);
void    IBUS_DecodeChannels(uint8_t *buf, uint16_t *channels);
void    MPU9250_Init(void);
void    MPU9250_Read(int16_t *accel, int16_t *gyro);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define CS_LOW()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)

void SPI_Write(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { reg & 0x7F, data };
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

void SPI_Read(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t addr = reg | 0x80;
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
    CS_HIGH();
}

void MPU9250_Init(void)
{
    HAL_Delay(100);
    SPI_Write(0x6B, 0x00);  /* wake up                        */
    HAL_Delay(10);
    SPI_Write(0x1A, 0x03);  /* DLPF 41 Hz                     */
    SPI_Write(0x1B, 0x18);  /* gyro  ±2000 dps → 16.4 LSB/dps */
    SPI_Write(0x1C, 0x10);  /* accel ±8 g    → 4096 LSB/g     */
}

void MPU9250_Read(int16_t *accel, int16_t *gyro)
{
    uint8_t raw[14];
    SPI_Read(0x3B, raw, 14);

    accel[0] = (int16_t)((raw[0]  << 8) | raw[1]);
    accel[1] = (int16_t)((raw[2]  << 8) | raw[3]);
    accel[2] = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] = temperature, intentionally skipped */
    gyro[0]  = (int16_t)((raw[8]  << 8) | raw[9]);
    gyro[1]  = (int16_t)((raw[10] << 8) | raw[11]);
    gyro[2]  = (int16_t)((raw[12] << 8) | raw[13]);
}

/* USER CODE END 0 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* ── iBUS init ──────────────────────────────────────────────────────── */
  IBUS_Init();
  for (int i = 0; i < IBUS_NUM_CHANNELS; i++)
      ibus_channels[i] = 1500;
  CS_HIGH();

  /* ── IMU init ───────────────────────────────────────────────────────── */
  MPU9250_Init();
  HAL_TIM_Base_Start_IT(&htim2);   /* start 500 Hz control tick */

  /* ── PWM outputs (ESC signals) ──────────────────────────────────────── */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* Let ESCs and IMU settle before calibration */
  HAL_Delay(2000);

  /* ── STEP 1: Gyro bias calibration (2000 samples ≈ 2 s) ────────────────
   * FIX: This loop was missing from the previous version.
   * Without it, gx_bias / gy_bias / gz_bias remain 0, the gyro integration
   * in the complementary filter drifts, and the level calibration is
   * inaccurate.  Keep the drone perfectly still during this phase.
   */
  for (int i = 0; i < 2000; i++)
  {
      MPU9250_Read(accel, gyro);
      gx_bias += gyro[0];
      gy_bias += gyro[1];
      gz_bias += gyro[2];
      HAL_Delay(1);
  }
  gx_bias /= 2000.0f;
  gy_bias /= 2000.0f;
  gz_bias /= 2000.0f;

  /* ── STEP 2: Level calibration (measures IMU mounting tilt offset) ────
   * roll_est / pitch_est are declared here so they are shared between
   * the calibration block below AND the while(1) loop that follows,
   * because both live inside main().  'static' ensures zero-initialisation
   * even though they are local.
   */
  static float roll_est  = 0.0f;
  static float pitch_est = 0.0f;

  /* 2a – Seed the complementary filter with 50 accel-only passes so the
   *       angle estimate has converged before we start averaging.         */
  for (int i = 0; i < 50; i++)
  {
      MPU9250_Read(accel, gyro);
      float ax = accel[0] / 4096.0f;
      float ay = accel[1] / 4096.0f;
      float az = accel[2] / 4096.0f;

      ax_f = 0.85f * ax_f + 0.15f * ax;
      ay_f = 0.85f * ay_f + 0.15f * ay;
      az_f = 0.85f * az_f + 0.15f * az;

      float n = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
      if (n > 0.0001f)
      {
          float ax_n = ax_f/n, ay_n = ay_f/n, az_n = az_f/n;
          roll_est  = CF_ALPHA * (roll_est)
                    + (1.0f - CF_ALPHA) * atan2f(ay_n, az_n) * 57.2958f;
          pitch_est = CF_ALPHA * (pitch_est)
                    + (1.0f - CF_ALPHA) * atan2f(-ax_n, sqrtf(ay_n*ay_n + az_n*az_n)) * 57.2958f;
      }
      HAL_Delay(2);
  }

  /* 2b – Average 500 settled CF samples to find the resting offset.      */
  float sum_roll  = 0.0f;
  float sum_pitch = 0.0f;
  const int CAL_SAMPLES = 500;

  for (int i = 0; i < CAL_SAMPLES; i++)
  {
      MPU9250_Read(accel, gyro);

      float gx_cal = (gyro[0] - gx_bias) / 16.4f;
      float gy_cal = (gyro[1] - gy_bias) / 16.4f;

      float ax = accel[0] / 4096.0f;
      float ay = accel[1] / 4096.0f;
      float az = accel[2] / 4096.0f;

      ax_f = 0.85f * ax_f + 0.15f * ax;
      ay_f = 0.85f * ay_f + 0.15f * ay;
      az_f = 0.85f * az_f + 0.15f * az;

      float n = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
      if (n > 0.0001f)
      {
          float ax_n = ax_f/n, ay_n = ay_f/n, az_n = az_f/n;
          float ra = atan2f(ay_n, az_n) * 57.2958f;
          float pa = atan2f(-ax_n, sqrtf(ay_n*ay_n + az_n*az_n)) * 57.2958f;

          roll_est  = CF_ALPHA * (roll_est  + gx_cal * DT) + (1.0f - CF_ALPHA) * ra;
          pitch_est = CF_ALPHA * (pitch_est + gy_cal * DT) + (1.0f - CF_ALPHA) * pa;
      }

      sum_roll  += roll_est;
      sum_pitch += pitch_est;
      HAL_Delay(2);
  }

  roll_offset  = sum_roll  / (float)CAL_SAMPLES;
  pitch_offset = sum_pitch / (float)CAL_SAMPLES;

  /* USER CODE END 2 */

  /* ═══════════════════════════════════════════════════════════════════════
   *  MAIN LOOP
   * ═══════════════════════════════════════════════════════════════════════ */
  /* USER CODE BEGIN WHILE */

  static int print_counter = 0;

  while (1)
  {
      /* Wait for the 500 Hz timer tick */
      if (!imu_flag) continue;
      imu_flag = 0;

      /* ── iBUS update ────────────────────────────────────────────────── */
      if (ibus_frame_ready)
      {
          ibus_frame_ready = 0;
          IBUS_DecodeChannels(ibus_frame, ibus_channels);
      }

      /* ── Read IMU ───────────────────────────────────────────────────── */
      MPU9250_Read(accel, gyro);

      float ax = accel[0] / 4096.0f;   /* g      */
      float ay = accel[1] / 4096.0f;
      float az = accel[2] / 4096.0f;

      float gx = (gyro[0] - gx_bias) / 16.4f;  /* deg/s */
      float gy = (gyro[1] - gy_bias) / 16.4f;
      float gz = (gyro[2] - gz_bias) / 16.4f;

      /* ── Low-pass filter — state kept in physical units ─────────────── */
      const float alpha = 0.15f;
      ax_f = (1.0f - alpha) * ax_f + alpha * ax;
      ay_f = (1.0f - alpha) * ay_f + alpha * ay;
      az_f = (1.0f - alpha) * az_f + alpha * az;
      gx_f = (1.0f - alpha) * gx_f + alpha * gx;
      gy_f = (1.0f - alpha) * gy_f + alpha * gy;
      gz_f = (1.0f - alpha) * gz_f + alpha * gz;

      /* ── Normalise accel into LOCAL variables only ───────────────────── */
      float ax_n = ax_f, ay_n = ay_f, az_n = az_f;
      float norm  = sqrtf(ax_n*ax_n + ay_n*ay_n + az_n*az_n);
      if (norm > 0.0001f)
      {
          ax_n /= norm;
          ay_n /= norm;
          az_n /= norm;
      }

      /* ── Accel-only tilt (used only as slow drift correction) ────────── */
      float roll_accel  = atan2f(ay_n, az_n) * 57.2958f;
      float pitch_accel = atan2f(-ax_n, sqrtf(ay_n*ay_n + az_n*az_n)) * 57.2958f;

      /* ── Complementary filter ───────────────────────────────────────────
       * 98 % gyro integration + 2 % accelerometer correction.
       *
       * SIGN CHECK — verify once before first flight (no props):
       *   Tilt drone RIGHT  → CF_Roll  should INCREASE. If it decreases,
       *   change +gx_f to -gx_f on the roll_est line.
       *   Tilt nose UP      → CF_Pitch should INCREASE. If it decreases,
       *   change +gy_f to -gy_f on the pitch_est line.
       */
      roll_est  = CF_ALPHA * (roll_est  + gx_f * DT) + (1.0f - CF_ALPHA) * roll_accel;
      pitch_est = CF_ALPHA * (pitch_est + gy_f * DT) + (1.0f - CF_ALPHA) * pitch_accel;

      /* ── FIX: Apply level-calibration offset ────────────────────────────
       * roll_est / pitch_est still contain the raw (uncorrected) angle.
       * Subtracting the offset cancels the IMU mounting tilt so that
       * a physically-level drone reads 0° / 0°.
       * The previous version forgot this subtraction, so the controller
       * permanently fought a phantom lean equal to the mounting offset.
       */
      float roll  = roll_est  - roll_offset;
      float pitch = -(pitch_est - pitch_offset);

      /* ── RC stick inputs ────────────────────────────────────────────── */
      float roll_input     = ((int)ibus_channels[0] - 1500) / 500.0f;
      float pitch_input    = ((int)ibus_channels[1] - 1500) / 500.0f;
      float yaw_input      = ((int)ibus_channels[3] - 1500) / 500.0f;
      float throttle_input = (ibus_channels[2] - 1000) / 1000.0f;

      if (throttle_input < 0.0f) throttle_input = 0.0f;
      if (throttle_input > 1.0f) throttle_input = 1.0f;

      /* ── Yaw: rate mode (stick → desired yaw rate) ──────────────────── */
      if (yaw_input > -0.05f && yaw_input < 0.05f)
          yaw_input = 0.0f;

      yaw_rate_setpoint = yaw_input * 150.0f;   /* ±150 deg/s max */

      float yaw_error = yaw_rate_setpoint - gz;  /* unfiltered = lower latency */
      float dy = (yaw_error - prev_yaw_error) * SAMPLE_FREQ;
      derivative_yaw_f = 0.9f * derivative_yaw_f + 0.1f * dy;
      float pid_yaw = Kp_yaw * yaw_error + Kd_yaw * derivative_yaw_f;
      prev_yaw_error = yaw_error;

      if (pid_yaw >  200.0f) pid_yaw =  200.0f;
      if (pid_yaw < -200.0f) pid_yaw = -200.0f;

      /* ── Arm / Disarm ───────────────────────────────────────────────────
       * ARM  : Channel 5 switch HIGH (>1800) AND throttle at bottom (<1050)
       * DISARM: Channel 5 switch LOW  (<1200)
       */
      float    arm_input = ibus_channels[4];
      uint16_t thr_raw   = ibus_channels[2];

      if (arm_input > 1800 && thr_raw < 1050) armed = 1;
      if (arm_input < 1200)                    armed = 0;

      /* ── Angle setpoints (from RC sticks) ───────────────────────────── */
      const float max_angle = 20.0f;
      roll_setpoint  = roll_input  * max_angle;
      pitch_setpoint = pitch_input * max_angle;

      /* ── Throttle base ──────────────────────────────────────────────── */
      float base     = 1000.0f + throttle_input * 800.0f;
      float min_idle = 1120.0f;

      if (armed && throttle_input < 0.05f)
          base = min_idle;   /* keep motors spinning at idle */

      /* ── PID — D-on-measurement (gyro rate) ─────────────────────────── */
      float error   = roll_setpoint  - roll;
      float error_p = pitch_setpoint - pitch;

      // Integrate the errors over your 2 ms step time (DT)
      roll_i += error * DT;
      pitch_i += error_p * DT;

      // Windup limits
      if (roll_i > I_LIMIT) roll_i = I_LIMIT;
      if (roll_i < -I_LIMIT) roll_i = -I_LIMIT;
      if (pitch_i > I_LIMIT) pitch_i = I_LIMIT;
      if (pitch_i < -I_LIMIT) pitch_i = -I_LIMIT;

      // Reset I-term if disarmed to prevent explosive takeoff spin-up
      if (!armed) {
          roll_i = 0.0f;
          pitch_i = 0.0f;
      }

      // Complete PID application
      float pid_roll = (Kp * error) + (Ki * roll_i) - (Kd * gx_f);
      float pid_pitch = (Kp_pitch * error_p) + (Ki * pitch_i) + (Kd_pitch * gy_f);


      if (pid_roll  >  150.0f) pid_roll  =  150.0f;
      if (pid_roll  < -150.0f) pid_roll  = -150.0f;
      if (pid_pitch >  150.0f) pid_pitch =  150.0f;
      if (pid_pitch < -150.0f) pid_pitch = -150.0f;

      /* ── Motor mix — X-frame ────────────────────────────────────────────
       *
       *        FRONT
       *  M1(CCW) | M2(CW)
       *  --------+--------
       *  M3(CW)  | M4(CCW)
       *        REAR
       *
       * QUICK SANITY TEST (no props, very low throttle, drone in hand):
       *   Tilt drone RIGHT  → M2 + M4 (right) should spin FASTER.
       *   If M1+M3 spin faster instead, negate pid_roll in all 4 lines.
       *   Tilt nose DOWN    → M1 + M2 (front) should spin FASTER.
       *   If M3+M4 spin faster instead, negate pid_pitch in all 4 lines.
       */
      float m1, m2, m3, m4;

      if (!armed)
      {
          m1 = m2 = m3 = m4 = 1000.0f;
      }
      else
      {
          m1 = base + pid_pitch + pid_roll - pid_yaw;  /* Front-Left  CCW */
          m2 = base + pid_pitch - pid_roll + pid_yaw;  /* Front-Right CW  */
          m3 = base - pid_pitch + pid_roll + pid_yaw;  /* Rear-Left   CW  */
          m4 = base - pid_pitch - pid_roll - pid_yaw;  /* Rear-Right  CCW */
      }

      /* ── Motor trim ─────────────────────────────────────────────────── */
      m1 += m1_trim;
      m2 += m2_trim;
      m3 += m3_trim;
      m4 += m4_trim;

      /* ── Output limits ──────────────────────────────────────────────── */
      if (armed)
      {
          if (m1 < min_idle) m1 = min_idle;
          if (m2 < min_idle) m2 = min_idle;
          if (m3 < min_idle) m3 = min_idle;
          if (m4 < min_idle) m4 = min_idle;
      }
      if (m1 > 2000.0f) m1 = 2000.0f;
      if (m2 > 2000.0f) m2 = 2000.0f;
      if (m3 > 2000.0f) m3 = 2000.0f;
      if (m4 > 2000.0f) m4 = 2000.0f;

      /* ── PWM output ─────────────────────────────────────────────────── */
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t)m1);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint16_t)m2);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)m3);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint16_t)m4);

      /* ── Debug UART (5 Hz = every 100 control ticks) ─────────────────── */
      print_counter++;
      if (print_counter >= 100)
      {
          print_counter = 0;
          char buf[300];
          sprintf(buf,
              "ARM:%d THR:%.0f | "
              "Roll:%.2f Pit:%.2f | "
              "AC_R:%.2f AC_P:%.2f | "
              "GX:%.2f GY:%.2f GZ:%.2f | "
              "M:%.0f %.0f %.0f %.0f\r\n",
              armed, base,
              roll, pitch,          /* offset-corrected — shows 0 when level */
              roll_accel, pitch_accel,
              gx_f, gy_f, gz_f,
              m1, m2, m3, m4);
          HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 50);
      }

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* ── TIM2 ISR: fires at 500 Hz, sets imu_flag ─────────────────────────── */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
        imu_flag = 1;
}

/* ── iBUS receive ────────────────────────────────────────────────────────
 * Single-byte interrupt-driven reception.
 * IBUS_ProcessByte runs in ISR context — keep it fast.
 */
void IBUS_Init(void)
{
    ibus_rx_index    = 0;
    ibus_frame_ready = 0;
    HAL_UART_Receive_IT(&huart1, &ibus_raw_buf[0], 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        IBUS_ProcessByte(ibus_raw_buf[0]);
        HAL_UART_Receive_IT(&huart1, &ibus_raw_buf[0], 1);
    }
}

/* ── iBUS frame parser ───────────────────────────────────────────────────
 * FIX: Validates BOTH header bytes (0x20 then 0x40) before collecting the
 * payload.  The old single-byte check allowed any 0x20 inside payload data
 * to trigger a false frame-sync, producing garbage channel values.
 */
void IBUS_ProcessByte(uint8_t byte)
{
    static uint8_t state = 0;

    switch (state)
    {
        case 0:   /* waiting for first header byte */
            if (byte == IBUS_HEADER_BYTE0)
            {
                ibus_frame[0] = byte;
                ibus_rx_index = 1;
                state = 1;
            }
            break;

        case 1:   /* waiting for second header byte */
            if (byte == IBUS_HEADER_BYTE1)
            {
                ibus_frame[1] = byte;
                ibus_rx_index = 2;
                state = 2;
            }
            else if (byte == IBUS_HEADER_BYTE0)
            {
                /* Might be the start of a new frame — stay ready */
                ibus_frame[0] = byte;
                ibus_rx_index = 1;
                /* stay in state 1 */
            }
            else
            {
                state = 0;   /* bad second byte, restart */
            }
            break;

        case 2:   /* collecting 30 payload bytes */
            ibus_frame[ibus_rx_index++] = byte;

            if (ibus_rx_index >= IBUS_FRAME_LENGTH)
            {
                ibus_rx_index = 0;
                state = 0;

                if (IBUS_ValidateFrame(ibus_frame))
                    ibus_frame_ready = 1;
            }
            break;

        default:
            state = 0;
            break;
    }
}

uint8_t IBUS_ValidateFrame(uint8_t *buf)
{
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < 30; i++)
        checksum -= buf[i];

    uint16_t recv = ((uint16_t)buf[31] << 8) | buf[30];
    return (checksum == recv);
}

void IBUS_DecodeChannels(uint8_t *buf, uint16_t *channels)
{
    for (int i = 0; i < IBUS_NUM_CHANNELS; i++)
    {
        int offset  = 2 + i * 2;
        channels[i] = (uint16_t)(buf[offset] | ((uint16_t)buf[offset + 1] << 8));
    }
}

/* USER CODE END 4 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  PERIPHERAL INITIALISATION  (generated by CubeMX — do not hand-edit)
 * ═══════════════════════════════════════════════════════════════════════════ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 360;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 2;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 8999;  /* 90 MHz / 9000 = 10 kHz tick */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 19;    /* 10 kHz / 20   = 500 Hz ISR  */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef      sConfigOC     = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 89;     /* 90 MHz / 90    = 1 MHz tick  */
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 19999;  /* 1 MHz  / 20000 = 50 Hz PWM   */
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 1000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
