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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f3xx_hal.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* -------------------------------------------------------------------------- */
/*  MLX90393 I2C command bytes                                                */
/* -------------------------------------------------------------------------- */
#define MLX_CMD_NOP       0x00
#define MLX_CMD_SB        0x10   /* Start Burst mode                         */
#define MLX_CMD_SW        0x20   /* Start Wake-on-change                     */
#define MLX_CMD_SM        0x3E   /* Start Single Measurement (X+Y+Z+T)       */
#define MLX_CMD_RM        0x4E   /* Read Measurement        (X+Y+Z+T)       */
#define MLX_CMD_RR        0x50   /* Read Register                            */
#define MLX_CMD_WR        0x60   /* Write Register                           */
#define MLX_CMD_EX        0x80   /* Exit mode                                */
#define MLX_CMD_RT        0xF0   /* Reset                                    */


/* Bit-mask for ZYXT in SM/RM commands */
#define MLX_ZYXT          0x0F

/* Status byte error flag */
#define MLX_STATUS_ERROR  0x10

/* -------------------------------------------------------------------------- */
/*  Timing                                                                    */
/* -------------------------------------------------------------------------- */
#define MLX_MEAS_DELAY_MS   20u  /* ≥ measurement conversion time            */
#define SENSOR_POLL_DELAY   5u   /* small gap between sensors                */
#define LOOP_DELAY_MS     500u   /* print all sensors every 500 ms           */

/* -------------------------------------------------------------------------- */
/*  Sensor address table                                                      */
/* -------------------------------------------------------------------------- */
#define NUM_SENSORS  16u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const uint8_t sensor_addr[NUM_SENSORS] = {
    0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B
};
/* -------------------------------------------------------------------------- */
/*  Data structure for one sensor reading                                     */
/* -------------------------------------------------------------------------- */
typedef struct {
    uint8_t  addr;
    int16_t  x;
    int16_t  y;
    int16_t  z;
    int16_t  t;          /* raw temperature counts                           */
    float    temp_c;     /* converted temperature in °C                      */
    uint8_t  status;
    uint8_t  valid;      /* 1 = data ok, 0 = sensor not responding           */
} MLX90393_Data_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* -------------------------------------------------------------------------- */
/*  UART helper – redirect printf to USART2                                   */
/* -------------------------------------------------------------------------- */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}

/* Small non-printf transmit for raw strings */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* -------------------------------------------------------------------------- */
/*  MLX90393 low-level helpers                                                */
/* -------------------------------------------------------------------------- */

/**
 * Send a single command byte and read back the status byte.
 * Returns HAL_OK on success.
 */
static HAL_StatusTypeDef mlx_send_cmd(uint8_t addr_7bit,
                                      uint8_t cmd,
                                      uint8_t *status_out)
{
    uint8_t status = 0;
    uint8_t i2c_addr = (uint8_t)(addr_7bit << 1);   /* HAL uses 8-bit addr  */

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
            &hi2c1, i2c_addr, &cmd, 1, 50);
    if (ret != HAL_OK) return ret;

    ret = HAL_I2C_Master_Receive(
            &hi2c1, i2c_addr, &status, 1, 50);
    if (status_out) *status_out = status;
    return ret;
}

/**
 * Issue SM (Start Single Measurement) for X, Y, Z and T, wait for
 * conversion, then issue RM (Read Measurement) and parse the 9 bytes:
 *   [status][T_H][T_L][X_H][X_L][Y_H][Y_L][Z_H][Z_L]
 */
static HAL_StatusTypeDef mlx_read_single(uint8_t addr_7bit,
                                         MLX90393_Data_t *out)
{
    uint8_t  i2c_addr = (uint8_t)(addr_7bit << 1);
    uint8_t  cmd;
    uint8_t  status;
    uint8_t  buf[9];
    HAL_StatusTypeDef ret;

    out->valid = 0;

/* Helper: reset the I2C peripheral so a failed transaction on one sensor
     * doesn't leave the bus in a stuck state and block all subsequent sensors. */
#define I2C_RESET_ON_ERR(ret) \
    do { if ((ret) != HAL_OK) { HAL_I2C_DeInit(&hi2c1); HAL_I2C_Init(&hi2c1); return ret; } } while(0)

    /* ---- 1. Start Single Measurement (ZYXT = 0x0F) ---- */
    cmd = MLX_CMD_SM | MLX_ZYXT;
    ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, &cmd, 1, 50);
    I2C_RESET_ON_ERR(ret);

    /* Read status byte that follows the SM command */
    ret = HAL_I2C_Master_Receive(&hi2c1, i2c_addr, &status, 1, 50);
    I2C_RESET_ON_ERR(ret);
    if (status & MLX_STATUS_ERROR) return HAL_ERROR;

    /* ---- 2. Wait for conversion ---- */
    HAL_Delay(MLX_MEAS_DELAY_MS);

    /* ---- 3. Read Measurement (ZYXT = 0x0F) → 9 bytes ---- */
    cmd = MLX_CMD_RM | MLX_ZYXT;
    ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, &cmd, 1, 50);
    I2C_RESET_ON_ERR(ret);

    /* Receive: [status][T_H][T_L][X_H][X_L][Y_H][Y_L][Z_H][Z_L] */
    ret = HAL_I2C_Master_Receive(&hi2c1, i2c_addr, buf, 9, 50);
    I2C_RESET_ON_ERR(ret);

    out->status = buf[0];
    if (out->status & MLX_STATUS_ERROR) return HAL_ERROR;

    /* ---- 4. Parse raw 16-bit signed values ---- */
    out->t = (int16_t)(((uint16_t)buf[1] << 8) | buf[2]);
    out->x = (int16_t)(((uint16_t)buf[3] << 8) | buf[4]);
    out->y = (int16_t)(((uint16_t)buf[5] << 8) | buf[6]);
    out->z = (int16_t)(((uint16_t)buf[7] << 8) | buf[8]);

    /*
     * Temperature conversion (from datasheet §11.2):
     *   T(°C) = (T_raw - 46244) / 45.2 + 25
     * T0 = 46244 at 25 °C, sensitivity = 45.2 LSB/°C
     */
    out->temp_c = ((float)out->t - 46244.0f) / 45.2f + 25.0f;

    out->addr  = addr_7bit;
    out->valid = 1;
    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * Reset all sensors on the bus (broadcast or individually).
 * Call once after power-up before reading.
 */
void MLX90393_ResetAll(void)
{
    uart_print("\r\n[MLX90393] Resetting all sensors...\r\n");
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        uint8_t status;
        uint8_t i2c_addr = (uint8_t)(sensor_addr[i] << 1);
        uint8_t cmd = MLX_CMD_RT;

        HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, &cmd, 1, 50);
        HAL_I2C_Master_Receive( &hi2c1, i2c_addr, &status, 1, 50);
        HAL_Delay(5);
    }
    HAL_Delay(100);   /* let all sensors finish reset */
    uart_print("[MLX90393] Reset done.\r\n");
}

/**
 * Scan the bus and print which addresses ACK.
 */
//void MLX90393_ScanBus(void)
//{
//    char line[64];
//    uart_print("\r\n[I2C Scan] Checking addresses 0x0C - 0x1B:\r\n");
//
//    for (uint8_t i = 0; i < NUM_SENSORS; i++)
//    {
//        uint8_t addr    = sensor_addr[i];
//        uint8_t i2c_8   = (uint8_t)(addr << 1);
//        HAL_StatusTypeDef ret =
//            HAL_I2C_IsDeviceReady(&hi2c1, i2c_8, 2, 20);
//
//        snprintf(line, sizeof(line),
//                 "  0x%02X : %s\r\n",
//                 addr,
//                 (ret == HAL_OK) ? "FOUND" : "---");
//        uart_print(line);
//    }
//    uart_print("[I2C Scan] Done.\r\n\r\n");
//}

/**
 * Read all 16 sensors and print results over UART2.
 * Call this repeatedly from your main loop.
 */
void MLX90393_ReadAllSensors(void)
{
    static uint32_t cycle = 0;
    MLX90393_Data_t data;
    char line[128];

    snprintf(line, sizeof(line),
             "\r\n===== Cycle %lu =====\r\n", cycle++);
    uart_print(line);

    uart_print("Addr |  X (raw) |  Y (raw) |  Z (raw) |  T (raw) | Temp(C) | Status\r\n");
    uart_print("-----+----------+----------+----------+----------+---------+--------\r\n");

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        HAL_StatusTypeDef ret = mlx_read_single(sensor_addr[i], &data);

        if (ret == HAL_OK && data.valid)
        {
            snprintf(line, sizeof(line),
                     "0x%02X | %8d | %8d | %8d | %8d | 0x%02X\r\n",
                     data.addr,
                     (int)data.x,
                     (int)data.y,
                     (int)data.z,
                     (int)data.t,
                     data.status);
        }
        else
        {
            snprintf(line, sizeof(line),
                     "0x%02X |  NO RESPONSE (HAL ret=%d)\r\n",
                     sensor_addr[i], (int)ret);
        }

        uart_print(line);
        HAL_Delay(SENSOR_POLL_DELAY);
    }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // i2c scanner
	  /*printf("\r\nScanning I2C bus...\r\n");

	  for (uint8_t addr = 1; addr < 128; addr++)
	  {
	      if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 3, 5) == HAL_OK)
	      {
	          printf("Device found at 0x%02X\r\n", addr);
	      }
	  }

	  printf("Scan done.\r\n\r\n");
	  HAL_Delay(2000);*/

	  MLX90393_ReadAllSensors();
	  HAL_Delay(LOOP_DELAY_MS);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
