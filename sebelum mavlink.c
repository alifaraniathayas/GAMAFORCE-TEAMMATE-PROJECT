/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *                   MPU6050 + Complementary Filter + Kalman Filter (Roll & Pitch)
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
#include "mpu6050.h"
#include "kalman.h"
#include <math.h>
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "mav_telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG        57.2957795f
#define COMP_ALPHA        0.96f   // Bobot gyro pada complementary filter (0.95 - 0.99)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1; // Menggunakan I2C1

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;

// Kalman filter
Kalman_t KalmanRoll, KalmanPitch;
float roll_kalman  = 0.0f;
float pitch_kalman = 0.0f;

// Complementary filter
float roll_comp  = 0.0f;
float pitch_comp = 0.0f;

// Sudut mentah dari akselerometer (pembanding)
float roll_acc  = 0.0f;
float pitch_acc = 0.0f;

uint32_t last_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void); // Prototipe untuk I2C1
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Membelokkan output printf ke jalur USB CDC
int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*) ptr, len);
    return len;
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
  MX_I2C1_Init();       // Memanggil inisialisasi I2C1
  MX_USB_DEVICE_Init(); // Mengaktifkan port USB CDC

  /* USER CODE BEGIN 2 */
  // 1. Inisialisasi komunikasi I2C1 dengan sensor MPU6050
  MPU6050_Init(&hi2c1, &MPU6050);

  // 2. Kalibrasi offset giroskop sebanyak 100 sampel
  // PENTING: Jangan goyangkan sensor saat perangkat baru dinyalakan/di-reset
  MPU6050_Calibrate_Gyro(&hi2c1, &MPU6050, 100);

  // 3. Inisialisasi Kalman filter
  Kalman_Init(&KalmanRoll);
  Kalman_Init(&KalmanPitch);

  // 4. Baca sensor sekali untuk menentukan sudut awal
  MPU6050_Read_All(&hi2c1, &MPU6050);
  roll_acc  = atan2f(MPU6050.ay, MPU6050.az) * RAD_TO_DEG;
  pitch_acc = atan2f(-MPU6050.ax,
                     sqrtf(MPU6050.ay * MPU6050.ay + MPU6050.az * MPU6050.az)) * RAD_TO_DEG;

  Kalman_SetAngle(&KalmanRoll,  roll_acc);
  Kalman_SetAngle(&KalmanPitch, pitch_acc);
  roll_comp  = roll_acc;
  pitch_comp = pitch_acc;
  roll_kalman  = roll_acc;
  pitch_kalman = pitch_acc;

  last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 5. Baca data mentah dari akselerometer, giroskop, dan suhu
    MPU6050_Read_All(&hi2c1, &MPU6050);

    // 6. Hitung dt nyata (detik)
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_tick) / 1000.0f;
    last_tick = now;
    if (dt <= 0.0f) dt = 0.01f;   // pengaman

    // 7. Sudut dari akselerometer (derajat)
    roll_acc  = atan2f(MPU6050.ay, MPU6050.az) * RAD_TO_DEG;
    pitch_acc = atan2f(-MPU6050.ax,
                       sqrtf(MPU6050.ay * MPU6050.ay + MPU6050.az * MPU6050.az)) * RAD_TO_DEG;

    // 8. Complementary filter
    //    angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel
    roll_comp  = COMP_ALPHA * (roll_comp  + MPU6050.gx * dt) + (1.0f - COMP_ALPHA) * roll_acc;
    pitch_comp = COMP_ALPHA * (pitch_comp + MPU6050.gy * dt) + (1.0f - COMP_ALPHA) * pitch_acc;

    // 9. Kalman filter (gyro dalam deg/s)
    roll_kalman  = Kalman_GetAngle(&KalmanRoll,  roll_acc,  MPU6050.gx, dt);
    pitch_kalman = Kalman_GetAngle(&KalmanPitch, pitch_acc, MPU6050.gy, dt);

    // 10. Kirim data sudut via USB CDC dalam format CSV
    printf("ROLL [Acc:%6.2f | Comp:%6.2f | Kal:%6.2f]   PITCH [Acc:%6.2f | Comp:%6.2f | Kal:%6.2f]\r\n",
               roll_acc, roll_comp, roll_kalman,
               pitch_acc, pitch_comp, pitch_kalman);

    // 11. Delay 50 milidetik (~20 Hz refresh rate) agar aliran serial stabil
    HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (96 MHz System / 48 MHz USB)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // 96 MHz SYSCLK
  RCC_OscInitStruct.PLL.PLLQ = 8;             // 48 MHz USB Clock
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1; // Mengatur hardware I2C ke I2C1
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
