/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Author: Adrian Majewski */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Movement/movement.h"
#include "Collision/collision.h"
#include "HCSR04/hcsr04.h"
#include "nRF24MyDesign/NRF24L01.h"
#include "Encoders/encoders.h"
#include "Convert/convert.h"
#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include "RP_LIDAR_A1M8/RP_LIDAR_A1M8.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NRF24_TIME_INTERVAL 10
#define VELOCITY_CAPTURE_TIME_INTERWAL 500
#define ACC_TIME_INTERVAL 100
#define MAG_TIME_INTERVAL 150

#define FIRST_SYNC_BYTE 5
#define SECOND_SYNC_BYTE 25

#define UART_TX_DATA_SIZE 127  
#define UART_RX_DATA_SIZE 104

/* LSM303AGR ACCELEROMETER DEFINES BEGIN */

/* Seven bits address */
#define LSM303AGR_ACCELEROMETER_READ_ADDRESS 0x33 // 0011 0011
#define LSM303AGR_ACCELEROMETER_WRITE_ADDRESS 0x32 // 0011 0010

#define LSM303AGR_ACCELEROMETER_ADDRESS (0x19 << 1) // 0011 001x -> HAL changes first bit itself depending on reading/writting

/* Register addresses */
#define TEMP_CFG_REG_A_ADDRESS 0x1F // 001 1111
#define OUT_TEMP_L_A_ADDRESS 0x0C // 000 1100
#define OUT_TEMP_H_A_ADDRESS 0x0D // 000 1101

#define CTRL_REG1_A_ADDRESS 0x20 // 010 0000
#define CTRL_REG2_A_ADDRESS 0x21 // 010 0001
#define CTRL_REG3_A_ADDRESS 0x22 // 010 0010
#define CTRL_REG4_A_ADDRESS 0x23 // 010 0011
#define CTRL_REG5_A_ADDRESS 0x24 // 010 0100
#define CTRL_REG6_A_ADDRESS 0x25 // 010 0101

/* Register values */
#define TEMP_CFG_REG_A 0xC0 // 1100 0000

#define CTRL_REG5_A_FIFO_ENABLED 0x40 // 0100 0000

#define CTRL_REG1_A_25HzLowPowerModeAndAllAxisEnabled 0x3F // 0011 1111

#define CTRL_REG1_A_25HzNormalModeAndAllAxisEnabled 0x37 // 0011 0111 was

/* 0000 0000 -> 2g, 0001 0000 -> 4g, 0010 0000 -> 8g, 0011 0000 -> 16g */
#define CTRL_REG4_A_2gScale 0x80 // 1000 0000 (before it was 0000 0000 -> 0x00)

/* First OUT register address with 1 at the begining */
#define OUT_X_L_A (0x28 | 0x80) //010 1000

/* LSM303AGR ACCELEROMETER DEFINES END */

/* LSM303AGR MAGNETOMETER DEFINES BEGIN */

/* Seven bits address */
#define LSM303AGR_MAGNETOMETER_READ_ADDRESS 0x3D // 00111101
#define LSM303AGR_MAGNETOMETER_WRITE_ADDRESS 0x3C // 00111100

#define LSM303AGR_MAGNETOMETER_ADDRESS (0x1E << 1) // 0011110x

/* Register addresses */
#define CFG_REG_A_M_ADDRESS 0x60 // 0110 0000
#define CFG_REG_B_M_ADDRESS 0x61 // 0110 0001
#define CFG_REG_C_M_ADDRESS 0x62 // 0110 0010

/* Register values */
#define CFG_REG_A_M 0x00 // 0000 0000
#define CFG_REG_B_M 0x03 // 0000 0011
#define CFG_REG_C_M 0x00  // 0000 0000

/* First OUT register address with 1 at the begining */
#define OUT_X_L_REG_M 0x68 // (0x68 | 0x80)
#define OUT_Y_L_REG_M 0x6A
#define OUT_Z_L_REG_M 0x6C

/* LSM303AGR MAGNETOMETER DEFINES END */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Task variables */
uint8_t TASK_COUNTER = 0;

/* AGM PRIVATE VARIABLES */
uint8_t ACC_Data[6]; // Variable used to read data from accelrometer directly
uint8_t MAG_X_Data[2];
uint8_t MAG_Y_Data[2];
uint8_t MAG_Z_Data[2];
uint8_t GRY_Data[6];
uint8_t TEMP_L;
uint8_t TEMP_H;

/* AGM PRIVATE VARIABLES END */

/* Vehicle state variables */
uint8_t vehicle_state = 0;
uint8_t resetButton = 0;
extern direction previousVehicleState;

/* nRF24 variables */
uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
uint8_t nRF24ReceivedData[50];

/* ESP32 variables */
uint8_t ESP32_transfer_data[UART_TX_DATA_SIZE];

/* Gyroscope variables */

/* Accelerometer variables */
uint8_t THERE_IS_NO_PREVIOUS_ACC_MEASUREMENT = 1;

/* Magnetometer variables */
uint8_t THERE_IS_NO_PREVIOUS_MAG_MEASUREMENT = 1;

/* Tables for converted data */
uint16_t nRF24convertedData[2];
uint8_t hcsr04convertedData[12];
uint8_t angularVelocityConvertedData[4];
uint8_t travel_distanceConvertedData[4];

/* Movement flags */
extern uint8_t currently_going_forward_flag;
extern uint8_t currently_going_backward_flag;
extern uint8_t speed_percantage_variable;

/* HCSR04 variables */
extern uint16_t HCSR04_measurementValues[6];

/* LIDAR variables */
uint8_t LIDARbuffer[5];
uint8_t LIDARbigBuffer[80];
uint8_t bigBufferPosition = 0;
uint8_t LIDARpacketCounter = 0;

/* Limit switch variables */
extern uint8_t limit_switch_table[4];

/* Encoders variables */
float preciseAngularVelocity[2];
extern float numberOfRevoultions[2];
extern float previousNumberOfRevolutions[2];
uint16_t angularVelocity[2];
extern uint16_t travel_distance100[2];

/* Joystick variables */
uint8_t joystickConvertedData[4];

/* Movement variables */
uint8_t max_speed = 25;

/* HAL systick variable */
uint32_t tickstart;
uint32_t velocity_capture_tickstart;
uint32_t ACC_tickstart;
uint32_t MAG_tickstart;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  	/* LIDAR INIT */
  	startLIDARengine(&htim5, TIM_CHANNEL_1, 25);
  	resetLIDAR(&huart6);
  	startScan(&huart6);

  	/* END OF LIDAR INIT */

  	/* Begining of time counting in main loop */
    tickstart = HAL_GetTick();
    velocity_capture_tickstart = HAL_GetTick();
    ACC_tickstart = HAL_GetTick();
    MAG_tickstart = HAL_GetTick();

    /* START TIMERS */
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim4);

    HAL_TIM_Base_Start(&htim10);

    /* Start front right hcsr04 input capture */
    HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

    /* Start generating servo PWM */
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    /* Start generating engines */
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

    /* Start front right HCSR04 trigger */
    HAL_GPIO_WritePin(HCSR04_TRIGGER_FRONT_RIGHT_GPIO_Port, HCSR04_TRIGGER_FRONT_RIGHT_Pin, GPIO_PIN_SET);
    HAL_TIM_Base_Start_IT(&htim11);

    /* Start encoders input capture */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

    /* nRF24 initialization */
    NRF24_Init();
    NRF24_RxMode(RxAddress, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Only one task in one while loop iteration to make every iteration as short as possible */
	  switch(TASK_COUNTER)
	  {
	  case 0:
	  /* ------ / CONTROL VEHICLE TASK BEGIN / ------ */

	  /* Triggers every time NRF24_TIME_INTERVAL passed [miliseconds] */
	  if((HAL_GetTick() - tickstart) > NRF24_TIME_INTERVAL){

	  /* Check if there is any data received via nRF24L0+ */
	  if(isDataAvailable(2)==1)
	  {
	  NRF24_Receive(nRF24ReceivedData);

	  /* Uncomment to enable orange diode blink if data received via nRF24L0+ */
	  //HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);

	  /* Convert received data from nRF24L0+ */
      convertU8tableTo16table(nRF24convertedData, nRF24ReceivedData, 2, 4);

	  /* Set max PWM recieved from controller */
      max_speed = nRF24ReceivedData[4];

	  /* Set reset state */
	  resetButton = nRF24ReceivedData[5];

	  /* Check resetButton state */
	  resetLimitSwitchFalgs(resetButton);

	  /* Pass data to remote control function */
	  remoteControl(nRF24convertedData, previousVehicleState, max_speed);

	  }
	  tickstart += NRF24_TIME_INTERVAL;
	  }
	  /* ------ / CONTROL VEHICLE TASK END / ------ */
	  TASK_COUNTER++;
	  break;

	  case 1:
	  /* ------ / VEHICLE VELOCITY CAPTURE TASK BEGIN / ------ */

	  /* Triggers every time VELOCITY_CAPTURE_TIME_INTERWAL passed */
	  if((HAL_GetTick() - velocity_capture_tickstart) > VELOCITY_CAPTURE_TIME_INTERWAL){

	  /* Calculate angular velocity for both wheels, revolutions per second */
	  preciseAngularVelocity[0] = (float) (numberOfRevoultions[0] - previousNumberOfRevolutions[0]);
	  preciseAngularVelocity[1] = (float) (numberOfRevoultions[1] - previousNumberOfRevolutions[1]);

	  /* Set previous number of revolutions */
	  previousNumberOfRevolutions[0] = numberOfRevoultions[0];
	  previousNumberOfRevolutions[1] = numberOfRevoultions[1];

	  /* Change from [rev/s] to [rev/min] */
	  preciseAngularVelocity[0] = preciseAngularVelocity[0] * 60 * 2;
	  preciseAngularVelocity[1] = preciseAngularVelocity[1] * 60 * 2;

	  /* Multiply by 10 to save one digit after coma */
	  float preciseAngularVelocity10[2];
	  preciseAngularVelocity10[0] = preciseAngularVelocity[0] * 10;
	  preciseAngularVelocity10[1] = preciseAngularVelocity[1] * 10;

	  /* Save the angular velocity in uint16_t so it can be converted and send via UART */
	  angularVelocity[0] = preciseAngularVelocity10[0];
	  angularVelocity[1] = preciseAngularVelocity10[1];

	  velocity_capture_tickstart+= VELOCITY_CAPTURE_TIME_INTERWAL;
	  }
	  /* ------ / VEHICLE VELOCITY CAPTURE TASK END / ------ */
	  TASK_COUNTER++;
	  break;

	  /* ----- / NEW LSM303AGR ACCELEROMETER MEASUREMENT TASK BEGIN / ----- */
	  case 2:
		  if(THERE_IS_NO_PREVIOUS_ACC_MEASUREMENT == 1){

			  uint8_t Settings = CTRL_REG1_A_25HzNormalModeAndAllAxisEnabled;

			  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ACCELEROMETER_ADDRESS, CTRL_REG1_A_ADDRESS, 1, &Settings, 1, 100);

			  /* TEMP Sensor */
			  Settings = TEMP_CFG_REG_A;
			  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ACCELEROMETER_ADDRESS, TEMP_CFG_REG_A_ADDRESS, 1, &Settings, 1, 100);

			  Settings = CTRL_REG4_A_2gScale;

			  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ACCELEROMETER_ADDRESS, CTRL_REG4_A_ADDRESS, 1, &Settings, 1, 100);

			  Settings = CTRL_REG5_A_FIFO_ENABLED;

			  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ACCELEROMETER_ADDRESS, CTRL_REG5_A_ADDRESS, 1, &Settings, 1, 100);

			  /* Toogle the flag */
			  THERE_IS_NO_PREVIOUS_ACC_MEASUREMENT = 0;

		  } else {

			  if((HAL_GetTick() - ACC_tickstart) > ACC_TIME_INTERVAL){
			  HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ACCELEROMETER_ADDRESS, OUT_X_L_A, 1, ACC_Data, 6, 100);
			  /* Get temperature value */
			  HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ACCELEROMETER_ADDRESS, OUT_TEMP_L_A_ADDRESS, 1, &TEMP_L, 1, 100);
			  HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ACCELEROMETER_ADDRESS, OUT_TEMP_H_A_ADDRESS, 1, &TEMP_H, 1, 100);
			  ACC_tickstart += ACC_TIME_INTERVAL;
			  }
		  }

	  /* ----- / NEW LSM303AGR ACCELEROMETER MEASUREMENT TASK END / ----- */
	  TASK_COUNTER++;
	  break;

	  case 3:
	  /* ----- / NEW LSM303AGR MAGNETOMETER MEASUREMENT TASK BEGIN / ----- */
	  if(THERE_IS_NO_PREVIOUS_MAG_MEASUREMENT == 1){

		  uint8_t Settings = CFG_REG_A_M;

		  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_MAGNETOMETER_ADDRESS, CFG_REG_A_M_ADDRESS, 1, &Settings, 1, 100);

		  Settings = CFG_REG_B_M;

		  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_MAGNETOMETER_ADDRESS, CFG_REG_B_M_ADDRESS, 1, &Settings, 1, 100);

		  Settings = CFG_REG_C_M;

		  HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_MAGNETOMETER_ADDRESS, CFG_REG_C_M_ADDRESS, 1, &Settings, 1, 100);

		  /* Toogle the flag */
		  THERE_IS_NO_PREVIOUS_MAG_MEASUREMENT = 0;

	  } else {

		  if((HAL_GetTick() - MAG_tickstart) > MAG_TIME_INTERVAL){
		  HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_MAGNETOMETER_ADDRESS, OUT_X_L_REG_M, 1, MAG_X_Data, 2, 100);
		  HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_MAGNETOMETER_ADDRESS, OUT_Y_L_REG_M, 1, MAG_Y_Data, 2, 100);
		  HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_MAGNETOMETER_ADDRESS, OUT_Z_L_REG_M, 1, MAG_Z_Data, 2, 100);
		  MAG_tickstart += MAG_TIME_INTERVAL;
		  }

	  }
	  /* ----- / NEW LSM303AGR MAGNETOMETER MEASUREMENT TASK END / ----- */
	  TASK_COUNTER++;
	  break;

	  case 4:

	  /* ------ / DATA CONVERSION TASK BEGIN / ------ */

	  /* Convert HCSR04 data to uint8_t table */
	  convertU16tableToU8table(HCSR04_measurementValues, hcsr04convertedData, 6, 12);

	  /* Convert travel distance data to uint8_t table */
	  convertU16tableToU8table(travel_distance100, travel_distanceConvertedData, 2, 4);

	  /* Convert angular velocity data to uint8_t table */
	  convertU16tableToU8table(angularVelocity, angularVelocityConvertedData, 2, 4);

	  /* Assign joystick data to uint8_t table */
      for(int i = 0; i < 4; i++)
	  {
	  joystickConvertedData[i] = nRF24ReceivedData[i];
	  }
	  /* ------ / DATA CONVERSION TASK END / ------ */
	  TASK_COUNTER++;
	  break;

	  case 5:
	  /* ------ / ESP32 ASSIGN DATA PACKET TASK BEGIN / ------ */

	  /* First sign of a message is '*' */
	  ESP32_transfer_data[0] = 42;

	  /* Assign HCSR04 data the ESP32 table */
	  uint8_t pointer_for_hcsr04_table = 0;
	  for(int i = 1; i < 13; i++)
	  {
		  ESP32_transfer_data[i] = hcsr04convertedData[pointer_for_hcsr04_table];
		  pointer_for_hcsr04_table++;
	  }

	  /* Assign LIDAR data to the ESP32 table */
	  uint8_t pointer_for_LIDAR_table = 0;
	  for(int i = 13; i < 93; i++)
	  {
		  ESP32_transfer_data[i] = LIDARbigBuffer[pointer_for_LIDAR_table];
		  pointer_for_LIDAR_table++;
	  }

	  /* Assign temperature data */
	  ESP32_transfer_data[93] = TEMP_L;
	  ESP32_transfer_data[94] = TEMP_H;

	  /* Assign limit switch data to the ESP32 table */
	  uint8_t pointer_for_limit_switch_table = 0;
	  for(int i = 97; i < 101; i++)
	  {
		  ESP32_transfer_data[i] = limit_switch_table[pointer_for_limit_switch_table];
		  pointer_for_limit_switch_table++;
	  }

	  /* Assign max speed and vehicle state variables to ESP32 table */
	  ESP32_transfer_data[101] = max_speed;
	  ESP32_transfer_data[102] = vehicle_state;

	  /* Assign travel distance data to the ESP32 table */
	  uint8_t travel_distance_data_pointer = 0;
	  for(int i = 103; i < 107; i++)
	  {
		  ESP32_transfer_data[i] = travel_distanceConvertedData[travel_distance_data_pointer];
		  travel_distance_data_pointer++;
	  }

	  /* Assign movement flags data to the ESP32 table */
      ESP32_transfer_data[107] = currently_going_forward_flag;
	  ESP32_transfer_data[108] = currently_going_backward_flag;
	  ESP32_transfer_data[109] = speed_percantage_variable;

	  /* Assign angular velocity data to the ESP32 table */
      uint8_t pointer_for_angular_velocity_table = 0;
	  for(int i = 110; i < 114; i++)
	  {
		 ESP32_transfer_data[i] = angularVelocityConvertedData[pointer_for_angular_velocity_table];
		 pointer_for_angular_velocity_table++;
	  }

	  /* Assign magnetometer data to the ESP32 table */
	  uint8_t pointer_for_magnetometer_table = 0;
	  uint8_t MAG_Data_To_Send[6];
	  MAG_Data_To_Send[0] = MAG_X_Data[0];
	  MAG_Data_To_Send[1] = MAG_X_Data[1];
	  MAG_Data_To_Send[2] = MAG_Y_Data[0];
	  MAG_Data_To_Send[3] = MAG_Y_Data[1];
	  MAG_Data_To_Send[4] = MAG_Z_Data[0];
	  MAG_Data_To_Send[5] = MAG_Z_Data[1];
	  for(int i = 114; i < 120; i++)
	  {
	  	 ESP32_transfer_data[i] = MAG_Data_To_Send[pointer_for_magnetometer_table];
	  	 pointer_for_magnetometer_table++;
	  }

	  /* Assign accelerometer data to the ESP32 table */
	  uint8_t pointer_for_accelerometer_table = 0;
	  uint8_t ACC_Data_To_Send[6];
      ACC_Data_To_Send[0] = ACC_Data[0];
	  ACC_Data_To_Send[1] = ACC_Data[1];
	  ACC_Data_To_Send[2] = ACC_Data[2];
	  ACC_Data_To_Send[3] = ACC_Data[3];
	  ACC_Data_To_Send[4] = ACC_Data[4];
	  ACC_Data_To_Send[5] = ACC_Data[5];
	  for(int i = 120; i < 126; i++)
      {
		 ESP32_transfer_data[i] = ACC_Data_To_Send[pointer_for_accelerometer_table];
	     pointer_for_accelerometer_table++;
      }

	  /* Assign unreserved data 3 uint8_t */

      /* Last byte of the message is '#' */
	  ESP32_transfer_data[UART_TX_DATA_SIZE - 1] = '#';


	  /* ------ / ESP32 ASSIGN DATA PACKET TASK END / ------ */
	  TASK_COUNTER++;
	  break;

	  case 6:
	  /* ------ / ESP32 SEND DATA TASK BEGIN / ------ */

	  /* Send prepared packet via UART in IT mode */
	  HAL_UART_Transmit_DMA(&huart2, ESP32_transfer_data, UART_TX_DATA_SIZE);
	  /* ------ / ESP32 SEND DATA TASK END / ------ */
	  TASK_COUNTER++;
	  break;

	  default:
	  TASK_COUNTER = 0;
	  break;
}

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* TIM1_TRG_COM_TIM11_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	hcsr04Measurement(htim);
	measureRevolutionNumber(htim);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	limitSwitch(GPIO_Pin);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART6){

			for(int i = 0; i < 5; i++){
				LIDARbigBuffer[bigBufferPosition] = LIDARbuffer[i];
				bigBufferPosition++;
			}

			LIDARpacketCounter++;

			if(LIDARpacketCounter == 16){
				LIDARpacketCounter = 0;
				bigBufferPosition = 0;
			}

			receiveOneMeasurement(&huart6, LIDARbuffer);

			}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM11){
		hcsr04Trigger();
	}
  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
