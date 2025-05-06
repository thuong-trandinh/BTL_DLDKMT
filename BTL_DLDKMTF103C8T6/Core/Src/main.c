/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "SPI.h"
#include "Frame.h"
#include <math.h>
#include <motor.h>
#include <string.h>
#include "eprrom.h"

#include"motor.h"
#include"setting_PID.h"
#include"PID.h"
#include"reset_data.h"
#include"read_tick.h"
#include"process_read_tick.h"
#include"control_motor.h"
#include"interrupt.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MAX_PWM 255
float uk = 254;
//tick_engine_rotation = tick_encoder_rotation*gear_ratio = 11*34 => read on 1 channel
#define PRES_TIM2 799 		// timer_clk = 0.1ms => T_period = 0.1*255 = 25.5 ms => f = 39.21 hz
/*SET UP TIMER 3*/
#define PRES_TIM3	7999 	// => timer_clk = 0.1 ms
#define resolution  1000 	// resolution = T / timer_clk = 10 ms / 0.1 ms = 100 hz
uint16_t counter_period = resolution - 1;
volatile uint8_t flag_10ms = 0,flag_1ms = 0,flag_1s = 0;


/*PID PARAMETER*/
double kp,ki,kd;
float output_val;
float setpoint_val;
uint8_t flag_start = 0,flag = 0;
float current_speed = 0;
uint16_t current_time,pre_time;
float rev_1m,rev_1s,tick_1s;
int16_t current_tick = 0,pre_tick = 0,now_tick = 0;
uint16_t tick_rpm = 0;

float out_pwm = 0;



frame_t transmit;
uint8_t RX_data[100];
uint8_t *tran_data;
uint16_t frameSize = 0;
uint8_t is_transmitting = 0;
uint8_t decode_payload[100] ;
uint8_t check_da[100] = {0};
uint8_t check_re[100] = {0};
uint8_t decode_frame[100];
uint8_t check_flag_complete_transmit = 0;
float position[4] = {0};
uint8_t flag_pos_mode = 0;
uint8_t floattobyte[4] = {0};
uint8_t trans_data[5000] = {0};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim1 && is_transmitting == 0 && tran_data != NULL) {
	        is_transmitting = 1;  // Đánh dấu đang truyền
	        HAL_UART_Transmit_DMA(&huart2, tran_data,transmit.length + 7);
	}
	if(htim == &htim3){
		flag_10ms = 1;
	}
}
void Xuat_PWM_1(float Duty_Cycle)
{
	Duty_Cycle = Duty_Cycle*__HAL_TIM_GET_AUTORELOAD(&htim2)/100;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Duty_Cycle);
}
void test_rev_1m(){
			//cach 1
	  	  	/*current_time = HAL_GetTick();
		  	if(current_time - pre_time >= 1000){
		  		current_tick = __HAL_TIM_GET_COUNTER(&htim1);
		  		tick_1s = current_tick - pre_tick;
		  	    rev_1s = tick_1s / tick_per_revolution;
		  	    rev_1m = rev_1s * 60;
		  	    pre_tick = current_tick;
		  	    pre_time = current_time;
		  	 }*/
		  	//cach 2
			if(flag_1s == 1){
				current_tick = __HAL_TIM_GET_COUNTER(&htim1);
				tick_1s = current_tick - pre_tick;
				rev_1s = tick_1s / 999;
				rev_1m = rev_1s * 60;
				pre_tick = current_tick;
				flag_1s = 0;
			}
		  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);
		     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET);

		     //Xuat_PWM_1(&htim2,TIM_CHANNEL_1,25);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart != &huart2) return;

	    // nếu là half‑transfer (Size gần bằng half buffer) thì bỏ qua
	if (Size < 7) {
	        // re‑arm DMA và exit
	    HAL_UARTEx_ReceiveToIdle_DMA(huart, RX_data, sizeof(RX_data));
	    return;
	}
	check_flag_complete_transmit = 1;
	//transmit.length = floorf(5000/rx_data)*4; // so mau can lay
	//SendADCValue(&transmit); // sua cai nay
//	frameSize = 7 + transmit.length*4; //
//	tran_data = calloc(frameSize,sizeof(uint8_t));
//	packframe(tran_data, &transmit, 0x20);
//	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX_data, 100);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        free(tran_data);
        is_transmitting = 0;
    }
}
uint8_t check_data[5]={0};

motor_name motor;

// global hoặc static trong file
static uint16_t enc_last_cnt;
static int32_t  enc_accum_cnt;
uint8_t flag_velocity_0x70 = 0;
uint8_t flag_velocity_0x71 = 0;
uint8_t flag_velocity_0x72 = 0;
int16_t velocity = 0;
uint8_t duty = 0;
void Quay_Thuan()
{
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
}
void Quay_Nghich()
{
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_14, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
}
void Ngung()
{
	HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
}
// global (hoặc static ở đầu file)
static uint16_t prev_cnt = 0;    // lưu giá trị counter lần trước
const float dt = 0.01f;          // 10 ms
const float ticks_per_rev = 1600; // nếu 1600 tick = 1 vòng
float calculate_rpm(void)
{
    // đọc counter 16‑bit
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim3);

    // tính delta (có bù tràn tự động khi ép sang int16_t)
    int16_t diff = (int16_t)(cnt - prev_cnt);

    // cập nhật prev_cnt cho lần sau
    prev_cnt = cnt;

    // tính RPM: (số vòng = diff/ticks_per_rev), nhân 60/dt
    return (diff / ticks_per_rev) * (60.0f / dt);
}
float velocity_1;
float vel[4] = {0};
// gọi 1 lần sau khi start encoder

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  init_motor(&motor,GPIOB,GPIO_PIN_14,GPIOB,GPIO_PIN_15,TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX_data, 100);
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1|TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
  uint8_t value[8]={0};
  Send_Value(value);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Quay_Thuan();
	  		  		Xuat_PWM_1(100);
	  if(check_flag_complete_transmit == 1){
		  uint16_t addr = 0;
		  	uint8_t bytes = 0;
		  	//uint32_t rx_data = 0;
		  	Decode_frame(RX_data, &transmit);
		  	switch(transmit.command){
		  	case 0x21:
		  		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		  		break;
		  	case 0x22:
		  		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
		  		break;
		  	case 0x23:
		  		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
		  		break;
		  	case 0x30:
		  		bytes = transmit.length - 3;
		  		Decode_Payload(decode_payload, &transmit, &addr, &bytes);
		  		memcpy(check_da,decode_payload,bytes);
		  		EEPRROM_WriteBuffer(&hi2c1,addr,decode_payload,bytes);
		  		break;
		  	case 0x31:
		  		bytes = transmit.length - 3;
		  		Decode_Payload(decode_payload,&transmit,&addr,&bytes);
		  		EEPRROM_ReadBuffer(&hi2c1,addr,decode_payload,bytes);
		  		tran_data = calloc(frameSize,sizeof(uint8_t));
		  		transmit.length = bytes;
		  		packframe(tran_data, &transmit, decode_payload, 0x31);
		  		memcpy(check_re,tran_data,transmit.length + 7);
		  		HAL_UART_Transmit_DMA(&huart2, tran_data, transmit.length + 7);
		  		break;
		  	case 0x40:
		  		Send_Value(transmit.payload);
		  		break;
		  	case 0x60:  // control position
		  		Decode_Position(position, &transmit);
		  		kp = position[1];
		  		ki = position[2];
		  		kd = position[3];
		  		motor.setpoint_val = position[0];   // lấy mục tiêu từ payload
		  		// reset lịch sử lỗi & output để P tính ngay lập tức
		  		reset_data(&motor);
		  		flag_start   = 1;
		  		flag_pos_mode = 1;// bật chế độ PID
		  		break;
		  	case 0x70:
		  		Quay_Thuan();
		  		Xuat_PWM_1(transmit.payload[0]);
		  		flag_velocity_0x70 = 1;
		  		break;
		  	case 0x71:
		  		Quay_Nghich();
		  		Xuat_PWM_1(transmit.payload[0]);
		  		flag_velocity_0x71 = 1;
		  		break;
		  	case 0x72:
		  		Ngung();
		  		Xuat_PWM_1(transmit.payload[0]);
		  		flag_velocity_0x72 = 1;
		  	case 0x80:
		  		flag_velocity_0x70 = 0;
		  		flag_velocity_0x71 = 0;
		  		flag_velocity_0x72 = 0;
		  		flag_start = 0;
		  		break;
		  	}


		  	check_flag_complete_transmit = 0;
	  }
	  if (flag_10ms) {
		  output_val = calculate_current_angle(&motor);
		  if (flag_start) {
			  motor.kp = kp;
			  motor.ki = ki;
			  motor.kd = kd;
			  out_pwm = PID_Calculation(&motor, motor.setpoint_val, output_val);
			  control_motor_status(&motor,out_pwm);
			  // nếu đã sát đích, tắt điều khiển
			  if(fabs(motor.setpoint_val - output_val) < 2.0f) {
				  flag_start = 0;
				  flag_pos_mode = 0;
				  // và có thể dừng motor hẳn:
				  control_motor_status(&motor, 0);
			      }
		  }else{
			  out_pwm = 0;
			  output_val = 0;
			  PID_INIT_PARAM(&motor, setpoint_val, motor.kp, ki, kd);
		  }
		  if(flag_pos_mode && !is_transmitting){
			  transmit.length = 4;

			  FloatToBytes(output_val, floattobyte);
			  packframe(trans_data, &transmit, floattobyte, 0x60);
			  memcpy(check_da,trans_data,sizeof(uint8_t));
			  HAL_UART_Transmit_DMA(&huart2, trans_data, transmit.length + 7);
		  }
		  if(flag_velocity_0x70){
			  velocity = calculate_rpm();
			  FloatToBytes(velocity_1,vel);
			  packframe(trans_data,&transmit,vel,0x70);
			  HAL_UART_Transmit_DMA(&huart2, trans_data, 7 + transmit.length);
		  }
		  if(flag_velocity_0x71){
			  velocity = calculate_rpm();
			  FloatToBytes(velocity_1,vel);
			  packframe(trans_data, &transmit,vel,0x70);
			  HAL_UART_Transmit_DMA(&huart2, trans_data, 7 + transmit.length);
		  }
		  if(flag_velocity_0x72){
			  velocity = calculate_rpm();
			  FloatToBytes(velocity_1,vel);
			  packframe(trans_data, &transmit,vel,0x70);
			  HAL_UART_Transmit_DMA(&huart2, trans_data, 7 + transmit.length);
		  }
		  flag_10ms = 0;
	  }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Instance = I2C1;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 PB3 PB4
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
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
