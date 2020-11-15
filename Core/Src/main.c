/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_SIZE                        128
#define START_STATE                       ONE_PHASE_INIT
#define ADC1_CONVERTEDVALUES_BUFFER_SIZE  ((uint32_t)   2)
#define ADC2_CONVERTEDVALUES_BUFFER_SIZE  ((uint32_t)   5)
#define VREF_MV                           2500
#define VOUT_DIV                          20
#define VOUT_MAX_MV                       50000
#define VOUT_SAMPLE_DEPTH                 8
#define HRTIM_PERIOD                      0x3fff
#define MAX_DUTY                          HRTIM_PERIOD/2
#define KP                                30

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Mode_TypeDef state = ONE_PHASE_INIT;
__IO uint32_t adc1_converted_values[ADC1_CONVERTEDVALUES_BUFFER_SIZE];
__IO uint32_t adc2_converted_values[ADC2_CONVERTEDVALUES_BUFFER_SIZE];
uint32_t vout_sample_buf[VOUT_SAMPLE_DEPTH];
volatile int vout_sample_ind      = 0;
volatile uint32_t vout_sample_acc = 0;
volatile uint32_t vout_mv         = 0;
volatile uint32_t vset_mv         = 0;
volatile int verr_mv              = 0;
volatile int duty_cycle           = 0;
uint8_t debug_buf[DEBUG_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
void LED_on(int led);
void LED_off(int led);
void LED_toggle(int led);
void add_sample(uint32_t sample, uint32_t* buf, int* sample_ind, int depth);

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
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  state = START_STATE;

  /* ------------ */
  /* ADC start-up */
  /* ------------ */
  // ADC1
  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK){
    state = FAULT;
  }
  HAL_ADC_Start_DMA(&hadc1, adc1_converted_values, ADC1_CONVERTEDVALUES_BUFFER_SIZE);
  HAL_ADC_Start_IT(&hadc1);

  // ADC2
  if(HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK){
    state = FAULT;
  }
  HAL_ADC_Start_DMA(&hadc2, adc2_converted_values, ADC2_CONVERTEDVALUES_BUFFER_SIZE);
  HAL_ADC_Start_IT(&hadc2);

  /* -------------- */
  /* HRTIM start-up */
  /* -------------- */
  // enable outputs A1, B1, C1, D1
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);

  // start timer master, A, B, C, D
  HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // DETECT ENABLE SIGNAL
    if(HAL_GPIO_ReadPin(ENABLE_GPIO_Port, ENABLE_Pin) == GPIO_PIN_RESET){
      state = ONE_PHASE_INIT;
    }else{
      state = OUTPUT_OFF;
    }

    // DEBUG OUTPUT
    snprintf(debug_buf, DEBUG_SIZE, "Vout(mV): %d\r\n", vout_mv);


    HAL_Delay(50);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  // is called by the ADC DMA ConvCplt callback
  if(hadc->Instance == ADC1){
    // pertains to the voltage or current measurements
    // add voltage measurment
    // add_sample(adc1_converted_values[0], vout_sample_buf, &vout_sample_ind, VOUT_SAMPLE_DEPTH);
    vout_sample_acc += adc1_converted_values[0];
    vout_sample_ind++;
    if(vout_sample_ind == VOUT_SAMPLE_DEPTH - 1){
      // buffer filled with new data, time to refactor duty cycle
      vout_sample_ind = 0;

      // calculate Vout in mv
      vout_mv = vout_sample_acc * VREF_MV * VOUT_DIV;
      vout_mv = vout_mv / VOUT_SAMPLE_DEPTH;
      vout_mv = vout_mv >> 12; // divide by 4096, as our ADC is 12 bit
      vout_sample_acc = 0;

      if(state == OUTPUT_OFF){
        return;
      }
      // PI-esque feedback loop
      verr_mv = vset_mv - vout_mv;
      duty_cycle += (verr_mv / KP);

      // BEGIN STATE MACHINE
      switch (state)
      {
      case OUTPUT_OFF:
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);
        break;

      case ONE_PHASE_INIT:
        
        break;
      
      case TWO_PHASE_INIT:
        /* code */
        break;
      
      case THREE_PHASE_INIT:
        /* code */
        break;
      
      case FOUR_PHASE_INIT:
        /* code */
        break;
      
      case ONE_PHASE:
        /* code */
        break;
      
      case TWO_PHASE:
        /* code */
        break;
      
      case THREE_PHASE:
        /* code */
        break;
      
      case FOUR_PHASE:
        /* code */
        break;
      
      case BURST_INIT:
        /* code */
        break;
      
      case BURST:
        /* code */
        break;
      
      default:
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);
        break;
      }
      // END STATE MACHINE
    }
  }else if(hadc->Instance == ADC2){
    // pertains to all the other stuff

    // calculate the setpoint
    vset_mv = adc2_converted_values[4] * VREF_MV;
    vset_mv = vset_mv >> 12;
    vset_mv = map(vset_mv, 0, VREF_MV, 0, VOUT_MAX_MV);
  }
}

void add_sample(uint32_t sample, uint32_t* buf, int* sample_ind, int depth){
  *sample_ind++;
  if(*sample_ind >= depth){
    *sample_ind = 0;
  }
  buf[*sample_ind] = sample;
}


uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void LED_on(int led){
  switch (led)
  {
  case 0:
    HAL_GPIO_WritePin(STAT_0_GPIO_Port, STAT_0_Pin, GPIO_PIN_SET);
    break;
  case 1:
    HAL_GPIO_WritePin(STAT_1_GPIO_Port, STAT_1_Pin, GPIO_PIN_SET);
    break;
  case 2:
    HAL_GPIO_WritePin(STAT_2_GPIO_Port, STAT_2_Pin, GPIO_PIN_SET);
    break;
  default:
    break;
  }
}
void LED_off(int led){
switch (led)
  {
  case 0:
    HAL_GPIO_WritePin(STAT_0_GPIO_Port, STAT_0_Pin, GPIO_PIN_RESET);
    break;
  case 1:
    HAL_GPIO_WritePin(STAT_1_GPIO_Port, STAT_1_Pin, GPIO_PIN_RESET);
    break;
  case 2:
    HAL_GPIO_WritePin(STAT_2_GPIO_Port, STAT_2_Pin, GPIO_PIN_RESET);
    break;
  default:
    break;
  }
}
void LED_toggle(int led){
switch (led)
  {
  case 0:
    HAL_GPIO_TogglePin(STAT_0_GPIO_Port, STAT_0_Pin);
    break;
  case 1:
    HAL_GPIO_TogglePin(STAT_1_GPIO_Port, STAT_1_Pin);
    break;
  case 2:
    HAL_GPIO_TogglePin(STAT_2_GPIO_Port, STAT_2_Pin);
    break;
  default:
    break;
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
