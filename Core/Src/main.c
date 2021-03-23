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
#include "hrtim.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_SIZE                        128
#define START_STATE                       OUTPUT_OFF
#define ADC1_CONVERTEDVALUES_BUFFER_SIZE  ((uint32_t)   2)
#define ADC2_CONVERTEDVALUES_BUFFER_SIZE  ((uint32_t)   5)
#define VREF_MV                           2500
#define VOUT_DIV                          20
#define VOUT_MAX_MV                       50000
#define VOUT_SAMPLE_DEPTH                 4
#define HRTIM_PERIOD                      0x3fff
#define BURST_DUTY_CYCLE                  HRTIM_PERIOD/40
#define MAX_DUTY                          ((HRTIM_PERIOD/2) - 10)
#define MIN_DUTY                          128
#define ADD_PHASE_THRESH                  HRTIM_PERIOD/8
#define ADD_PHASE_CURR_THRESH             2000
#define SHED_PHASE_THRESH                 HRTIM_PERIOD/20
#define SHED_PHASE_CURR_THRESH            1500
#define KP_1                              4
#define KP_2                              KP_1
#define KP_3                              KP_1
#define KP_4                              KP_1
#define KP_LIGHT_LOAD                     100
#define KI_1                              250
#define KI_LIGHT_LOAD                     25000
#define KD_1                              2
#define VINTEG_MAX                        HRTIM_PERIOD*KI_1
#define KP                                30
#define SS_SLOPE_V_PER_S                  50
#define IOUT_SAMPLE_DEPTH                 8
#define IPHASE_SAMPLE_DEPTH               8
#define DEFAULT_V_SET                     12000
#define ALLOWABLE_ERROR                   10

#define VERR_RUNNING_MEAN_SIZE            4

#define I2C_DATA_SIZE                     8
#define I2C_CMD_SET_VOUT                  0x01
#define I2C_CMD_ENABLE_OUTPUT             0x02
#define I2C_CMD_DISABLE_OUTPUT            0x03

/* gains in counts-per-A */
#define IGAIN3                            123
#define IGAIN2                            246
#define IGAIN1                            492
#define IGAIN0                            3686

#define CLAMP(x, a, b)                    (x < a ? a : (x > b ? b : x))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Mode_TypeDef state = OUTPUT_OFF;
__IO uint32_t adc1_converted_values[ADC1_CONVERTEDVALUES_BUFFER_SIZE];
__IO uint32_t adc2_converted_values[ADC2_CONVERTEDVALUES_BUFFER_SIZE];
volatile bool status_update       = false;
uint32_t ss_tick                  = 0;
volatile int n_phases             = 1; // number of phases enabled
uint32_t vout_sample_buf[VOUT_SAMPLE_DEPTH];
volatile int kp                   = KP_1;
volatile int ki                   = KI_1;
volatile int kd                   = KD_1;
volatile int vinteg               = 0;
volatile int vdelta               = 0;
volatile int vout_sample_ind      = 0;
volatile uint32_t vout_sample_acc = 0;
volatile int verr_acc             = 0;
volatile uint32_t verr_history[VERR_RUNNING_MEAN_SIZE] = {};
volatile int verr_rm_ind          = 0;
volatile uint32_t vout_offset_mv  = 2000;
volatile uint32_t vout_mv         = 0;
volatile uint32_t vset_mv         = 0;
volatile uint32_t target_vset_acc = 0;
volatile int target_vset_count    = 0;
volatile uint32_t target_vset_mv  = DEFAULT_V_SET;
volatile int verr_mv              = 0;
volatile int duty_cycle           = 0;
volatile bool is_enabled          = false;
volatile int i_gain_setting       = 0; // 0 thru 3
volatile uint32_t i_gain          = 0; // i sense gain in counts-per-A
volatile uint32_t iout_ma         = 0;
volatile uint32_t iout_offset_ma  = 0;
volatile uint32_t iout_sample_acc = 0;
volatile int iout_sample_count    = 0;
volatile uint32_t iphase_accs[4]  = {};
volatile uint32_t iphase_meas[4]  = {};
volatile int iphase_sample_count  = 0;
int debug_a_size = 0;
int debug_b_size = 0;
int debug_buf_to_use = 0;
uint8_t debug_buf[DEBUG_SIZE];
uint8_t debug_buf_a[DEBUG_SIZE];
uint8_t debug_buf_b[DEBUG_SIZE];
volatile bool i2c_enabled_status = false;
volatile bool i2c_cmd_rcvd = false;
uint8_t i2c_data[I2C_DATA_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void goto_state(uint8_t next_state);
void set_i_gain(uint8_t setting);
void refactor_duty_cycle();
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
void LED_on(int led);
void LED_off(int led);
void LED_toggle(int led);
void add_sample(uint32_t sample, uint32_t* buf, int* sample_ind, int depth);
void disable_adc_interrupts();
void enable_adc_interrupts();
void disable_hrtim_interrupts();
void enable_hrtim_interrupts();
void debug(char* str);

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
  i_gain_setting = 0;
  set_i_gain(i_gain_setting);

  /* ------------ */
  /* ADC start-up */
  /* ------------ */
  // ADC1
  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK){
    debug("ADC1 calibration failed. Entering FAULT state.");
    state = FAULT;
  }
  HAL_ADC_Start_DMA(&hadc1, adc1_converted_values, ADC1_CONVERTEDVALUES_BUFFER_SIZE);
  HAL_ADC_Start(&hadc1);

  // ADC2
  if(HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK){
    debug("ADC2 calibration failed. Entering FAULT state.");
    state = FAULT;
  }
  HAL_ADC_Start_DMA(&hadc2, adc2_converted_values, ADC2_CONVERTEDVALUES_BUFFER_SIZE);
  HAL_ADC_Start(&hadc2);

  /* -------------- */
  /* HRTIM start-up */
  /* -------------- */
  // enable outputs A1, B1, C1, D1
  // HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);

  // start timer master, A, B, C, D
  HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);

  /* ------------------ */
  /* I2C Slave start-up */
  /* ------------------ */
  HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_data, 1); // receive a 1-byte command

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // DETECT ENABLE SIGNAL
    GPIO_PinState enable_pin = HAL_GPIO_ReadPin(ENABLE_GPIO_Port, ENABLE_Pin);
    if(enable_pin == GPIO_PIN_RESET && !is_enabled){
      debug("Enabling output\r\n");
      LED_on(0);
      is_enabled = true;
      vset_mv = 0;
      ss_tick = HAL_GetTick();
      duty_cycle = MIN_DUTY;
      state = ONE_PHASE_INIT;
    }else if(enable_pin == GPIO_PIN_SET && is_enabled){
      debug("Disabling output\r\n");
      LED_off(0);
      is_enabled = false;
      state = OUTPUT_OFF;
    }

    // main loop state machine
    // mostly for controlling vset_mv
    switch(state){
      case ONE_PHASE_INIT:
      case TWO_PHASE_INIT:
      case THREE_PHASE_INIT:
      case FOUR_PHASE_INIT:
      case ONE_PHASE:
      case TWO_PHASE:
      case THREE_PHASE:
      case FOUR_PHASE:
      case BURST:
      {
        // change vset_mv at a controlled rate

        uint32_t dt = HAL_GetTick() - ss_tick;
        int dv = SS_SLOPE_V_PER_S * dt; // maximum voltage step to take
        int diff = (int)target_vset_mv - (int)vset_mv; // actual difference to make up

        if(diff > dv){
          // increment by maximum step
          vset_mv += dv;
        }else{
          // just add the difference, since its magnitude is less than max step (dv)
          vset_mv += diff;
        }
        
        ss_tick += dt;
        break;
      }
      case FAULT:{
        // LED 2 SOLID
        LED_on(2);
        break;
      }default:{
        break;
      }
    };

    // PHASE CURRENT CALCULATION
    if(iphase_sample_count >= IPHASE_SAMPLE_DEPTH){
      for(int i = 0; i < n_phases; i++){
        iphase_meas[i] = ((iphase_accs[i] * VREF_MV) / iphase_sample_count) >> 12; // millivolts at the ADC input
        iphase_accs[i] = 0;
      }
      iphase_sample_count = 0;
    }

    // DEBUG OUTPUT
    if(HAL_GetTick()/250 % 2 == 0){
      if(status_update){
        snprintf(debug_buf, DEBUG_SIZE, "V:%6d; Vset:%6d; Target:%6d; Ph:%2d; I:%6d\r\n", vout_mv, vset_mv, target_vset_mv, n_phases, iout_ma);
        debug(debug_buf);
        // snprintf(debug_buf, DEBUG_SIZE, "Vset: %d;\t",vset_mv);
        // debug(debug_buf);
        // snprintf(debug_buf, DEBUG_SIZE, "Target: %d;\t", target_vset_mv);
        // debug(debug_buf);
        // snprintf(debug_buf, DEBUG_SIZE, "Ph: %d\r\n", n_phases);
        // debug(debug_buf);
        status_update = false;
      }
    }else{
      status_update = true;
    }

    // HEARTBEAT
    if(HAL_GetTick()/1000 % 2 == 0){
      LED_on(2);
    }else{
      LED_off(2);
    }

    HAL_Delay(10);
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

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim,
                                              uint32_t TimerIdx)
{
  LED_toggle(1);
  if(TimerIdx == HRTIM_TIMERINDEX_MASTER){

    // only run feedback loop if required
    if(state != OUTPUT_OFF && state != FAULT && state != BURST){
      disable_adc_interrupts();
      refactor_duty_cycle();
      enable_adc_interrupts();
    }

    uint32_t duty = 0; // duty cycle per phase
    // if(n_phases > 0){
    //   duty = duty_cycle / n_phases;
    // }else{
    //   duty = duty_cycle;
    // }
    // clamp per-phase duty cycle
    duty = CLAMP(duty_cycle, MIN_DUTY, MAX_DUTY);

    // BEGIN STATE MACHINE
    switch (state)
    {
    case ONE_PHASE_INIT:{
      n_phases = 1;
      kp = KP_1;
      memset(iphase_accs, 0, sizeof(uint32_t) * n_phases); // phase current accumulator reset
      iphase_sample_count = 0;
      HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty;
      hhrtim1.Instance->sMasterRegs.MCMP4R = duty / 2; // ADC interrupt timing, halfway through first phase
      HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1);
      state = ONE_PHASE; // goto 1 phase state
      break;
    
    }case TWO_PHASE_INIT:{
      n_phases = 2;
      kp = KP_2;
      memset(iphase_accs, 0, sizeof(uint32_t) * n_phases); // phase current accumulator reset
      iphase_sample_count = 0;
      HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);
      hhrtim1.Instance->sMasterRegs.MCMP1R = HRTIM_PERIOD / 2; // interrupt to set second phase

      hhrtim1.Instance->sMasterRegs.MCMP4R = duty / 2; // ADC interrupt timing, halfway through first phase
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = duty;
      HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1);
      state = TWO_PHASE; // goto 2 phase state
      break;
    
    }case THREE_PHASE_INIT:{
      n_phases = 3;
      kp = KP_3;
      memset(iphase_accs, 0, sizeof(uint32_t) * n_phases); // phase current accumulator reset
      iphase_sample_count = 0;
      HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1);
      hhrtim1.Instance->sMasterRegs.MCMP1R = HRTIM_PERIOD / 3; // interrupt to set second phase
      hhrtim1.Instance->sMasterRegs.MCMP2R = 2 * (HRTIM_PERIOD / 3); // interrupt to set third phase
      hhrtim1.Instance->sMasterRegs.MCMP4R = duty / 2; // ADC interrupt timing, halfway through first phase

      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = duty;
      HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 |  HRTIM_OUTPUT_TC1);

      state = THREE_PHASE; // goto 3 phase state
      break;
    
    }case FOUR_PHASE_INIT:{
      n_phases = 4;
      kp = KP_4;
      memset(iphase_accs, 0, sizeof(uint32_t) * n_phases); // phase current accumulator reset
      iphase_sample_count = 0;
      hhrtim1.Instance->sMasterRegs.MCMP1R = HRTIM_PERIOD / 4; // interrupt to set second phase
      hhrtim1.Instance->sMasterRegs.MCMP2R = 2 * (HRTIM_PERIOD / 4); // interrupt to set third phase
      hhrtim1.Instance->sMasterRegs.MCMP2R = 3 * (HRTIM_PERIOD / 4); // interrupt to set fourth phase
      hhrtim1.Instance->sMasterRegs.MCMP4R = duty / 2; // ADC interrupt timing, halfway through first phase
      // hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_MASTER].CMP4xR = duty / 2; // ADC interrupt timing, halfway through first phase
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = duty;
      HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 |  HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);
      
      state = FOUR_PHASE;
      break;
    
    }case ONE_PHASE:{
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty;
      
      hhrtim1.Instance->sMasterRegs.MCMP4R = duty / 2; // ADC interrupt timing, halfway through first phase
      // hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_MASTER].CMP4xR = duty_cycle / 2; // ADC interrupt timing, halfway through first phase
      
      if(iout_ma >= ADD_PHASE_CURR_THRESH){
        state = TWO_PHASE_INIT; // shift to two phases
      }
      // else if(iout_ma < 80){
      //   kp = KP_LIGHT_LOAD;
      //   ki = KI_LIGHT_LOAD;
      //   // state = BURST; // very light load, shift to burst mode
      // }else if(iout_ma > 100){
      //   kp = KP_1;
      //   ki = KI_1;
      // }

      break;
    
    }case TWO_PHASE:{
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = duty;

      hhrtim1.Instance->sMasterRegs.MCMP4R = duty / 2; // ADC interrupt timing, halfway through first phase
      // hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_MASTER].CMP4xR = duty / 2; // ADC interrupt timing, halfway through first phase

      if(iout_ma >= 2*ADD_PHASE_CURR_THRESH){
        state = THREE_PHASE_INIT;
      }else if(iout_ma < 2*SHED_PHASE_CURR_THRESH){
        // shed phase
        state = ONE_PHASE_INIT;
      }
      break;
    
    }case THREE_PHASE:{
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = duty;

      hhrtim1.Instance->sMasterRegs.MCMP4R = duty / 2; // ADC interrupt timing, halfway through first phase
      // hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_MASTER].CMP4xR = duty / 2; // ADC interrupt timing, halfway through first phase
      
      if(iout_ma >= 3*ADD_PHASE_CURR_THRESH){
        state = FOUR_PHASE_INIT;
      }else if(iout_ma < 3*SHED_PHASE_CURR_THRESH){
        // shed phase
        state = TWO_PHASE_INIT;
      }
      break;
    
    }case FOUR_PHASE:{
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = duty;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = duty;

      hhrtim1.Instance->sMasterRegs.MCMP4R = duty / 2; // ADC interrupt timing, halfway through first phase
      // hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_MASTER].CMP4xR = duty / 2; // ADC interrupt timing, halfway through first phase

      if(iout_ma < 4*SHED_PHASE_CURR_THRESH){
        // shed phase
        state = THREE_PHASE_INIT;
      }
      break;
    
    }case BURST:{
      break;
      verr_mv = vset_mv - vout_mv;
      if(abs(verr_mv) > 2*ALLOWABLE_ERROR){
        // 50mV or more below setpoint, likely dropping out of regulation, so switch to single phase
        state = ONE_PHASE_INIT;
      }else if(verr_mv > ALLOWABLE_ERROR){
        // less than 50mV but more than 25mV below setpoint, so turn on phase
        hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = BURST_DUTY_CYCLE;
      }else if(verr_mv < -ALLOWABLE_ERROR){
        // 25mV or more above setpoint, turn off phase
        hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = MIN_DUTY;
      }
      // break;
    }case FAULT:
    case OUTPUT_OFF:{
      duty_cycle = MIN_DUTY;
      vset_mv = 0;
      n_phases = 0;
      hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_MASTER].CMP4xR = HRTIM_PERIOD / 2; // ADC interrupt timing, halfway through first phase
      HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);
      break;
    }
    default:{
      // this will catch the FAULT state too
      HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TD1);
      break;
    }

    } //  end switch
    // END STATE MACHINE
  }
}
/**
 * On conversion complete, add to vout_sample_accumulator
 * After VOUT_SAMPLE_DEPTH many conversions, refactor duty cycle and set compare registers
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  // is called by the ADC DMA ConvCplt callback

  if(hadc->Instance == ADC1){
    
    // pertains to the voltage or current measurements

    // add current measurment
    iout_sample_acc += adc1_converted_values[1];
    iout_sample_count++;
    if(iout_sample_count >= IOUT_SAMPLE_DEPTH){
      uint32_t iout_adc_value = iout_sample_acc / iout_sample_count;
      iout_ma = (1000 * iout_adc_value) / (i_gain);
      iout_ma -= iout_offset_ma;
      if(iout_adc_value >= 3890){
        // greater than 95% of full scale, shift to less sensitive gain setting if possible
        if(i_gain_setting < 3){
          i_gain_setting += 1;
          set_i_gain(i_gain_setting);
        }
      }else if(iout_adc_value < 205){
        // less than 5% of full scale, shift to more sensetive gain setting if possible
        if(i_gain_setting > 0){
          i_gain_setting -= 1;
          set_i_gain(i_gain_setting);
        }
      }
      iout_sample_count = 0;
      iout_sample_acc = 0;
    }

    // add voltage measurment
    // add_sample(adc1_converted_values[0], vout_sample_buf, &vout_sample_ind, VOUT_SAMPLE_DEPTH);
    vout_sample_acc += adc1_converted_values[0];
    vout_sample_ind++;
    if(vout_sample_ind >= VOUT_SAMPLE_DEPTH){
      // buffer filled with new data, time to refactor duty cycle
      vout_sample_ind = 0;

      disable_hrtim_interrupts();
      // calculate Vout in mv
      vout_mv = vout_sample_acc * VREF_MV * VOUT_DIV;
      vout_mv = vout_mv / VOUT_SAMPLE_DEPTH;
      vout_mv = vout_mv >> 12; // divide by 4096, as our ADC is 12 bit
      vout_mv -= vout_offset_mv;
      enable_hrtim_interrupts();
      vout_sample_acc = 0;

    }

  }else if(hadc->Instance == ADC2){
    // pertains to all the other stuff

    // indices 0 thru 3 are phase current 1 thru 4
    if(iphase_sample_count < IPHASE_SAMPLE_DEPTH){
      for(int i = 0; i < n_phases; i++){
        iphase_accs[i] += adc2_converted_values[i];
      }

      iphase_sample_count++;
    }
    
    /*
    target_vset_acc += adc2_converted_values[4];
    target_vset_count++;

    if(target_vset_count >= 8){
      target_vset_mv = ((target_vset_acc * VREF_MV)/target_vset_count) >> 12;
      target_vset_mv = map(target_vset_mv, 0, VREF_MV, 0, VOUT_MAX_MV);
      target_vset_acc = 0;
      target_vset_count = 0;
    }
    */

    // calculate the target setpoint
    
    
  }
  // LED_off(1);
}


void set_i_gain(uint8_t setting){

  switch(setting){
    case 0:
      i_gain = IGAIN0;
      break;
    case 1:
      i_gain = IGAIN1;
      break;
    case 2:
      i_gain = IGAIN2;
      break;
    case 3:
      i_gain = IGAIN3;
      break;
    default:
      setting = 3;
      i_gain = IGAIN3;
      break;
  };

  HAL_GPIO_WritePin(IMUX_0_GPIO_Port, IMUX_0_Pin, setting & 1);
  HAL_GPIO_WritePin(IMUX_1_GPIO_Port, IMUX_1_Pin, (setting >> 1) & 1);
  HAL_GPIO_WritePin(IMUX_2_GPIO_Port, IMUX_2_Pin, (setting >> 2) & 1);

}

void refactor_duty_cycle(){
  // PI-esque feedback loop
  verr_mv = (int)vset_mv - (int)vout_mv;
  // verr_history[verr_rm_ind++] = verr_mv;
  // if(verr_rm_ind >= VERR_RUNNING_MEAN_SIZE){
  //   vdelta = 0;
  //   int verr_mean = verr_acc / verr_rm_ind;
  //   for(int i = 0; i < verr_rm_ind - 1; i++){
  //     vdelta += verr_history[i + 1] - verr_history[i];
  //   }
  //   verr_rm_ind = 0;
  // }
  if(abs(verr_mv) < ALLOWABLE_ERROR){
    return;
  }
  vinteg += verr_mv;
  vinteg = CLAMP(vinteg, -MAX_DUTY*ki, MAX_DUTY*ki);
  duty_cycle = (verr_mv * kp) + vinteg / ki;// + vdelta / kd;
  duty_cycle = CLAMP(duty_cycle, MIN_DUTY*n_phases, MAX_DUTY*n_phases);
}

void add_sample(uint32_t sample, uint32_t* buf, int* sample_ind, int depth){
  *sample_ind++;
  if(*sample_ind >= depth){
    *sample_ind = 0;
  }
  buf[*sample_ind] = sample;
}

void disable_adc_interrupts(){
  HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
}

void enable_adc_interrupts(){
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

void disable_hrtim_interrupts(){
  HAL_NVIC_DisableIRQ(HRTIM1_Master_IRQn);
}

void enable_hrtim_interrupts(){
  HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
}

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void LED_on(int led){
  switch (led)
  {
  case 0:
    // HAL_GPIO_WritePin(STAT_0_GPIO_Port, STAT_0_Pin, GPIO_PIN_SET);
    STAT_0_GPIO_Port->ODR |= STAT_0_Pin;
    break;
  case 1:
    // HAL_GPIO_WritePin(STAT_1_GPIO_Port, STAT_1_Pin, GPIO_PIN_SET);
    STAT_1_GPIO_Port->ODR |= STAT_1_Pin;
    break;
  case 2:
    // HAL_GPIO_WritePin(STAT_2_GPIO_Port, STAT_2_Pin, GPIO_PIN_SET);
    STAT_2_GPIO_Port->ODR |= STAT_2_Pin;
    break;
  default:
    break;
  }
}
void LED_off(int led){
switch (led)
  {
  case 0:
    // HAL_GPIO_WritePin(STAT_0_GPIO_Port, STAT_0_Pin, GPIO_PIN_RESET);
    STAT_0_GPIO_Port->ODR &= ~STAT_0_Pin;
    break;
  case 1:
    // HAL_GPIO_WritePin(STAT_1_GPIO_Port, STAT_1_Pin, GPIO_PIN_RESET);
    STAT_1_GPIO_Port->ODR &= ~STAT_1_Pin;
    break;
  case 2:
    // HAL_GPIO_WritePin(STAT_2_GPIO_Port, STAT_2_Pin, GPIO_PIN_RESET);
    STAT_2_GPIO_Port->ODR &= ~STAT_2_Pin;
    break;
  default:
    break;
  }
}
void LED_toggle(int led){
switch (led)
  {
  case 0:
    // HAL_GPIO_TogglePin(STAT_0_GPIO_Port, STAT_0_Pin);
    STAT_0_GPIO_Port->ODR ^= STAT_0_Pin;
    break;
  case 1:
    // HAL_GPIO_TogglePin(STAT_1_GPIO_Port, STAT_1_Pin);
    STAT_1_GPIO_Port->ODR ^= STAT_1_Pin;
    break;
  case 2:
    // HAL_GPIO_TogglePin(STAT_2_GPIO_Port, STAT_2_Pin);
    STAT_2_GPIO_Port->ODR ^= STAT_2_Pin;
    break;
  default:
    break;
  }
}


void debug(char* str){
  int len = strlen(str);

  if(debug_buf_to_use == 0){
    if(debug_a_size + len > DEBUG_SIZE){
      len = DEBUG_SIZE - debug_a_size; // max allowable length to append
    }
    memcpy(debug_buf_a + debug_a_size, str, len); // store new data
    debug_a_size += len;

    if(huart2.gState == HAL_UART_STATE_READY){
      debug_buf_to_use = 1;
      HAL_UART_Transmit_IT(&huart2, debug_buf_a, debug_a_size);
      debug_a_size = 0;
    }


  }else{
    if(debug_b_size + len > DEBUG_SIZE){
      len = DEBUG_SIZE - debug_b_size; // max allowable length to append
    }
    memcpy(debug_buf_b + debug_b_size, str, len); // store new data
    debug_b_size += len;

    if(huart2.gState == HAL_UART_STATE_READY){
      debug_buf_to_use = 0;
      HAL_UART_Transmit_IT(&huart2, debug_buf_b, debug_b_size);
      debug_b_size = 0;
    }
  }
  
  
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(debug_buf_to_use == 0 && debug_a_size > 0){
    debug_buf_to_use = 1; // indicate that buffer B should be filled while we send out buffer A
    HAL_UART_Transmit_IT(&huart2, debug_buf_a, debug_a_size); // transmit buffer a
    debug_a_size = 0;
  }else if(debug_buf_to_use == 1 && debug_b_size > 0){
    debug_buf_to_use = 0; // indicate that buffer A should be filled while we send out buffer B
    HAL_UART_Transmit_IT(&huart2, debug_buf_b, debug_b_size); // transmit buffer b
    debug_b_size = 0;
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
  // do I2C data reception things
  debug("I2C\r\n");
  switch(i2c_data[0]){
    case I2C_CMD_SET_VOUT:{
      if(!i2c_cmd_rcvd){
        // we received the command byte just now
        i2c_cmd_rcvd = true;
        // next we expect to receive the data
        HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_data + 1, sizeof(int));
      }else{
        // we just received the data
        int new_vout = 0;
        new_vout |= i2c_data[1] << 24;
        new_vout |= i2c_data[2] << 16;
        new_vout |= i2c_data[3] << 8;
        new_vout |= i2c_data[4];
        target_vset_mv = CLAMP(new_vout, 0, VOUT_MAX_MV);
        i2c_cmd_rcvd = false;
        HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_data, 1); // wait for a command byte
      }
      break;
    }case I2C_CMD_ENABLE_OUTPUT:{
      i2c_enabled_status = true;
      HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_data, 1); // wait for a command byte
      break;
    }case I2C_CMD_DISABLE_OUTPUT:{
      i2c_enabled_status = false;
      HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_data, 1); // wait for a command byte
      break;
    }default:{
      HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_data, 1); // wait for a command byte
      break;
    }
  };
  
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
