
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
//#define USE_OFFICIAL_API

#include "debug.h"
#include "state.h"
#include "uart.h"
#ifdef USE_OFFICIAL_API
#include "vl53l1_api.h"
#else
#include "VL53L1X.h"
#endif  /* USE_OFFICIAL_API */
#include "protocol.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#ifdef USE_OFFICIAL_API
#define SENSOR_START_IDX 0
#define SENSOR_NBR     4
#else
#define SENSOR_START_IDX 0
#define SENSOR_NBR     12
#endif
/* Private variables ---------------------------------------------------------*/

static void idle_callback(state_t *obj);
static void stop_callback(state_t *obj);
static void start_callback(state_t *obj);
static void measuring_callback(state_t *obj);
static void config_callback(state_t *obj);


#ifdef USE_OFFICIAL_API
VL53L1_Dev_t  vl53l1_dev[SENSOR_NBR];
VL53L1_DEV    dev;
#else
VL53L1X sensor[SENSOR_NBR];
#endif /* USE_OFFICIAL_API */

uint16_t XSHUTx[12] = { XSHUT1_Pin, XSHUT2_Pin, XSHUT3_Pin, XSHUT4_Pin,
    XSHUT5_Pin, XSHUT6_Pin, XSHUT7_Pin, XSHUT8_Pin,
    XSHUT9_Pin, XSHUT10_Pin, XSHUT11_Pin, XSHUT12_Pin, };


static state_t state_obj = {
    .curr_state = STATE_NONE,
};

uart_t uart_obj;
CBUFFER_DEF_STATIC(uart_rxbuf, 80);

protocol_frame_t tx_frame;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef USE_OFFICIAL_API
void AutonomousLowPowerRangingTest(VL53L1_DEV);
#endif
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void idle_callback(state_t *obj)
{
    char cmd[8] = { 0 };
    DBG_LOG("read cmd from UART\r\n");
    
    if (uart_receive(&uart_obj, (uint8_t *) cmd, 1, HAL_MAX_DELAY) == -1) {
        DBG_LOG("%s error\r\n", __func__);
        return;
    }

    if (cmd[0] == 's')
        state_transit(obj, EVENT_START);
}

static void stop_callback(state_t *obj)
{
#ifndef USE_OFFICIAL_API
    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {
        sensor[i].stopContinuous();
    }

    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {
        GPIO_TypeDef* gpio = (i >= 0 && i <= 7) ? GPIOA : GPIOB;
        HAL_GPIO_WritePin(gpio, XSHUTx[i], GPIO_PIN_RESET);

        sensor[i].resetKlass();
    }
#else
    VL53L1_Error status;

    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {
        dev = &vl53l1_dev[i - SENSOR_START_IDX];
        status = VL53L1_StopMeasurement(dev);
        if (status) {
            DBG_LOG("VL53L1_StopMeasurement failed (%d)\n", status);
            while(1);
        }
    }

    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {
        GPIO_TypeDef* gpio = (i >= 0 && i <= 7) ? GPIOA : GPIOB;
        HAL_GPIO_WritePin(gpio, XSHUTx[i], GPIO_PIN_RESET);
    }
#endif
}

static void start_callback(state_t *obj)
{
#ifndef USE_OFFICIAL_API
    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {
        GPIO_TypeDef* gpio = (i >= 0 && i <= 7) ? GPIOA : GPIOB;
        HAL_GPIO_WritePin(gpio, XSHUTx[i], GPIO_PIN_SET);
        HAL_Delay(2);

        // uint16_t model_id = sensor[i].readReg16Bit(0x010F);
        // DBG_LOG("[%d] VL53L1X Model_ID: %02X\n", i, model_id);
        // sensor[i].setAddress(sensor[i].getAddress() + i + 1);
        // HAL_Delay(2);

        if (!sensor[i].init()) {
            DBG_LOG("Failed to detect and initialize sensor!\r\n");
        }

        sensor[i].setAddress(sensor[i].getAddress() + i + 1);

        sensor[i].setTimeout(1200);

        sensor[i].setDistanceMode(VL53L1X::Long);
        sensor[i].setMeasurementTimingBudget(50000);
    }

    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {
        sensor[i].startContinuous(50);
    }
#else
    VL53L1_Error status;
    uint8_t newI2C = 0x52;

    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {

        GPIO_TypeDef* gpio = (i >= 0 && i <= 7) ? GPIOA : GPIOB;
      
        HAL_GPIO_WritePin(gpio, XSHUTx[i], GPIO_PIN_SET);
        HAL_Delay(2);

        dev = &vl53l1_dev[i - SENSOR_START_IDX];
        dev->I2cHandle = &hi2c1;
        dev->I2cDevAddr = 0x52;/* sensor_addrs[i]; */
        /* VL53L1_RdByte(dev, 0x010F, &byteData); */
        /* DBG_LOG("VL53L1X Model_ID: %02X\n", byteData); */
        newI2C = dev->I2cDevAddr + (i + 1) * 2;
        status = VL53L1_SetDeviceAddress(dev, newI2C);
        dev->I2cDevAddr = newI2C;
        /* DBG_LOG("VL53L1X Model_ID: %02X\n", byteData); */

        status = VL53L1_WaitDeviceBooted(dev);	
        if (status){
            DBG_LOG("VL53L1_WaitDeviceBooted failed (%d)\n", status);
            while(1);
        }
        status = VL53L1_DataInit(dev);
        if (status){
            DBG_LOG("VL53L1_DataInit failed (%d)\n", status);
            while(1);
        }
        status = VL53L1_StaticInit(dev);
        if (status){
            DBG_LOG("VL53L1_StaticInit failed (%d)\n", status);
            while(1);
        }
        status = VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_LONG);
        if (status){
            DBG_LOG("VL53L1_SetDistanceMode failed (%d)\n", status);
            while(1);
        }
        status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 50000);
        if (status){
            DBG_LOG("VL53L1_SetMeasurementTimingBudgetMicroSeconds failed (%d)\n", status);
            while(1);
        }
        status = VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 100);
        if (status){
            DBG_LOG("VL53L1_SetInterMeasurementPeriodMilliSeconds failed (%d)\n", status);
            while(1);
        }
    }  

    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {
        dev = &vl53l1_dev[i - SENSOR_START_IDX];
        status = VL53L1_StartMeasurement(dev);
        if (status){
            DBG_LOG("VL53L1_StartMeasurement failed (%d)\n", status);
            while(1);
        }
    }
#endif
}

static void measuring_callback(state_t *obj)
{
    char cmd[8] = { 0 };

    if (uart_receive(&uart_obj, (uint8_t *) cmd, 1, 10) == -1) {
        goto start_measuring;
    }
    else {
        if (cmd[0] == 's') {
            state_transit(obj, EVENT_STOP);
        }
        return;
    }

#ifndef USE_OFFICIAL_API
  start_measuring:
    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) {
        sensor[i].read();
        tof_result_t *result = &tx_frame.payload.tof_result_payload[i];
        result->id = i;
        result->status = sensor[i].ranging_data.range_status;
        result->range_mm = sensor[i].ranging_data.range_mm;

        DBG_LOG("[%02d] range: %d\n", i, sensor[i].ranging_data.range_mm);
        DBG_LOG("\tstatus: %s\n", VL53L1X::rangeStatusToString(sensor[i].ranging_data.range_status));
        DBG_LOG("\tpeak signal: %d\n", sensor[i].ranging_data.peak_signal_count_rate_MCPS);
        DBG_LOG("\tambient: %d\n", sensor[i].ranging_data.ambient_count_rate_MCPS);
    }
    tx_frame.type = 3;
    tx_frame.length = sizeof(tx_frame.payload.tof_result_payload);
    uart_send(&uart_obj, &tx_frame, FRAME_SIZE(&tx_frame));
#else
  start_measuring:
    VL53L1_Error status;
    VL53L1_RangingMeasurementData_t RangingData;
    char measure_str[32] = { 0 };
  
    for (int i = SENSOR_START_IDX; i < SENSOR_START_IDX + SENSOR_NBR; i++) { // polling mode
        dev = &vl53l1_dev[i - SENSOR_START_IDX];

        status = VL53L1_WaitMeasurementDataReady(dev);
        if (!status) {
            status = VL53L1_GetRangingMeasurementData(dev, &RangingData);
            if (status == VL53L1_ERROR_NONE) {
                sprintf(measure_str, "%d: %d,%d,%.2f,%.2f\r\n", i, RangingData.RangeStatus,RangingData.RangeMilliMeter,
                        (RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
                uart_send(&uart_obj, measure_str, strlen(measure_str) + 2);
                DBG_LOG("%d: %d,%d,%.2f,%.2f\n", i, RangingData.RangeStatus,RangingData.RangeMilliMeter,
                        (RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
            }
            status = VL53L1_ClearInterruptAndStartMeasurement(dev);
        }
    }
#endif
}

static void config_callback(state_t *obj)
{
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  uart_init(&uart_obj, &huart1, &uart_rxbuf);
  state_init(&state_obj, NULL);
  struct state_ops_s *op = &state_obj.ops;
  op->idle_func     = idle_callback;
  op->stop_func     = stop_callback;
  op->start_func    = start_callback;
  op->measure_func  = measuring_callback;
  op->config_func   = config_callback;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      state_loop(&state_obj);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, XSHUT1_Pin|XSHUT2_Pin|XSHUT3_Pin|XSHUT4_Pin 
                          |XSHUT5_Pin|XSHUT6_Pin|XSHUT7_Pin|XSHUT8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, XSHUT9_Pin|XSHUT10_Pin|XSHUT11_Pin|XSHUT12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : XSHUT1_Pin XSHUT2_Pin XSHUT3_Pin XSHUT4_Pin 
                           XSHUT5_Pin XSHUT6_Pin XSHUT7_Pin XSHUT8_Pin */
  GPIO_InitStruct.Pin = XSHUT1_Pin|XSHUT2_Pin|XSHUT3_Pin|XSHUT4_Pin 
                          |XSHUT5_Pin|XSHUT6_Pin|XSHUT7_Pin|XSHUT8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT9_Pin XSHUT10_Pin XSHUT11_Pin XSHUT12_Pin */
  GPIO_InitStruct.Pin = XSHUT9_Pin|XSHUT10_Pin|XSHUT11_Pin|XSHUT12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// for linking error
extern "C"{
  int _getpid(){ return -1;}
  int _kill(int pid, int sig){ return -1; }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char const *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
