/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int16_t Enc_Counter_0 = 0;
int16_t Enc_Counter_1 = 0;
int16_t Enc_Counter_2 = 0;
int16_t Enc_Counter_3 = 0;
int16_t Enc_Counter_4 = 0;
uint8_t flaging = 0;
float metr_chain = 0.0;
float old_chain = 0.0;
float result_speed_0 = 0.0;
float result_speed_1 = 0.0;
float result_speed_2 = 0.0;
float result_speed_3 = 0.0;
float result_speed_grab = 0.0;
float fi = 0.000001;
float delta_x;
float delta_y;
float delta_U;
float delta_V;
float UV_convert_xy[2][2];
float delta_UV[1][2];
float delta_XY[1][2];
float position_x;
float position_y;
float speed_uv;
float speed_fi;
float coords_x;
float coords_y;
float coords_fi;
float state_autonom = 0;
float state_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	state_autonom = state_autonom + 0.01;
	state_flag = state_flag + 0.01;
	 Enc_Counter_0 = TIM8->CNT;
	 Enc_Counter_1 = TIM2->CNT;
	 Enc_Counter_2 = TIM3->CNT;
	 Enc_Counter_3 = TIM1->CNT;
	 Enc_Counter_4 = TIM5->CNT;

	 result_speed_0 = ((float)Enc_Counter_0) *  pi_Radius_pulse_enc * 100.0;
	 result_speed_1 = ((float)Enc_Counter_1) *  pi_Radius_pulse_enc * 100.0;
	 result_speed_2 = ((float)Enc_Counter_2) *  pi_Radius_pulse_enc * 100.0;
	 result_speed_3 = ((float)Enc_Counter_3) *  pi_Radius_pulse_enc * 100.0;
	 result_speed_grab = ((float)Enc_Counter_4) * pi_Radius_pulse_enc_chain  * 100.0;

	 TIM8->CNT = 0;
	 TIM2->CNT = 0;
	 TIM3->CNT = 0;
	 TIM1->CNT = 0;
	 TIM5->CNT = 0;

	 Wheel_1.current = result_speed_0;
	 Wheel_2.current = result_speed_1;
	 Wheel_3.current = result_speed_2;
	 Wheel_4.current = result_speed_3;
	 Chain_motor.current = result_speed_grab;

	 PID_Controller(&Wheel_1);
	 PID_Controller(&Wheel_2);
	 PID_Controller(&Wheel_3);
	 PID_Controller(&Wheel_4);
	 PID_Controller(&Chain_motor);

	 fi = fi + speed_W * 0.01;
	 delta_UV[0][0] = speed_U *0.01;
	 delta_UV[0][1] = speed_V *0.01;
	 if(fi>=(2*pi)) fi = fi-2*pi;
	 if(fi<0.0)	 fi = fi+2*pi;
	 if(quest_FI>fi){
		 if((quest_FI-fi)>pi){
			delta_fi = (2*pi-quest_FI+fi);
			flaging = 1;
		 }
		 else {

			 delta_fi = (quest_FI - fi);
			 flaging = 2;
		 }

	 }
	 else{
		 if((fi-quest_FI)>pi){
			 flaging = 3;
			delta_fi =  2*pi-fi+quest_FI;;

		 }
		 else {
			 flaging = 4;
			 delta_fi = fi-quest_FI;
		 }


	 }

	 UV_convert_xy[0][0] = cos(fi);
	 UV_convert_xy[0][1] = sin(fi);
	 UV_convert_xy[1][1] = cos(fi);
	 UV_convert_xy[1][0] = -sin(fi);


	 matrixMultiplyM2M(&delta_UV[0][0],1,2,&UV_convert_xy[0][0],2,2,&delta_XY[0][0]);
	 metr_chain = metr_chain + result_speed_grab*0.01 ;
	 position_x = position_x +  delta_XY[0][0];
	 position_y = position_y +  delta_XY[0][1];


  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
