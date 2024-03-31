/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern float speed_uv;
extern float speed_fi;
extern float coords_x;
extern float coords_y;
extern float coords_fi;
extern float fi;
extern float delta_fi;
extern float delta_x;
extern float delta_y;
extern float result_speed_0;
extern float result_speed_1;
extern float result_speed_2;
extern float result_speed_3;
extern float result_speed_grab;
extern float speed_U;
extern float speed_V;
extern float speed_W;
extern float position_x;
extern float position_y;
extern float quest_FI;
extern float metr_chain;
extern int16_t Enc_Counter_0;
extern int16_t Enc_Counter_1;
extern int16_t Enc_Counter_2;
extern int16_t Enc_Counter_3;
extern int16_t Enc_Counter_4;
extern uint8_t flaging;
extern int tim_1;
extern float UV_convert_xy[2][2];
extern float delta_UV[1][2];
extern float delta_XY[1][2];
extern float state_autonom;
extern float state_flag;
typedef struct {
float P;
float I;
float D;
float I_old;
float target;
float current;
float error;
float sum_error;
float dif_error;
float output;
float Kp;
float Ki;
float Kd;
float old_error;
float wh_L;
uint8_t motor;
} PID;

extern PID Wheel_1;
extern PID Wheel_2;
extern PID Wheel_3;
extern PID Wheel_4;
extern PID Chain_motor;
void delta_FI(uint8_t pol);
void matrixMultiplyM2M(float *m1, char rows1, char columns1, float *m2, char rows2, char columns2, float *new_m);
void Move_robot_coordinates_X_Y(float speed,float x_target,float y_target);
void Move_robot_coordinates_W(float fee , float speed);
void Move_robot(float VU,float VV,float VW);
void Movingfront_gripper(float);
void convertData(char *data) ;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void convert_chushpan(void);
void convert_typedef(void);
void set_voltage(uint8_t motor , float duty);
void PID_Controller(PID *reg);
void matrixCopy(float *m, char rows, char columns, float *new_m);
void matrixSetCell(float *m, char rows, char columns, char row, char column, float val);
void matrixFill(float *m, char rows, char columns, float val);
float matrixGetCell(float *m, char rows, char columns, char row, char column);
void matrixMultiplyS2M(float *m, char rows, char columns, float s, float *new_m);
void matrixPlusMinus(float *m1, float *m2, char rows, char columns, signed char sign,float *new_m);
 void matrixTranspose(float *m, char rows, char columns, float *new_m);
void matrixCofactor(float *m, char size, float *new_m);
void matrixInverse(float *m, char size, float *new_m);
void matrixDet_LU_Transform(float *A, char n,float *out);


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define radius 0.03 //wheel
#define pulse_enc 2236
#define pulse_enc_omron 1025
#define pi 3.1415926535897932
#define pi_Radius 0.188495559215387592 //wheel
#define pi_Radius_pulse_enc 0.00008430033954176547 //wheel
#define radius_chain 0.052 //chain
#define pi_Radius_chain 0.3267256359733384928 //chain
#define pi_Radius_pulse_enc_chain 0.0003187567180227692613 //chain
#define traget_break  quest_UV[0][1] / gipotinus * V * (0.4 + (0.6 * gipotinus) / 0.25)
#define target_acceleration  quest_UV[0][1] / gipotinus * V * (0.4 + (0.6 * (distante - gipotinus)) / 0.25)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define dig_pin3_inverse_Pin GPIO_PIN_2
#define dig_pin3_inverse_GPIO_Port GPIOE
#define servo_2_Pin GPIO_PIN_5
#define servo_2_GPIO_Port GPIOE
#define servo_1_Pin GPIO_PIN_6
#define servo_1_GPIO_Port GPIOE
#define dig_pin1_Pin GPIO_PIN_14
#define dig_pin1_GPIO_Port GPIOC
#define dig_pin2_Pin GPIO_PIN_15
#define dig_pin2_GPIO_Port GPIOC
#define ENC_5_A_Pin GPIO_PIN_0
#define ENC_5_A_GPIO_Port GPIOA
#define ENC_5_B_Pin GPIO_PIN_1
#define ENC_5_B_GPIO_Port GPIOA
#define ENC_3_A_Pin GPIO_PIN_6
#define ENC_3_A_GPIO_Port GPIOA
#define ENC_3_B_Pin GPIO_PIN_7
#define ENC_3_B_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define dig_pin1_inverse_Pin GPIO_PIN_7
#define dig_pin1_inverse_GPIO_Port GPIOE
#define ENC_4_A_Pin GPIO_PIN_9
#define ENC_4_A_GPIO_Port GPIOE
#define dig_pin3_Pin GPIO_PIN_10
#define dig_pin3_GPIO_Port GPIOE
#define ENC_4_B_Pin GPIO_PIN_11
#define ENC_4_B_GPIO_Port GPIOE
#define dig_pin5_Pin GPIO_PIN_12
#define dig_pin5_GPIO_Port GPIOE
#define dig_pin4_Pin GPIO_PIN_14
#define dig_pin4_GPIO_Port GPIOE
#define pwm5_MT_Pin GPIO_PIN_14
#define pwm5_MT_GPIO_Port GPIOB
#define omron_1_Pin GPIO_PIN_9
#define omron_1_GPIO_Port GPIOD
#define omron_2_Pin GPIO_PIN_10
#define omron_2_GPIO_Port GPIOD
#define pwm1_MT_Pin GPIO_PIN_12
#define pwm1_MT_GPIO_Port GPIOD
#define pwm2_MT_Pin GPIO_PIN_13
#define pwm2_MT_GPIO_Port GPIOD
#define pwm3_MT_Pin GPIO_PIN_14
#define pwm3_MT_GPIO_Port GPIOD
#define pwm4_MT_Pin GPIO_PIN_15
#define pwm4_MT_GPIO_Port GPIOD
#define ENC_1_A_Pin GPIO_PIN_6
#define ENC_1_A_GPIO_Port GPIOC
#define ENC_1_B_Pin GPIO_PIN_7
#define ENC_1_B_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ENC_2_A_Pin GPIO_PIN_15
#define ENC_2_A_GPIO_Port GPIOA
#define limit_switch_1_Pin GPIO_PIN_0
#define limit_switch_1_GPIO_Port GPIOD
#define mosfet_enable_Pin GPIO_PIN_1
#define mosfet_enable_GPIO_Port GPIOD
#define limit_switch_3_Pin GPIO_PIN_2
#define limit_switch_3_GPIO_Port GPIOD
#define ENC_2_B_Pin GPIO_PIN_3
#define ENC_2_B_GPIO_Port GPIOB
#define dig_pin4_inverse_Pin GPIO_PIN_8
#define dig_pin4_inverse_GPIO_Port GPIOB
#define dig_pin2_inverse_Pin GPIO_PIN_9
#define dig_pin2_inverse_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
