/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <math.h>
#define DEF_PAD_DATA "0a255255255255/0b0000000000000000000/a255255255255/0b000000000000000000/\r"
#define UART_TIMEOUT 300
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

/* USER CODE BEGIN PV */


PID Wheel_1;
PID Wheel_2;
PID Wheel_3;
PID Wheel_4;

float constatnts_test[1][3] = {
	{0.1,0.1,0.1}

};
float robot_matrix[3][4] = {
	{1.0,1.0,1.0,1.0},
	{-1.0,1.0,-1.0,1.0},
	{0.32,-0.32,-0.32,0.32}
};
float axes_robot_matrix_inverse[4][3] = {
	{0.25,-0.25,0.78125},
	{0.25,0.25,-0.78125},
	{0.25,-0.25,-0.78125},
	{0.25,0.25,0.78125}
};
float target_speed[3] = {
		0.0,0.0,0.0
};
float speed_wheels[4] = {
		0.0,0.0,0.0,0.0,
};
float axes_robot_matrix_transpone[3][3];
float mov_axes[3];
float now_speead [3];
float read_speed [4];
float robot_speed [3];
float delta_fi;
float speed_U;
float speed_V;
float speed_W;
float quest_xy[1][3];
float quest_UV[1][3];
float convert_xy_UV[3][3];
float inverse_converte_xy_UV[3][3];
float gipotinus;
float V = 0.05;
float W = 0.1;
float quest_FI;
float mot_grab = 0.5;
float distante;
uint8_t znamya_position;
uint8_t position = 0;
uint32_t posik = 90;
uint32_t posik_small = 55;
uint32_t posik1 = 90;
uint32_t posik_2 = 10;


uint32_t autonom_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void omron(uint8_t robot_position){  /*robot_position = 0 left , robot_position = 1 right*/
	if (robot_position == 0){ /*left robot position on place*/
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET){ //left_omron
			znamya_position = 1;
		}
		else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET){ //right_omron
			znamya_position = 2;
		}
		else znamya_position =3;

	}
	else{ /*right robot position on place*/
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_SET){ //left_omron
			znamya_position = 2;
		}

		else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET){ //right_omron
			znamya_position = 3;
		}
		else znamya_position = 1;
	}

}
typedef struct switches {
	bool mid_switch;
	bool down_switch;
	bool up_switch;
} switches;
switches switch_c;
void set_voltage_chain(float duty) {
 if(duty > 1.0) duty = 1.0;
 if(duty < -1.0) duty = -1.0;

	 if(duty >= 0.0)  {
		 TIM12->CCR1 = ((int32_t)(TIM12->ARR * duty));   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
  }
	 else {   TIM12->CCR1 = ((int32_t)(TIM12->ARR + (TIM12->ARR * duty)));
	 	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);  }
}
void check_switches(){

	switch_c.mid_switch = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET) ? 1 : 0;
	switch_c.down_switch = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_SET) ? 1 : 0;
	switch_c.up_switch = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;
}
void chain_control(int pos){

	static int flag = -1;

	if (pos == flag){
		set_voltage_chain(0);
		return;
	}

	switch (pos){//0-down, 1-mid, 2-up
	case(0):
		if (!switch_c.down_switch){
			set_voltage_chain(mot_grab);
		}
		else{
			flag = 0;
			set_voltage_chain(0);
		}
		break;
	case(1):
		if (!switch_c.mid_switch){
			if(flag == 0){
				set_voltage_chain(-mot_grab);
			}
			else{
				set_voltage_chain(mot_grab);
			}
			if (switch_c.up_switch && flag == 0){
				flag = 2;
				set_voltage_chain(mot_grab);
			}
			if (switch_c.down_switch && flag == 2){
				flag = 0;
				set_voltage_chain(-mot_grab);
			}
		}
		else{
			flag = 1;
			set_voltage_chain(0);
		}
		break;
	case(2):
		if (!switch_c.up_switch){
			set_voltage_chain(-mot_grab);
		}
		else{
			flag = 2;
			set_voltage_chain(0);
		}

	}

//
//	if(switch_c.up_switch){
//		mot_1 =
//	}
//	if(switch_c.down_switch){
//		set_voltage_chain(mot_1);
//	}
//	set_voltage_chain(mot_1);

}
void servo_control(uint8_t servo ,uint8_t position ){ /*position = 0 open , position = 1 close*/
	 switch(servo){
	 case 0:
		 if(position == 0){
			 TIM9->CCR1 = 30 +0.5*90;
		 }
		 else{
			 TIM9->CCR1 = 30 +0.5*43;
		 }
		 break;
	 case 1:
		 if(position == 0){
			 TIM9->CCR2 = 30 +0.5*120;
		 }
		 else{
			 TIM9->CCR2 = 30 +0.5*30;
		 }
		 break;
	 }
}
void chain(float duty){
	 if(duty >= 0.0)  {
			 TIM12->CCR1 = ((int32_t)(TIM12->ARR * duty));   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

	  }
		 else {   TIM12->CCR1 = ((int32_t)(-(TIM12->ARR * duty)));
		 	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);

		 }

}
void switch_lim(uint8_t pos){
	switch(pos){
		case (1):
			while (switch_c.down_switch != 1){
				check_switches();
				position = 0;
				chain_control(0);
				Wheel_1.target = 0;
				Wheel_2.target = 0;
				Wheel_3.target = 0;
				Wheel_4.target = 0;
			}
		break;
		case (2):
			while (switch_c.mid_switch != 1){
				check_switches();
				position = 1;
				chain_control(1);
				Wheel_1.target = 0;
				Wheel_2.target = 0;
				Wheel_3.target = 0;
				Wheel_4.target = 0;

		}
		break;
		case (3):
				position = 2;
			while (switch_c.up_switch != 1){
				check_switches();
				pos = 2;
				chain_control(2);
				Wheel_1.target = 0;
				Wheel_2.target = 0;
				Wheel_3.target = 0;
				Wheel_4.target = 0;
		}
		break;


	}
}
void Move_robot_coordinates_X_Y_W(float speed_v,float speed_w,float x_target,float y_target,float fi_target){
	V = speed_v;
	W = speed_w;
	quest_xy[0][0] = x_target;
	quest_xy[0][1] = y_target;
	quest_FI = fi_target;
	convert_xy_UV[0][0]=cos(fi);
	convert_xy_UV[0][1]=sin(fi);
	convert_xy_UV[0][2]=0;
	convert_xy_UV[1][0]=-sin(fi);
	convert_xy_UV[1][1]=cos(fi);
	convert_xy_UV[1][2]=0;
	convert_xy_UV[2][0]=position_x;
	convert_xy_UV[2][1]=position_y;
	convert_xy_UV[2][2]=1;
	matrixInverse(&convert_xy_UV[0][0],3,&inverse_converte_xy_UV[0][0]);
	matrixMultiplyM2M(&quest_xy[0][0],1,3,&inverse_converte_xy_UV[0][0],3,3,&quest_UV[0][0]);
	gipotinus =  sqrtf((quest_UV[0][0]*quest_UV[0][0])+(quest_UV[0][1]*quest_UV[0][1]) );
	distante = gipotinus;
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
	while(((gipotinus>=0.005)||delta_fi>=0.02)){
	convert_xy_UV[0][0]=cos(fi);
    convert_xy_UV[0][1]=sin(fi);
	convert_xy_UV[0][2]=0;
	convert_xy_UV[1][0]=-sin(fi);
	convert_xy_UV[1][1]=cos(fi);
	convert_xy_UV[1][2]=0;
	convert_xy_UV[2][0]=position_x;
	convert_xy_UV[2][1]=position_y;
	convert_xy_UV[2][2]=1;
	matrixInverse(&convert_xy_UV[0][0],3,&inverse_converte_xy_UV[0][0]);
	matrixMultiplyM2M(&quest_xy[0][0],1,3,&inverse_converte_xy_UV[0][0],3,3,&quest_UV[0][0]);
	gipotinus =  sqrtf((quest_UV[0][0]*quest_UV[0][0])+(quest_UV[0][1]*quest_UV[0][1]) );
	if (gipotinus > 0.005){
		if (gipotinus < 0.15){
			target_speed[0] = quest_UV[0][0]/gipotinus*V*(0.4+(0.6*gipotinus)/0.25);
			target_speed[1] = quest_UV[0][1]/gipotinus*V*(0.4+(0.6*gipotinus)/0.25);
		}
		else if(distante-gipotinus<0.15){

			target_speed[0] = quest_UV[0][0]/gipotinus*V*(0.4+(0.6*(distante-gipotinus))/0.25);
			target_speed[1] = quest_UV[0][1]/gipotinus*V*(0.4+(0.6*(distante-gipotinus))/0.25);

		}
		else{
			target_speed[0] = quest_UV[0][0]/gipotinus*V;
			target_speed[1] = quest_UV[0][1]/gipotinus*V;
		}
	}
	else{
		target_speed[0] = 0.0;
		target_speed[1] = 0.0;
	}
	if(fi>=(2*pi))fi = fi-2*pi;

		  	  if(fi<0.0) fi = fi+2*pi;

		  	if (delta_fi>0.02){
		  		switch(flaging){
		  			case(1):target_speed[2] = W;
		  			case(2):target_speed[2] = -W;
		  			case(3):target_speed[2] = -W;
		  			case(4):target_speed[2] = W;
		  		}

		  		  }
		  		else target_speed[2] = 0.0;


		  	matrixMultiplyM2M(&target_speed[0],1,3,&robot_matrix[0][0],3,4,&speed_wheels[0]);
		  	now_speead[0] = result_speed_1;
		  	now_speead[1] = result_speed_2;
		  	now_speead[2] = result_speed_3;
		  	read_speed[0] = result_speed_0;
		  	read_speed[1] = result_speed_1;
		  	read_speed[2] = result_speed_2;
		  	read_speed[3] = result_speed_3;
		  	matrixMultiplyM2M(&read_speed[0],1,4,&axes_robot_matrix_inverse[0][0],4,3,&robot_speed[0]);

		  	Wheel_1.target = speed_wheels[0];
		  	Wheel_2.target = speed_wheels[1];
		  	Wheel_3.target = speed_wheels[2];
		  	Wheel_4.target = speed_wheels[3];
	  		  speed_U = robot_speed[0];
		  		  speed_V = robot_speed[1];
		  		  speed_W = robot_speed[2];

	}


	speed_U = 0;
	speed_V = 0;
	speed_W = 0;
  	Wheel_1.target = 0;
  	Wheel_2.target = 0;
  	Wheel_3.target = 0;
  	Wheel_4.target = 0;

}
typedef struct gamepad {
	int axes[4];
	bool buttons[20];
} gamepad;
gamepad pads[2];
uint8_t rx_data[1];
uint8_t temp_data[100];
char data[100];
uint8_t size_data = 0;
uint8_t ind_data = 0;
bool write_data = 0;
uint8_t check_sum;
uint32_t timer = 0;
float move_axes[4];
void convertData(char *data) {
	bool now; // 1 - axes, 0 - buttons
	uint8_t pad;
	if (!(data[1] == 'a' || data[1] == 'b' || data[size_data - 1] == '\r')) {
		return;
	}
	for (int i = 1; data[i] != '\r'; i++) {
		if (data[i] == '\r')
			break;
		if (data[i] == 'a' || data[i] == 'b') {
			now = data[i] == 'a' ? 1 : 0;
			pad = data[i - 1] - 48;
			continue;
		}
		for (int j = 0; data[i - 1] != '/';) {
			if (data[i] == '/' || data[i - 1] == '/')
				break;
			if (now) {
				pads[pad].axes[j] = (data[i] - 48) * 100 + (data[i + 1] - 48) * 10 + (data[i + 2] - 48);
				j++;
				i += 3;
			} else if (!now) {
				pads[pad].buttons[j] = data[i] - 48;
				j++;
				i++;
			}
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (rx_data[0] == '%') {
		check_sum = 0;
		write_data = 1;
		ind_data = 0;
		size_data = 0;
	}

	if (write_data) {


		temp_data[ind_data] = rx_data[0];

		if (ind_data == 3){
			size_data = (temp_data[1] - 48) * 100 + (temp_data[2] - 48) * 10 + (temp_data[3] - 48);
		}
		if (ind_data <= size_data - 4 || ind_data <= 3) {
			check_sum += rx_data[0];
		}


		ind_data++;
	}
	if (ind_data >= 99) {
		check_sum = 0;
		write_data = 0;
		ind_data = 0;
		size_data = 0;
	}
	if (ind_data == size_data) {
		HAL_GetTick();
	}
	if (rx_data[0] == '\r') {

		//size_data = (temp_data[1] - 48) * 100 + (temp_data[2] - 48) * 10 + (temp_data[3] - 48);
		uint8_t check_sum_data = (temp_data[size_data - 3] - 48) * 100 + (temp_data[size_data - 2] - 48) * 10 + (temp_data[size_data - 1] - 48);
		write_data = 0;
		if (check_sum == check_sum_data && ind_data == size_data + 1) {
			timer = HAL_GetTick();
			memcpy(data, &temp_data[5], size_data);
			data[size_data - 8] = '\r';
		}
	}
}
bool autonom_flag = false;
void convert_chushpan(void) {
	if (HAL_GetTick() - timer >= UART_TIMEOUT) {
		memcpy(data, DEF_PAD_DATA, 74);
	}

	HAL_UART_Receive_DMA(&huart1, rx_data, 1);

	convertData((char*) data);

	for (int i = 0; i < 4; i++) {
		move_axes[i] = i % 2 == 0 ? (double)((pads[0].axes[i]) - 256.0f) / 256.0f : (double)((pads[0].axes[i]) - 256.0f) / -256.0f;
		move_axes[i] /= pads[0].buttons[7] ? 2:1;
		if (move_axes[i] >= -0.2 && move_axes[i] <= 0.2) move_axes[i] = 0.0f;

	}
	move_axes[2] *= -1;

	if (pads[0].buttons[8] && pads[0].buttons[9] &&
		pads[1].buttons[8] && pads[1].buttons[9])
	{
		if(!autonom_flag){
			autonom_timer = HAL_GetTick();
		}

		autonom_flag = true;

	}
	else
	{
		autonom_flag = false;
	}
}
void convert_typedef(void){

	Wheel_1.motor = 0;
	Wheel_1.target = 0.0;
	Wheel_1.current = 0;
	Wheel_1.Kp = 14;
	Wheel_1.Ki = 1;
	Wheel_1.Kd = 0.1;
	Wheel_1.wh_L = 0.0;

	Wheel_2.motor = 1;
	Wheel_2.target = 0.0;
	Wheel_2.current = 0;
	Wheel_2.Kp = 14;
	Wheel_2.Ki = 1;
	Wheel_2.Kd = 0.1;
	Wheel_2.wh_L = 0.0;

	Wheel_3.motor = 2;
	Wheel_3.target = 0.0;
	Wheel_3.current = 0;
	Wheel_3.Kp = 14;
	Wheel_3.Ki = 1;
	Wheel_3.Kd = 0.1;
	Wheel_3.wh_L = 0.0;

	Wheel_4.motor = 3;
	Wheel_4.target = 0.0;
	Wheel_4.current = 0;
	Wheel_4.Kp = 14;
	Wheel_4.Ki = 1;
	Wheel_4.Kd = 0.1;
	Wheel_4.wh_L = 0.0;
}
void control_mod (void){
	if(!autonom_flag){
		check_switches();
	convert_chushpan();
	  enum {
		  A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, LB_BUTTON, RB_BUTTON,
		  LT_BUTTON, RT_BUTTON, BACK_BUTTON, START_BUTTON, L3_BUTTON,
		  R3_BUTTON, UP_BUTTON, DOWN_BUTTON, LEFT_BUTTON, RIGHT_BUTTON,
		  XBOX_BUTTON
	  };

	target_speed[0] = move_axes[0]/2;
	target_speed[1] = move_axes[1]/2;
	target_speed[2] = move_axes[2]*1.5;

	matrixMultiplyM2M(&target_speed[0],1,3,&robot_matrix[0][0],3,4,&speed_wheels[0]);

	Wheel_1.target = speed_wheels[0];
	Wheel_2.target = speed_wheels[1];
	Wheel_3.target = speed_wheels[2];
	Wheel_4.target = speed_wheels[3];

	now_speead[0] = result_speed_1;
	now_speead[1] = result_speed_2;
	now_speead[2] = result_speed_3;

	read_speed[0] = result_speed_0;
	read_speed[1] = result_speed_1;
	read_speed[2] = result_speed_2;
	read_speed[3] = result_speed_3;

	matrixMultiplyM2M(&read_speed[0],1,4,&axes_robot_matrix_inverse[0][0],4,3,&robot_speed[0]);

	  if(pads[0].buttons[A_BUTTON]){
		  servo_control(1,1);

	  }
	  if(pads[0].buttons[B_BUTTON]){
		  servo_control(1,0);

	  }
	  if(pads[0].buttons[X_BUTTON]){
		  servo_control(0,1);

	  }
	  if(pads[0].buttons[Y_BUTTON]){
		  servo_control(0,0);

	  }
	  if(pads[0].buttons[LB_BUTTON]){
		  position = 0;
		  chain_control(position);
	  }
	  if(pads[0].buttons[LT_BUTTON]){
		  position = 1;
		  chain_control(position);
	  }
	  if(pads[0].buttons[RB_BUTTON]){
		  position = 2;
		  chain_control(position);
	  }
	  chain_control(position);
	}
}
void set_voltage(uint8_t motor , float duty) {
 if(duty > 1.0) duty = 1.0;
 if(duty < -1.0) duty = -1.0;
 switch(motor) {
 case 0:
	 if(duty >= 0.0)  {
		 TIM4->CCR1 = ((int32_t)(TIM4->ARR * duty));   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

  }
	 else {   TIM4->CCR1 = ((int32_t)(-(TIM4->ARR * duty)));
	 	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	 	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
	 }
  break;
  case 1:
	  if(duty >= 0.0)  {
		  TIM4->CCR2 = ((int32_t)(TIM4->ARR * duty));   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  }
	  else {   TIM4->CCR2 = ((int32_t)(-(TIM4->ARR * duty)));
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	  }
  break;
  case 2:
	  if(duty >= 0.0)  {
		  TIM4->CCR3 = ((int32_t)(TIM4->ARR * duty));   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2, GPIO_PIN_SET);
  }
	  else {   TIM4->CCR3 = ((int32_t)(-(TIM4->ARR * duty)));
	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
	  }
  break;
  case 3:
	  if(duty >= 0.0)  {
		  TIM4->CCR4 = ((int32_t)(TIM4->ARR * duty));   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  }
	  else {   TIM4->CCR4 = ((int32_t)(-(TIM4->ARR * duty)));
	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	  }
  break;
 }
}
void PID_Controller(PID *reg){

	reg->error = reg->target - reg->current;

	reg->sum_error += reg->error;
    if(reg->sum_error > 1) reg->sum_error = 1;
    if(reg->sum_error < -1) reg->sum_error = -1;

    reg->P  = reg->Kp * reg->error;
    reg->I  = reg->Ki * reg->sum_error;
    reg->dif_error = reg->error - reg->old_error;
    reg->D  = reg->Kd * reg->dif_error;
    reg->output  = reg->P  + reg->I  + reg->D ;
    set_voltage(  reg->motor , reg->output );
    reg->old_error = reg->error;
 }
void matrixCopy(float *m, char rows, char columns, float *new_m)
{
  char i,j;

    for(i = 0; i < rows; i++)
        for(j = 0; j < columns; j++)
          *(new_m+columns*i+j) = *(m+columns*i+j);
}
void matrixSetCell(float *m, char rows, char columns, char row, char column, float val)
{
  *(m+columns*(row-1)+column-1) = val;
}
void matrixFill(float *m, char rows, char columns, float val)
{
char i,j;

for(i = 0; i < rows; i++)
   for(j = 0; j < columns; j++)
      *(m+columns*i+j) = val;
}
float matrixGetCell(float *m, char rows, char columns, char row, char column)
{
  return *(m+columns*(row-1)+column-1);
}
void matrixMultiplyM2M(float *m1, char rows1, char columns1, float *m2, char rows2, char columns2, float *new_m)
{
float Sum;
char i,j,k;

  if (columns1 != rows2)
      *new_m = 0;
  else
    {
      for(i = 0; i < rows1; i++)
        for (j = 0; j < columns2; j++)
        {
            Sum = 0;
            for(k = 0; k < columns1; k++)
                Sum+= (*(m1+columns1*i+k)) * (*(m2+columns2*k+j));
            *(new_m+columns2*i+j) = Sum;
        }
    }
}
void matrixMultiplyS2M(float *m, char rows, char columns, float s, float *new_m)
{
char i,j;

for(i = 0; i < rows; i++)
  for (j = 0; j < columns; j++)
    *(new_m+i*columns+j) = (*(m+i*columns+j))*s;
}
void matrixPlusMinus(float *m1, float *m2, char rows, char columns, signed char sign,float *new_m)
{
char i,j;

if (sign >= 0)
  {
    for(i = 0; i < rows; i++)
      for (j = 0; j < columns; j++)
        *(new_m+i*columns+j) = (*(m1+i*columns+j)) + (*(m2+i*columns+j));
  }
else
  {
    for(i = 0; i < rows; i++)
      for (j = 0; j < columns; j++)
        *(new_m+i*columns+j) = (*(m1+i*columns+j)) - (*(m2+i*columns+j));
  }
}
void matrixTranspose(float *m, char rows, char columns, float *new_m)
{
char i,j;
for(i = 0; i < rows; i++)
  for (j = 0; j < columns; j++)
    *(new_m+j*rows+i) = *(m+i*columns+j);
}
void matrixCofactor(float *m, char size, float *new_m)
{
//float *buf1 = malloc(sizeof(float) * (size-1) * (size-1));
  float bufxx[10][10];
  float *buf1=(float*)bufxx;
char i=0,j=0,k=0,l=0, c=0, d=0;
signed char sign;

while (i<size)
  {
    j = 0;
    while (j<size)
      {
        k = 0;
        c = 0;
        if (((i+j)%2) == 0)
          sign = 1;
        else
          sign = -1;


        while (k<(size-1))
          {
            if (c == i)
              c++;
            l = 0;
            d = 0;
            while(l<(size-1))
              {
                if (d == j)
                  d++;
                *(buf1+(size-1)*k+l) = *(m+size*c+d);
                l++;
                d++;
              }
            k++;
            c++;
          }
          matrixDet_LU_Transform(buf1, size - 1,(new_m+size*i+j));
        *(new_m+size*i+j)*=sign;
        j++;
      }
    i++;
  }
}
void matrixInverse(float *m, char size, float *new_m)
{
float buf1[4][4];// = malloc(sizeof(float) * size * size);
float buf2 [4][4];//= malloc(sizeof(float) * size * size);
float buf;
float buf3[3][3];
 float det ;
  matrixDet_LU_Transform(m, size,&det);
char i,j;

matrixCofactor(m, size, &buf1[0][0]);
matrixTranspose(&buf1[0][0], size, size, &buf2[0][0]);

for (i = 1; i <= size; i++)
  for (j = 1; j <= size; j++)
    {
      buf = matrixGetCell(&buf1[0][0], size, size, i, j)/det;
      matrixSetCell((float*)buf3, size, size, i, j, buf);
    }
matrixTranspose(&buf3[0][0], size, size, new_m);
}
void matrixDet_LU_Transform(float *A, char n,float *out) //необходимо задать и�?ходную матрицу и переменную дл�? LU-матрицы
{
float temp = 0;//, *LU = malloc(sizeof(float) * n * n);
float LU_ [3][3];
float * LU = (float*)LU_;
char i,j,k;
for (j = 0; j < n; j++)
  {
    *(LU+j) = *(A+j);
    if (j >= 1)
      *(LU+j*n) = *(A+j*n)/(*LU);
  }
for (i = 1; i < n; i++)
  {
    for (j = i; j < n; j++)
      {
        for (k = 0; k <= (i-1); k++)
          temp += (*(LU+n*i+k))*(*(LU+k*n+j));
        *(LU+i*n+j) = *(A+i*n+j) - temp;
        temp = 0;
      }
    for (j = i+1; j < n; j++)
      {
        for (k = 0; k <= (i-1); k++)
          temp += (*(LU+j*n+k))*(*(LU+k*n+i));
        *(LU+j*n+i) = ((*(A+j*n+i)) - temp)/(*(LU+i*n+i));
        temp = 0;
      }
  }
temp = 1;
for (i = 0; i < n; i++)
  temp *= *(LU+i*n+i);
*out=temp;
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
			convert_typedef();




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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim6);



  HAL_UART_Receive_DMA (&huart1, rx_data, 1);
  target_speed[0] = 0;
  target_speed[1] = 0;
  target_speed[2] = 0;
  quest_xy[0][0] = 0;
  quest_xy[0][1] = 0;
  quest_xy[0][2] = 1;



  ////////////////////


/////////////////


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(autonom_flag){
	  convert_chushpan();
	  omron(1);
	  	  position = 0;
	  	  //switch_lim(1);
	  	  servo_control(0,0);
	  	  servo_control(1,0);
	  	  HAL_Delay(500);
	  	  servo_control(0 , 1);
	  	  HAL_Delay(300);
	  	  switch_lim(2);
	  	  Move_robot_coordinates_X_Y_W(0.25,0.7,-0.06,0.9,0.0);
	  	  Move_robot_coordinates_X_Y_W(0.25,0.7,0.03,2.1,3.3);
	  	  HAL_Delay(300);
	  	  servo_control(1,1);
	  	  fi = 3.14;
	  	  Move_robot_coordinates_X_Y_W(0.25,0.7,-0.22,2.03,3.93);
	  	  Move_robot_coordinates_X_Y_W(0.25,0.7,-1.22,0.35,5.7);
	  	  servo_control(1,0);
	  	  switch_lim(3);
	  	  HAL_Delay(250);
	  	  servo_control(0,0);

	  	  switch(znamya_position){
	  	  case (1):
	  	   Move_robot_coordinates_X_Y_W(0.35,0.2,-1.4,1.66,6.28);

	  	        break;

	  	  case(2):
	  		  Move_robot_coordinates_X_Y_W(0.35,0.3,-0.22,1.13,5.7);


	  	  	  	break;
	  	  case(3):
	  	  	  	  break;
	  	  }

  	  }
  		  while(autonom_flag ){
  			  convert_chushpan();
  			  Wheel_1.target = 0;
  			  Wheel_2.target = 0;
  			  Wheel_3.target = 0;
  			  Wheel_4.target = 0;
  			  	  }

  		control_mod();
	  }












    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
