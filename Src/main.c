/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#ifdef __cplusplus
 }
#endif

/* USER CODE BEGIN Includes */
#include "ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"	
#include <nav_msgs/Odometry.h>
#include "time.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <stdio.h>

float steering_angle = 0;
extern float encoder_speed ; // r/s
float after_filter = 0;
extern	 uint16_t count ;
extern 	int sum;
 
const float L = 0.26; //m
 
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

 
 void messageCb( const std_msgs::Empty& toggle_msg){
  HAL_GPIO_TogglePin(LD6_GPIO_Port,LD6_Pin);   // blink the led
}
 void cmdCb( const ackermann_msgs::AckermannDriveStamped & cmd_msg){
   float k = 1.0 ;
	 float angle_k = 1; 
	 if(cmd_msg.drive.speed>=0)
		 k=0.05;
	 else
		 k=0.1;
	 uint32_t move_speed = (cmd_msg.drive.speed*k+1.6) * 84000 ;   //限速1.7
	 steering_angle = (cmd_msg.drive.steering_angle*angle_k+1.5) * 84000;
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,move_speed);  //PA1 move_speed:(0-168000)<->(0-2ms)
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,steering_angle);  //PA2 steering_angle:(0-168000) <-> (0-2ms)
  HAL_GPIO_TogglePin(LD6_GPIO_Port,LD6_Pin);   // blink the led
	 steering_angle = cmd_msg.drive.steering_angle; //unit: rad
}
 #ifdef __cplusplus
 }
#endif

/* USER CODE END 0 */

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
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim1);
	
	HAL_TIM_IC_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
//	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,84000);
//	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,168000);  //设置舵机转向和电机的转速
	
	
	
	ros::NodeHandle nh;
	std_msgs::String str_msg;
  ros::Publisher chatter("chatter", &str_msg);
	
  char hello[13] = "hello world!";
	
  tf::TransformBroadcaster odom_broadcaster;

	double x = 0.0;
	double y = 0.0;
	double theta = 0;
	ros::Time temp;
	float delta_T = 0;

	//char base_link[] = "/base_link";
//	char odom[] = "/odom";
	
	
  ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );
  ros::Subscriber<ackermann_msgs::AckermannDriveStamped> sub_cmd("/IRC/cmd_vel", cmdCb );
	nav_msgs::Odometry odom;
	ros::Publisher odom_pub("odom",&odom);
  nh.initNode();

  nh.advertise(odom_pub);
  nh.advertise(chatter);
  nh.subscribe(sub);
	nh.subscribe(sub_cmd);
	odom_broadcaster.init(nh);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {

//		int8_t temp[9] = {0};
//		int64_t num = 0;
//		num = MILLISEONS;
//    for(int i=0;i!=8;i++) 
//		 {
//			 temp[7-i] =num%10+'0';
//       num/=10;
//		 }
//		temp[8] = '\n';
//		CDC_Transmit_FS((uint8_t *)temp,sizeof(temp));
//    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
		
// 		Main_loop();
	
	str_msg.data = "hello";
  chatter.publish(&str_msg);
	  // drive in a circle
  double dx = encoder_speed;
		delta_T = nh.now().toSec() - temp.toSec();
		temp = nh.now();
 // double dtheta = steering_angle;
  float vx =cos(theta)*dx*delta_T;  //非全向移动平台
  float vy = sin(theta)*dx*delta_T;
  float vth = dx/L*tan(steering_angle)*0.1;  //
	x += vx;
	y += vy;
	theta += vth ;
    
		
	//since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
                                                
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = temp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
  //  nav_msgs::Odometry odom;
    odom.header.stamp = temp;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    //odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(&odom);


  encoder_speed = 0;
  nh.spinOnce();
	for(int64_t i = 720000;i!=0;i--);

  /* USER CODE END WHILE */

}
}
/* USER CODE BEGIN 3 */
#ifdef __cplusplus
  extern "C" {
#endif
/* USER CODE END 3 */

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 
#ifdef __cplusplus
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
