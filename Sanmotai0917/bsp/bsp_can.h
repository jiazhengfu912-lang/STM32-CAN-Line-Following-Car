/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F103xB
#include "stm32f1xx_hal.h"
#endif
#include "mytype.h"

/*CAN发送或是接收的ID*/
/* USER CODE BEGIN Private defines */
#define CAN_Tx_StdId0 0x200
#define CAN_Tx_StdId1 0x1FF

#define CAN_Rx_StdId1 0x201
#define CAN_Rx_StdId2 0x202
#define CAN_Rx_StdId3 0x203
#define CAN_Rx_StdId4 0x204
#define CAN_Rx_StdId5 0x205
#define CAN_Rx_StdId6 0x206
#define CAN_Rx_StdId7 0x207
#define CAN_Rx_StdId8 0x208

/*接收到电机参数结构体*/
#define FILTER_BUF_LEN		5
typedef struct{
	int16_t	 	speed_rpm;
    float  	    real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	        //abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			buf_idx;
	u16			angle_buf[FILTER_BUF_LEN];
	u16			fited_angle;
	u32			msg_cnt;
}moto_measure_t;

typedef struct
{
	uint16_t angle;
	int16_t speed_rpm;
	uint32_t last_update_ms;
	uint8_t valid;
} motor_feedback_t;

extern volatile motor_feedback_t motor_feedback[2];

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void CAN_Filter_Init(void);   //过滤器配置函数
uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len);  //数据发送函数
 
extern CAN_TxHeaderTypeDef	TxHeader;      //发送
extern CAN_RxHeaderTypeDef	RxHeader;      //接收
extern uint8_t	CanRxData[8];   //数据接收数组
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
uint8_t Set_moto_current(s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/* USER CODE END Prototypes */

#endif
