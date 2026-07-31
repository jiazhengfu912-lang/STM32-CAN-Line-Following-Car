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

#include "bsp_can.h"
#include "main.h"
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef	TxHeader;      //发送
CAN_RxHeaderTypeDef	RxHeader;      //接收
 
uint8_t	CanRxData[8];  //数据接收数组，can的数据帧只有8帧
extern  CAN_HandleTypeDef hcan;
extern  uint8_t CanRx_Flag;
extern  uint8_t CanTx_Flag;
extern  uint8_t CanRxBuf1[8];
extern  uint8_t CanRxBuf2[8];
extern  uint8_t CanRxBuf3[8]; //g
extern  uint8_t CanTxBuf[8];
volatile motor_feedback_t motor_feedback[2] = {0};
/* USER CODE END 0 */

/*CAN过滤器初始化*/
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;
	
  sFilterConfig.FilterBank = 0;                    /* 过滤器组0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;  /* 屏蔽位模式 */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; /* 16位。*/
  
  sFilterConfig.FilterIdHigh         = CAN_Rx_StdId1<<5;			/* 要过滤的ID高位 */
  sFilterConfig.FilterIdLow          = CAN_Rx_StdId2<<5;          /* 要过滤的ID低位 */
  sFilterConfig.FilterMaskIdHigh     = CAN_Rx_StdId3<<5;			/* 过滤器高16位每位必须匹配 */
  sFilterConfig.FilterMaskIdLow      = CAN_Rx_StdId4<<5;			/* 过滤器低16位每位必须匹配 */
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;           /* 过滤器被关联到FIFO 0 */
  sFilterConfig.FilterActivation = ENABLE;          /* 使能过滤器 */ 
  sFilterConfig.SlaveStartFilterBank = 14;
	
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
		/* Filter configuration Error */
    Error_Handler();
  }
	
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	TxHeader.StdId=CAN_Tx_StdId0;       //标准帧标识符(11位)
	TxHeader.IDE=0x0000;                //使用标准帧
	TxHeader.RTR=CAN_RTR_DATA;          //数据帧
	TxHeader.DLC=8; 
	TxHeader.TransmitGlobalTime = DISABLE;
}
/*CAN发送数据，入口参数为要发送的数组指针，数据长度，返回0代表发送数据无异常，返回1代表传输异常*/
uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len)
{	
    uint8_t i1=0;
	uint32_t TxMailbox;
	uint8_t message[8];

	if (len > sizeof(message))
	{
		return 1;
	}

	TxHeader.StdId=CAN_Tx_StdId0;        //扩展标识符(29位)
	TxHeader.IDE=0x0000;        //使用扩展帧
	TxHeader.RTR=CAN_RTR_DATA;  //数据帧
	TxHeader.DLC=len;    
	
  for(i1=0;i1<len;i1++)
  {
		message[i1]=msg[i1];
	}
	
  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, message, &TxMailbox) != HAL_OK)//发送
	{
		return 1;
	}
    return 0;
}
/*CAN接收中断函数*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanNum)
{
	uint32_t i2;
	uint32_t i4;	
	CanRx_Flag =1;  //接收标志位
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &RxHeader, CanRxData);
    switch(RxHeader.StdId)
    {
        case 0x00000201: 	for(i2=0;i2<RxHeader.DLC;i2++)  CanRxBuf1[i2]=CanRxData[i2];  //用RxBuf1转存201的数据
                            break;
        case 0x00000202: 	for(i4=0;i4<RxHeader.DLC;i4++)  CanRxBuf2[i4]=CanRxData[i4];  //用RxBuf2转存202的数据
                            break;
        case 0x00000203:    for(i2=0;i2<RxHeader.DLC;i2++)  CanRxBuf3[i2]=CanRxData[i2]; //g
                            break; //g
        default:            break;
    }

	if (RxHeader.DLC < 4U)
	{
		return;
	}

	if (RxHeader.StdId == CAN_Rx_StdId1)
	{
		motor_feedback[0].angle = ((uint16_t)CanRxData[0] << 8) | CanRxData[1];
		motor_feedback[0].speed_rpm = (int16_t)(((uint16_t)CanRxData[2] << 8) | CanRxData[3]);
		motor_feedback[0].last_update_ms = HAL_GetTick();
		motor_feedback[0].valid = 1U;
	}
	else if (RxHeader.StdId == CAN_Rx_StdId2)
	{
		motor_feedback[1].angle = ((uint16_t)CanRxData[0] << 8) | CanRxData[1];
		motor_feedback[1].speed_rpm = (int16_t)(((uint16_t)CanRxData[2] << 8) | CanRxData[3]);
		motor_feedback[1].last_update_ms = HAL_GetTick();
		motor_feedback[1].valid = 1U;
	}
}

uint8_t Set_moto_current(s16 iq1, s16 iq2, s16 iq3, s16 iq4)
 {
	CanTxBuf[0] = (iq1 >> 8);
	CanTxBuf[1] = iq1;
	CanTxBuf[2] = (iq2 >> 8);
	CanTxBuf[3] = iq2;
	CanTxBuf[4] = iq3 >> 8;
	CanTxBuf[5] = iq3;
	CanTxBuf[6] = iq4 >> 8;
	CanTxBuf[7] = iq4;

	return CAN_Send_Msg(CanTxBuf,8);
}	

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
