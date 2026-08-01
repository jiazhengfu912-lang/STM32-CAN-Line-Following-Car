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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "bsp_can.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SpeedStep 50		//0915 20 -->>50
#define MOTOR3_GEAR_RATIO 36 //g
#define MOTOR3_MAX_OUTPUT_RPM 500 //g
#define Target_SpeedO 45
#define Target_SpeedC 120
#define Target_SpeedD 65
#define DECAY_STEP   70 // ÿ�μ��ٵ��ٶ�ֵ  //ԭΪ80 ����0730 HTG	//0915�޸�Ϊ70 
#define ACTUATOR_MIN    45     // ??CCR?(????,??????Target_SpeedO)
#define ACTUATOR_MAX    120    // ??CCR?(????,??????Target_SpeedC)
#define ACTUATOR_STEP   2      // ?10ms???,??????(??????)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


uint8_t Uart2RxByte;
volatile uint16_t PendingDuty = 0; //g
volatile uint8_t DutyUpdateFlag = 0;//
 uint16_t duty ; //g
		uint16_t pwmM1=0;
    uint8_t TxBuf[5]="str !";
    uint8_t RxBuf[5]="str !";
    uint8_t CanRxBuf1[8];
    uint8_t CanRxBuf2[8];
    uint8_t CanRxBuf3[8]; //g
    uint8_t CanTxBuf[8];
    uint8_t CanRx_Flag;
    uint8_t CanTx_Flag;
	
		volatile uint8_t decelerating = 0; // ��¼�Ƿ����ڼ���
		char CurrentStatus = 0x00;													//00_ֹͣ    01_ֱ��     02_��ת    03_��ת    04_����     05_����     06_���    07_���    08_����   
		char LastStatus = 0x00;													    //00_ֹͣ    01_ֱ��     02_��ת    03_��ת    04_����     05_����     06_���    07_���    08_����    
		char Pid_en = 0x00;                                                         //00_�ر�PID   01_����PID
    char RX_Status=0;
    char CANStatus;
    char ChangeMode=0x00;                                                       //00_��������    01_���     02_���     03_��������
    char RxFlag;
    char RxCnt;
  
	int16_t Target_Speed = 0x0000;                                              //0x0000-0xffff  ָ��Ŀ����ٶ�
	int16_t Runtime_Speed = 0x0000;                                             //0x0000-0xffff  ʵʱĿ����ٶ�
  int16_t Right_Speed = 0x0000;                                               //0x0000-0xffff  ʵʱ���ֽ��ٶ�
  int16_t Left_Speed = 0x0000;                                                //0x0000-0xffff  ʵʱ���ֽ��ٶ�
  int16_t SetSpeed = 0x0000;                                                  //0x0000-0xffff  ָ����ٶȣ����Դ�������
  int16_t Target_SpeedL = 0x0000;                                             //0x0000-0xffff  ����Ŀ����ٶȣ�������ת
  int16_t Target_SpeedR = 0x0000;                                              //0x0000-0xffff  ����Ŀ����ٶȣ�������ת
  volatile int16_t Target_Speed3 = 0x0000; //g
  volatile int16_t Motor3OutputRpm = 0x0000; //g
  uint8_t Motor3RxBuf[2] = {0}; //g
  volatile uint8_t Motor3RxPending = 0; //g
  int16_t Target_SpeedB = 0x0000;                                              //0x0000-0xffff  ����Ŀ����ٶȣ����ڵ���
  int16_t new_speedL = 0x0000;                                             //0x0000-0xffff  ����Ŀ����ٶȣ�������ת
  int16_t new_speedR = 0x0000;                                              //0x0000-0xffff  ����Ŀ����ٶȣ�������ת 
  
    
	int16_t Left_current = 0x0000;                                              //0x0000-0xffff  ����ʵʱ����
	int16_t Right_current = 0x0000;                                             //0x0000-0xffff  ����ʵʱ����   
  int16_t speed_rpm[3]; //g
  uint8_t  auto_actuator_state = 0; // 0=??? 1=???? 2=??? 3=????
  uint16_t auto_wait_cnt = 0;       // ???????
    PID_TypeDef Motor_pid[3]; //g
    moto_measure_t moto_chassis[3] = {0};//3 chassis moto //g
uint8_t  actuator_move_flag = 0; //0=stop,1=moving
uint16_t actuator_target_ccr = 0; //CCR
volatile uint8_t should_send_stop_cmd = 0; // 0: �����κ���, 1: ��ʼ��ʱ��׼������ֹͣ����
volatile uint32_t command_timer_start = 0;   // ��¼��ʼ��ʱ��ʱ���
// 0917����ȫ�ֱ��������ڴ洢���������ʵʱλ�� RHS
volatile int16_t motor1_current_position = 0;
volatile int16_t motor2_current_position = 0;
volatile uint8_t line_sensor_raw = 0;
uint8_t lora_rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
// 0917 �������ڴ���2���յĻ���������СΪ22�ֽ� RHS
#define UART2_RX_BUFFER_SIZE 22 
uint8_t huart2_rx_buffer[UART2_RX_BUFFER_SIZE];

/* USER CODE BEGIN PFP */
void rep(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 硬件标定参数：仅在更换电机或车轮后修改。 */
#define M2006_GEAR_RATIO                         36    /* M2006 电机减速比。 */
#define LINE_FOLLOW_ENCODER_COUNTS_PER_MOTOR_REV   8192U /* 电机轴每圈的反馈编码器计数。 */
#define LINE_FOLLOW_WHEEL_CIRCUMFERENCE_UM       204204U /* 直径 65 mm 车轮的周长，单位：微米。 */

/* LoRa 透明传输协议：帧格式为 AA 55 SRC CMD RID SUM。 */
#define LORA_FRAME_HEADER_0                      0xAAU /* 帧头第 1 字节。 */
#define LORA_FRAME_HEADER_1                      0x55U /* 帧头第 2 字节。 */
#define LORA_FRAME_SIZE                             6U /* 固定帧长，单位：字节。 */
#define LORA_SOURCE_CAR                          0x01U /* 小车发送方编号。 */
#define LORA_SOURCE_GROUND                       0x10U /* 地面站发送方编号。 */
#define LORA_COMMAND_READY                       0x01U /* 小车就绪并等待启动命令。 */
#define LORA_COMMAND_START                       0x02U /* 地面站允许小车启动。 */
#define LORA_COMMAND_STOPPED                     0x03U /* 小车已到终点并停稳。 */
#define LORA_READY_PERIOD_MS                      500U /* 未启动时 READY 的发送周期。 */
#define LORA_STOPPED_PERIOD_MS                    100U /* STOPPED 重发间隔。 */
#define LORA_STOPPED_REPEAT_COUNT                   3U /* STOPPED 连续发送次数。 */

/* A 点终点判定：先经过 C、D，再按右侧先扫线、左侧后扫线的顺序确认。 */
#define LINE_FOLLOW_A_MARKER_ARM_DISTANCE_MM      6200U /* 达到该平均行程前忽略所有终点标记。 */
#define LINE_FOLLOW_A_MARKER_MIN_SIDE_SENSORS       2U /* 单侧横线标记至少需要检测到的黑线传感器数量；右侧与左侧合计为 4 路。 */
#define LINE_FOLLOW_A_MARKER_VALID_SAMPLES           3U /* 右侧和左侧标记各自需要连续满足的控制周期数。 */
#define LINE_FOLLOW_A_MARKER_SEQUENCE_MAX_DISTANCE_MM 300U /* 右侧标记到左侧标记允许的最大平均行程，单位：mm。 */

/* B、C 点分段减速参数：赛道 A 到 B 为 150 cm，B 到 C 为半径 75 cm 的右半圆。 */
#define LINE_FOLLOW_B_ENTER_DISTANCE_MM           1500U /* A 点起步后达到该平均行程即进入 B-C 半圆减速；实车进入 B 过早则减小，过晚则增大。 */
#define LINE_FOLLOW_BC_ARC_DISTANCE_MM             2356U /* 从确认 B 起累计该行程后视为离开 C；数值为 PI×750 mm，通常无需调整。 */
#define LINE_FOLLOW_CURVE_BASE_OUTPUT_RPM            22  /* B-C 半圆循迹基础速度；减小更稳，增大更快。 */

/* A 点停车制动参数。 */
#define LINE_FOLLOW_STOP_SPEED_RPM                  90  /* 判定电机停止的反馈转速阈值。 */
#define LINE_FOLLOW_STOP_SETTLE_TIME_MS            100U /* 低于停止阈值后需持续的时间，单位：ms。 */
#define LINE_FOLLOW_STOP_TIMEOUT_MS               1500U /* 主动 PID 制动的最长时间，单位：ms。 */

/* 循迹调参区：下列转速均为车轮输出转速，单位：rpm。 */
#define LINE_FOLLOW_START_DELAY_MS                3000U /* 上电后开始检测黑线前的等待时间。 */
#define LINE_FOLLOW_FEEDBACK_TIMEOUT_MS           100U  /* CAN 电机反馈丢失判定阈值。 */
#define LINE_FOLLOW_RUN_TIMEOUT_MS                85000U /* 连续运行的最大时间保护。 */
#define LINE_FOLLOW_BASE_OUTPUT_RPM               32    /* 直线循迹基础速度。 */
#define LINE_FOLLOW_RIGHT_BIAS_RPM                 6    /* 针对当前车架的恒定向右修正量。 */
#define LINE_FOLLOW_RIGHT_TURN_BOOST_RPM           3    /* 右转时额外增加的修正量。 */
#define LINE_FOLLOW_STRAIGHT_TRIM_TARGET_RPM     -2.0f  /* 直线专用补偿目标；正值向右修正，黑线偏车身左侧时使用负值向左修正。 */
#define LINE_FOLLOW_STRAIGHT_TRIM_RAMP_RPM        0.08f  /* 进入直线后每 10 ms 靠近目标补偿的最大变化量。 */
#define LINE_FOLLOW_STRAIGHT_TRIM_RELEASE_RPM     0.25f  /* 进入弯道后每 10 ms 回零的最大变化量。 */
#define LINE_FOLLOW_STRAIGHT_TURN_WINDOW_RPM      2.0f   /* 实际转向量接近恒定修正量时，判定为直线的允许范围。 */
#define LINE_FOLLOW_STRAIGHT_CONFIRM_SAMPLES        5U   /* 连续满足直线条件后才开始施加补偿的控制周期数。 */
#define LINE_FOLLOW_LOST_OUTPUT_RPM               12    /* 丢线搜索时的行驶速度。 */
#define LINE_FOLLOW_KP_RPM_PER_ERROR              3.0f  /* 增大可加强横向误差修正，支持小数。 */
#define LINE_FOLLOW_KD_RPM_PER_ERROR_DELTA        1.2f  /* 增大可加强误差突变时的响应，支持小数。 */
#define LINE_FOLLOW_MAX_D_TURN_RPM                4.0f  /* D 项转向修正的最大值，支持小数。 */
#define LINE_FOLLOW_MAX_TURN_RPM                  18.0f /* 总转向修正的最大值，支持小数。 */
#define LINE_FOLLOW_LOST_TURN_RPM                 8     /* 丢线搜索时的转向量。 */
#define LINE_FOLLOW_SENSOR_FILTER_SAMPLES         3U    /* 多数滤波使用的连续采样帧数。 */
#define LINE_FOLLOW_START_VALID_SAMPLES            3U    /* 在 A 点允许起步前要求的有效黑线帧数。 */
#define LINE_FOLLOW_ERROR_JUMP_LIMIT              3     /* 超过该位置跳变的传感器结果会被拒绝。 */
#define LINE_FOLLOW_TURN_SLEW_RPM                 3.0f  /* 每 10 ms 允许的最大转向变化量，支持小数。 */

/* 0x201、0x202 电机速度 PID 调参区，数值为 PID 内部单位。 */
#define MOTOR_SPEED_PID_MAX_OUTPUT              4500U  /* 电流命令输出限幅。 */
#define MOTOR_SPEED_PID_INTEGRAL_LIMIT          5000U  /* 积分累加限幅。 */
#define MOTOR_SPEED_PID_DEADBAND                   1.0f /* 小于该转速误差时不调节。 */
#define MOTOR_SPEED_PID_CONTROL_PERIOD             0U   /* pid.c 当前未使用，仅保留参数位置。 */
#define MOTOR_SPEED_PID_MAX_ERROR                8000  /* pid.c 当前未使用，仅保留参数位置。 */
#define MOTOR_SPEED_PID_KP                         0.4f /* 比例调节系数。 */
#define MOTOR_SPEED_PID_KI                         0.03f /* 积分调节系数。 */
#define MOTOR_SPEED_PID_KD                         0.0f /* 微分调节系数；当前关闭。 */

typedef enum
{
  LINE_FOLLOW_WAIT_FEEDBACK,
  LINE_FOLLOW_START_DELAY,
  LINE_FOLLOW_WAIT_LINE,
  LINE_FOLLOW_RUN,
  LINE_FOLLOW_SEARCH,
  LINE_FOLLOW_STOP_A,
  LINE_FOLLOW_DONE,
  LINE_FOLLOW_FAULT
} LineFollowState;

typedef struct
{
  LineFollowState state;
  uint32_t state_start_ms;
  int32_t encoder_count[2];
  int32_t start_count[2];
  uint32_t a_marker_arm_encoder_count;
  uint32_t a_marker_sequence_max_encoder_count;
  uint32_t a_marker_right_encoder_count;
  uint32_t b_enter_encoder_count;
  uint32_t bc_arc_encoder_count;
  uint32_t b_curve_start_encoder_count;
  uint32_t stop_settle_start_ms;
  uint16_t last_angle[2];
  int16_t last_error;
  int16_t last_control_error;
  int16_t last_nonzero_error;
  float last_turn_rpm;
  float straight_trim_rpm;
  uint8_t sensor_history[LINE_FOLLOW_SENSOR_FILTER_SAMPLES];
  uint8_t sensor_history_count;
  uint8_t valid_line_count;
  uint8_t straight_confirm_count;
  uint8_t a_marker_right_valid_count;
  uint8_t a_marker_left_valid_count;
  uint8_t a_marker_right_seen;
  uint8_t bc_curve_active;
  uint8_t bc_curve_finished;
  uint8_t encoder_ready;
} LineFollowControl;

typedef struct
{
  uint32_t last_ready_ms;
  uint32_t last_stopped_ms;
  uint8_t run_id;
  uint8_t rx_frame[LORA_FRAME_SIZE];
  uint8_t rx_index;
  uint8_t armed;
  volatile uint8_t start_received;
  uint8_t stopped_sent_count;
} LoraProtocol;

/* 传感器映射标定：仅在修改模块接线或安装位置后调整。 */
/* 原始位顺序为 L01、L02、L03、L04、R01、R02、R03、R04。 */
/* 物理从左到右顺序为 L01、L02、L03、L04、R04、R03、R02、R01。 */
static const int8_t line_sensor_weight[8] = {-7, -5, -3, -1, 7, 5, 3, 1};
static LineFollowControl line_follow = {LINE_FOLLOW_WAIT_FEEDBACK};
static LoraProtocol lora_protocol = {0};

static float LineFollow_Clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static uint8_t LineSensor_ReadRaw(void)
{
  uint8_t raw = 0U;

  /* Bit order follows the electrical DO channel mapping, not physical position. */
  raw |= (HAL_GPIO_ReadPin(TRACK_L_D01_GPIO_Port, TRACK_L_D01_Pin) == GPIO_PIN_SET) ? (1U << 0) : 0U;
  raw |= (HAL_GPIO_ReadPin(TRACK_L_D02_GPIO_Port, TRACK_L_D02_Pin) == GPIO_PIN_SET) ? (1U << 1) : 0U;
  raw |= (HAL_GPIO_ReadPin(TRACK_L_D03_GPIO_Port, TRACK_L_D03_Pin) == GPIO_PIN_SET) ? (1U << 2) : 0U;
  raw |= (HAL_GPIO_ReadPin(TRACK_L_D04_GPIO_Port, TRACK_L_D04_Pin) == GPIO_PIN_SET) ? (1U << 3) : 0U;
  raw |= (HAL_GPIO_ReadPin(TRACK_R_D01_GPIO_Port, TRACK_R_D01_Pin) == GPIO_PIN_SET) ? (1U << 4) : 0U;
  raw |= (HAL_GPIO_ReadPin(TRACK_R_D02_GPIO_Port, TRACK_R_D02_Pin) == GPIO_PIN_SET) ? (1U << 5) : 0U;
  raw |= (HAL_GPIO_ReadPin(TRACK_R_D03_GPIO_Port, TRACK_R_D03_Pin) == GPIO_PIN_SET) ? (1U << 6) : 0U;
  raw |= (HAL_GPIO_ReadPin(TRACK_R_D04_GPIO_Port, TRACK_R_D04_Pin) == GPIO_PIN_SET) ? (1U << 7) : 0U;

  return raw;
}

static uint8_t LineFollow_GetLineMask(void)
{
  uint8_t filtered_mask;

  /* The installed sensor module outputs high level over the black line. */
  if (line_follow.sensor_history_count < LINE_FOLLOW_SENSOR_FILTER_SAMPLES)
  {
    line_follow.sensor_history[line_follow.sensor_history_count] = line_sensor_raw;
    line_follow.sensor_history_count++;
    return 0U;
  }

  line_follow.sensor_history[0] = line_follow.sensor_history[1];
  line_follow.sensor_history[1] = line_follow.sensor_history[2];
  line_follow.sensor_history[2] = line_sensor_raw;
  filtered_mask = (uint8_t)((line_follow.sensor_history[0] & line_follow.sensor_history[1]) |
                            (line_follow.sensor_history[0] & line_follow.sensor_history[2]) |
                            (line_follow.sensor_history[1] & line_follow.sensor_history[2]));

  return filtered_mask;
}

static uint8_t LineFollow_ComputeError(uint8_t line_mask, int16_t *error)
{
  uint8_t index;
  uint8_t count = 0U;
  int16_t sum = 0;

  for (index = 0U; index < 8U; index++)
  {
    if ((line_mask & (1U << index)) != 0U)
    {
      sum += line_sensor_weight[index];
      count++;
    }
  }

  if (count != 0U)
  {
    *error = sum / (int16_t)count;
  }

  return count;
}

static uint8_t LineFollow_CountSideSensors(uint8_t side_mask)
{
  uint8_t count = 0U;

  while (side_mask != 0U)
  {
    count += side_mask & 1U;
    side_mask >>= 1U;
  }

  return count;
}

static void LineFollow_ResetAMarkerSequence(void)
{
  line_follow.a_marker_right_valid_count = 0U;
  line_follow.a_marker_left_valid_count = 0U;
  line_follow.a_marker_right_seen = 0U;
}

static uint32_t LineFollow_DistanceMmToEncoderCounts(uint32_t distance_mm)
{
  uint64_t numerator;

  numerator = (uint64_t)distance_mm * 1000U *
              LINE_FOLLOW_ENCODER_COUNTS_PER_MOTOR_REV * M2006_GEAR_RATIO;
  return (uint32_t)((numerator + (LINE_FOLLOW_WHEEL_CIRCUMFERENCE_UM / 2U)) /
                    LINE_FOLLOW_WHEEL_CIRCUMFERENCE_UM);
}

static uint32_t LineFollow_GetTravelEncoderCount(void)
{
  int32_t right_delta = line_follow.encoder_count[0] - line_follow.start_count[0];
  int32_t left_delta = line_follow.encoder_count[1] - line_follow.start_count[1];
  uint32_t right_count = (right_delta < 0) ? (uint32_t)(-right_delta) : (uint32_t)right_delta;
  uint32_t left_count = (left_delta < 0) ? (uint32_t)(-left_delta) : (uint32_t)left_delta;

  return (right_count + left_count) / 2U;
}

static void LineFollow_UpdateBCSpeedSegment(void)
{
  uint32_t travel_encoder_count = LineFollow_GetTravelEncoderCount();

  /* B 点只允许确认一次，避免后续经过其他弯道时重复进入半圆减速。 */
  if (line_follow.bc_curve_finished != 0U)
  {
    return;
  }

  if (line_follow.bc_curve_active == 0U)
  {
    /* 此处是 B 点编码器阈值判断。阈值在启动时由毫米参数自动换算。 */
    if (travel_encoder_count >= line_follow.b_enter_encoder_count)
    {
      line_follow.bc_curve_active = 1U;
      line_follow.b_curve_start_encoder_count = travel_encoder_count;
    }
    return;
  }

  /* 从 B 点起重新累计半圆弧长，到 C 点后恢复直线基础速度。 */
  if ((travel_encoder_count >= line_follow.b_curve_start_encoder_count) &&
      ((travel_encoder_count - line_follow.b_curve_start_encoder_count) >=
       line_follow.bc_arc_encoder_count))
  {
    line_follow.bc_curve_active = 0U;
    line_follow.bc_curve_finished = 1U;
  }
}

static float LineFollow_GetBaseOutputRpm(void)
{
  return (line_follow.bc_curve_active != 0U) ?
         (float)LINE_FOLLOW_CURVE_BASE_OUTPUT_RPM :
         (float)LINE_FOLLOW_BASE_OUTPUT_RPM;
}

static uint8_t LineFollow_HasReturnedToA(uint8_t line_mask)
{
  uint32_t travel_encoder_count;
  uint8_t left_count;
  uint8_t right_count;

  travel_encoder_count = LineFollow_GetTravelEncoderCount();
  if (travel_encoder_count < line_follow.a_marker_arm_encoder_count)
  {
    LineFollow_ResetAMarkerSequence();
    return 0U;
  }

  left_count = LineFollow_CountSideSensors(line_mask & 0x0FU);
  right_count = LineFollow_CountSideSensors((line_mask >> 4U) & 0x0FU);

  if (line_follow.a_marker_right_seen == 0U)
  {
    if (right_count < LINE_FOLLOW_A_MARKER_MIN_SIDE_SENSORS)
    {
      line_follow.a_marker_right_valid_count = 0U;
      return 0U;
    }

    if (line_follow.a_marker_right_valid_count < LINE_FOLLOW_A_MARKER_VALID_SAMPLES)
    {
      line_follow.a_marker_right_valid_count++;
    }

    if (line_follow.a_marker_right_valid_count >= LINE_FOLLOW_A_MARKER_VALID_SAMPLES)
    {
      line_follow.a_marker_right_seen = 1U;
      line_follow.a_marker_right_encoder_count = travel_encoder_count;
      line_follow.a_marker_left_valid_count = 0U;
    }

    return 0U;
  }

  if ((travel_encoder_count - line_follow.a_marker_right_encoder_count) >
      line_follow.a_marker_sequence_max_encoder_count)
  {
    LineFollow_ResetAMarkerSequence();
    return 0U;
  }

  if (left_count < LINE_FOLLOW_A_MARKER_MIN_SIDE_SENSORS)
  {
    line_follow.a_marker_left_valid_count = 0U;
    return 0U;
  }

  if (line_follow.a_marker_left_valid_count < LINE_FOLLOW_A_MARKER_VALID_SAMPLES)
  {
    line_follow.a_marker_left_valid_count++;
  }

  return (line_follow.a_marker_left_valid_count >= LINE_FOLLOW_A_MARKER_VALID_SAMPLES) ? 1U : 0U;
}

static uint8_t LineFollow_ErrorWithinJumpLimit(int16_t error, int16_t reference_error)
{
  if ((error > (reference_error + LINE_FOLLOW_ERROR_JUMP_LIMIT)) ||
      (error < (reference_error - LINE_FOLLOW_ERROR_JUMP_LIMIT)))
  {
    return 0U;
  }

  return 1U;
}

static void LineFollow_RecordError(int16_t error)
{
  line_follow.last_error = error;
  if (error != 0)
  {
    line_follow.last_nonzero_error = error;
  }
}

static float LineFollow_ApplyTurnSlew(float target_turn_rpm)
{
  if (target_turn_rpm > (line_follow.last_turn_rpm + LINE_FOLLOW_TURN_SLEW_RPM))
  {
    line_follow.last_turn_rpm += LINE_FOLLOW_TURN_SLEW_RPM;
  }
  else if (target_turn_rpm < (line_follow.last_turn_rpm - LINE_FOLLOW_TURN_SLEW_RPM))
  {
    line_follow.last_turn_rpm -= LINE_FOLLOW_TURN_SLEW_RPM;
  }
  else
  {
    line_follow.last_turn_rpm = target_turn_rpm;
  }

  return line_follow.last_turn_rpm;
}

static float LineFollow_Approach(float current_value, float target_value, float step)
{
  if (current_value < target_value)
  {
    current_value += step;
    if (current_value > target_value)
    {
      current_value = target_value;
    }
  }
  else if (current_value > target_value)
  {
    current_value -= step;
    if (current_value < target_value)
    {
      current_value = target_value;
    }
  }

  return current_value;
}

static void LineFollow_ResetStraightTrim(void)
{
  line_follow.straight_trim_rpm = 0.0f;
  line_follow.straight_confirm_count = 0U;
}

static void LineFollow_UpdateStraightTrim(float untrimmed_turn_rpm)
{
  float lower_limit = LINE_FOLLOW_RIGHT_BIAS_RPM - LINE_FOLLOW_STRAIGHT_TURN_WINDOW_RPM;
  float upper_limit = LINE_FOLLOW_RIGHT_BIAS_RPM + LINE_FOLLOW_STRAIGHT_TURN_WINDOW_RPM;

  if ((untrimmed_turn_rpm >= lower_limit) && (untrimmed_turn_rpm <= upper_limit))
  {
    if (line_follow.straight_confirm_count < LINE_FOLLOW_STRAIGHT_CONFIRM_SAMPLES)
    {
      line_follow.straight_confirm_count++;
    }
  }
  else
  {
    line_follow.straight_confirm_count = 0U;
  }

  if (line_follow.straight_confirm_count >= LINE_FOLLOW_STRAIGHT_CONFIRM_SAMPLES)
  {
    line_follow.straight_trim_rpm = LineFollow_Approach(
      line_follow.straight_trim_rpm,
      LINE_FOLLOW_STRAIGHT_TRIM_TARGET_RPM,
      LINE_FOLLOW_STRAIGHT_TRIM_RAMP_RPM);
  }
  else
  {
    line_follow.straight_trim_rpm = LineFollow_Approach(
      line_follow.straight_trim_rpm,
      0.0f,
      LINE_FOLLOW_STRAIGHT_TRIM_RELEASE_RPM);
  }
}

static uint8_t LineFollow_FeedbackFresh(uint32_t now_ms)
{
  uint8_t motor;

  for (motor = 0U; motor < 2U; motor++)
  {
    if ((motor_feedback[motor].valid == 0U) ||
        ((uint32_t)(now_ms - motor_feedback[motor].last_update_ms) > LINE_FOLLOW_FEEDBACK_TIMEOUT_MS))
    {
      return 0U;
    }
  }

  return 1U;
}

static void LineFollow_UpdateEncoder(void)
{
  uint8_t motor;

  if (line_follow.encoder_ready == 0U)
  {
    if ((motor_feedback[0].valid == 0U) || (motor_feedback[1].valid == 0U))
    {
      return;
    }

    for (motor = 0U; motor < 2U; motor++)
    {
      line_follow.last_angle[motor] = motor_feedback[motor].angle;
      line_follow.encoder_count[motor] = 0;
    }
    line_follow.encoder_ready = 1U;
    return;
  }

  for (motor = 0U; motor < 2U; motor++)
  {
    int16_t delta = (int16_t)(motor_feedback[motor].angle - line_follow.last_angle[motor]);

    if (delta > 4096)
    {
      delta -= 8192;
    }
    else if (delta < -4096)
    {
      delta += 8192;
    }

    line_follow.encoder_count[motor] += delta;
    line_follow.last_angle[motor] = motor_feedback[motor].angle;
  }
}

static uint8_t LoraProtocol_Checksum(const uint8_t *frame)
{
  return (uint8_t)(frame[2] + frame[3] + frame[4]);
}

static uint8_t LoraProtocol_SendFrame(uint8_t source, uint8_t command)
{
  uint8_t frame[LORA_FRAME_SIZE];

  if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET)
  {
    return 0U;
  }

  frame[0] = LORA_FRAME_HEADER_0;
  frame[1] = LORA_FRAME_HEADER_1;
  frame[2] = source;
  frame[3] = command;
  frame[4] = lora_protocol.run_id;
  frame[5] = LoraProtocol_Checksum(frame);

  return (HAL_UART_Transmit(&huart3, frame, LORA_FRAME_SIZE, 5U) == HAL_OK) ? 1U : 0U;
}

static void LoraProtocol_Arm(uint32_t now_ms)
{
  uint16_t seed;

  if (lora_protocol.armed != 0U)
  {
    return;
  }

  seed = motor_feedback[0].angle ^ motor_feedback[1].angle ^ (uint16_t)now_ms;
  lora_protocol.run_id = (uint8_t)seed;
  if (lora_protocol.run_id == 0U)
  {
    lora_protocol.run_id = 0xA5U;
  }
  lora_protocol.last_ready_ms = now_ms - LORA_READY_PERIOD_MS;
  lora_protocol.armed = 1U;
}

static void LoraProtocol_OnRxByte(uint8_t byte)
{
  if (lora_protocol.rx_index == 0U)
  {
    if (byte == LORA_FRAME_HEADER_0)
    {
      lora_protocol.rx_frame[0] = byte;
      lora_protocol.rx_index = 1U;
    }
    return;
  }

  if (lora_protocol.rx_index == 1U)
  {
    if (byte == LORA_FRAME_HEADER_1)
    {
      lora_protocol.rx_frame[1] = byte;
      lora_protocol.rx_index = 2U;
    }
    else if (byte == LORA_FRAME_HEADER_0)
    {
      lora_protocol.rx_frame[0] = byte;
    }
    else
    {
      lora_protocol.rx_index = 0U;
    }
    return;
  }

  lora_protocol.rx_frame[lora_protocol.rx_index] = byte;
  lora_protocol.rx_index++;

  if (lora_protocol.rx_index >= LORA_FRAME_SIZE)
  {
    if ((LoraProtocol_Checksum(lora_protocol.rx_frame) == lora_protocol.rx_frame[5]) &&
        (lora_protocol.rx_frame[2] == LORA_SOURCE_GROUND) &&
        (lora_protocol.rx_frame[3] == LORA_COMMAND_START) &&
        (lora_protocol.armed != 0U) &&
        (lora_protocol.rx_frame[4] == lora_protocol.run_id))
    {
      lora_protocol.start_received = 1U;
    }
    lora_protocol.rx_index = 0U;
  }
}

static void LoraProtocol_Task(uint32_t now_ms)
{
  if ((lora_protocol.armed != 0U) && (lora_protocol.start_received == 0U))
  {
    if ((uint32_t)(now_ms - lora_protocol.last_ready_ms) >= LORA_READY_PERIOD_MS)
    {
      if (LoraProtocol_SendFrame(LORA_SOURCE_CAR, LORA_COMMAND_READY) != 0U)
      {
        lora_protocol.last_ready_ms = now_ms;
      }
    }
    return;
  }

  if ((line_follow.state == LINE_FOLLOW_DONE) &&
      (lora_protocol.stopped_sent_count < LORA_STOPPED_REPEAT_COUNT) &&
      ((lora_protocol.stopped_sent_count == 0U) ||
       ((uint32_t)(now_ms - lora_protocol.last_stopped_ms) >= LORA_STOPPED_PERIOD_MS)))
  {
    if (LoraProtocol_SendFrame(LORA_SOURCE_CAR, LORA_COMMAND_STOPPED) != 0U)
    {
      lora_protocol.last_stopped_ms = now_ms;
      lora_protocol.stopped_sent_count++;
    }
  }
}

static void LineFollow_ResetPid(PID_TypeDef *pid)
{
  pid->target = 0.0f;
  pid->measure = 0.0f;
  pid->err = 0.0f;
  pid->last_err = 0.0f;
  pid->pout = 0.0f;
  pid->iout = 0.0f;
  pid->dout = 0.0f;
  pid->output = 0.0f;
  pid->last_output = 0.0f;
}

static uint8_t LineFollow_MotorsStopped(void)
{
  uint8_t motor;

  for (motor = 0U; motor < 2U; motor++)
  {
    if ((motor_feedback[motor].speed_rpm > LINE_FOLLOW_STOP_SPEED_RPM) ||
        (motor_feedback[motor].speed_rpm < -LINE_FOLLOW_STOP_SPEED_RPM))
    {
      return 0U;
    }
  }

  return 1U;
}

static void LineFollow_StartStopA(uint32_t now_ms)
{
  line_follow.state = LINE_FOLLOW_STOP_A;
  line_follow.state_start_ms = now_ms;
  line_follow.stop_settle_start_ms = 0U;
  LineFollow_ResetPid(&Motor_pid[0]);
  LineFollow_ResetPid(&Motor_pid[1]);
}

static void LineFollow_StartRun(uint32_t now_ms, int16_t initial_error)
{
  line_follow.start_count[0] = line_follow.encoder_count[0];
  line_follow.start_count[1] = line_follow.encoder_count[1];
  line_follow.a_marker_arm_encoder_count =
    LineFollow_DistanceMmToEncoderCounts(LINE_FOLLOW_A_MARKER_ARM_DISTANCE_MM);
  line_follow.a_marker_sequence_max_encoder_count =
    LineFollow_DistanceMmToEncoderCounts(LINE_FOLLOW_A_MARKER_SEQUENCE_MAX_DISTANCE_MM);
  /* 保留毫米宏定义供人工调参，编码器阈值随车轮直径和减速比自动重算。 */
  line_follow.b_enter_encoder_count =
    LineFollow_DistanceMmToEncoderCounts(LINE_FOLLOW_B_ENTER_DISTANCE_MM);
  line_follow.bc_arc_encoder_count =
    LineFollow_DistanceMmToEncoderCounts(LINE_FOLLOW_BC_ARC_DISTANCE_MM);
  line_follow.b_curve_start_encoder_count = 0U;
  line_follow.bc_curve_active = 0U;
  line_follow.bc_curve_finished = 0U;
  line_follow.stop_settle_start_ms = 0U;
  LineFollow_ResetAMarkerSequence();
  line_follow.state = LINE_FOLLOW_RUN;
  line_follow.state_start_ms = now_ms;
  line_follow.last_error = initial_error;
  line_follow.last_control_error = initial_error;
  line_follow.last_nonzero_error = (initial_error != 0) ? initial_error : 0;
  line_follow.last_turn_rpm = 0.0f;
  LineFollow_ResetStraightTrim();
  LineFollow_ResetPid(&Motor_pid[0]);
  LineFollow_ResetPid(&Motor_pid[1]);
}

static void LineFollow_StartSearch(uint32_t now_ms)
{
  (void)now_ms;
  line_follow.state = LINE_FOLLOW_SEARCH;
  LineFollow_ResetStraightTrim();
}

static uint8_t LineFollow_SendSpeedCommand(int16_t right_output_rpm, int16_t left_output_rpm)
{
  Motor_pid[0].target = -(right_output_rpm * M2006_GEAR_RATIO);
  Motor_pid[0].f_cal_pid(&Motor_pid[0], motor_feedback[0].speed_rpm);

  Motor_pid[1].target = left_output_rpm * M2006_GEAR_RATIO;
  Motor_pid[1].f_cal_pid(&Motor_pid[1], motor_feedback[1].speed_rpm);

  return Set_moto_current((s16)Motor_pid[0].output,
                          (s16)Motor_pid[1].output,
                          0,
                          0);
}

static int16_t LineFollow_RoundOutputRpm(float output_rpm)
{
  return (output_rpm >= 0.0f) ?
         (int16_t)(output_rpm + 0.5f) :
         (int16_t)(output_rpm - 0.5f);
}

static uint8_t LineFollow_SendTrackingCommand(int16_t line_error)
{
  float base_output_rpm;
  float error_delta;
  float d_turn_rpm;
  float target_turn_rpm;
  float turn_rpm;

  base_output_rpm = LineFollow_GetBaseOutputRpm();
  error_delta = (float)(line_error - line_follow.last_control_error);
  d_turn_rpm = LineFollow_Clamp(error_delta * LINE_FOLLOW_KD_RPM_PER_ERROR_DELTA,
                                 -LINE_FOLLOW_MAX_D_TURN_RPM,
                                 LINE_FOLLOW_MAX_D_TURN_RPM);
  target_turn_rpm = LineFollow_Clamp((line_error * LINE_FOLLOW_KP_RPM_PER_ERROR) + d_turn_rpm,
                                     -LINE_FOLLOW_MAX_TURN_RPM,
                                     LINE_FOLLOW_MAX_TURN_RPM);
  if (target_turn_rpm > 0.0f)
  {
    target_turn_rpm = LineFollow_Clamp(target_turn_rpm + LINE_FOLLOW_RIGHT_TURN_BOOST_RPM,
                                       -LINE_FOLLOW_MAX_TURN_RPM,
                                       LINE_FOLLOW_MAX_TURN_RPM);
  }
  LineFollow_UpdateStraightTrim(target_turn_rpm);
  target_turn_rpm = LineFollow_Clamp(target_turn_rpm + line_follow.straight_trim_rpm,
                                     -LINE_FOLLOW_MAX_TURN_RPM,
                                     LINE_FOLLOW_MAX_TURN_RPM);
  turn_rpm = LineFollow_ApplyTurnSlew(target_turn_rpm);
  turn_rpm = LineFollow_Clamp(turn_rpm,
                               LINE_FOLLOW_RIGHT_BIAS_RPM - base_output_rpm,
                               base_output_rpm + LINE_FOLLOW_RIGHT_BIAS_RPM);
  line_follow.last_turn_rpm = turn_rpm;
  line_follow.last_control_error = line_error;

  return LineFollow_SendSpeedCommand(
    LineFollow_RoundOutputRpm(base_output_rpm + LINE_FOLLOW_RIGHT_BIAS_RPM - turn_rpm),
    LineFollow_RoundOutputRpm(base_output_rpm - LINE_FOLLOW_RIGHT_BIAS_RPM + turn_rpm));
}

static uint8_t LineFollow_SendSearchCommand(void)
{
  float target_turn_rpm;
  float turn_rpm;

  if (line_follow.last_nonzero_error == 0)
  {
    line_follow.last_turn_rpm = 0.0f;
    return LineFollow_SendSpeedCommand(0, 0);
  }

  target_turn_rpm = (line_follow.last_nonzero_error > 0) ?
                    LINE_FOLLOW_LOST_TURN_RPM : -LINE_FOLLOW_LOST_TURN_RPM;
  turn_rpm = LineFollow_ApplyTurnSlew(target_turn_rpm);

  return LineFollow_SendSpeedCommand(
    LineFollow_RoundOutputRpm(LINE_FOLLOW_LOST_OUTPUT_RPM - turn_rpm),
    LineFollow_RoundOutputRpm(LINE_FOLLOW_LOST_OUTPUT_RPM + turn_rpm));
}

static void LineFollow_Control(void)
{
  uint32_t now_ms = HAL_GetTick();
  int16_t line_error = 0;
  uint8_t line_mask;
  uint8_t line_count;

  LineFollow_UpdateEncoder();
  line_mask = LineFollow_GetLineMask();

  switch (line_follow.state)
  {
    case LINE_FOLLOW_WAIT_FEEDBACK:
      if (Set_moto_current(0, 0, 0, 0) != 0U)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
        break;
      }
      if (LineFollow_FeedbackFresh(now_ms) != 0U)
      {
        LoraProtocol_Arm(now_ms);
        if (lora_protocol.start_received != 0U)
        {
          line_follow.state = LINE_FOLLOW_START_DELAY;
          line_follow.state_start_ms = now_ms;
        }
      }
      break;

    case LINE_FOLLOW_START_DELAY:
      if (Set_moto_current(0, 0, 0, 0) != 0U)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
        break;
      }
      if (LineFollow_FeedbackFresh(now_ms) == 0U)
      {
        line_follow.state = LINE_FOLLOW_WAIT_FEEDBACK;
      }
      else if ((uint32_t)(now_ms - line_follow.state_start_ms) >= LINE_FOLLOW_START_DELAY_MS)
      {
        line_follow.state = LINE_FOLLOW_WAIT_LINE;
        line_follow.valid_line_count = 0U;
      }
      break;

    case LINE_FOLLOW_WAIT_LINE:
      if (Set_moto_current(0, 0, 0, 0) != 0U)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
        break;
      }
      if (LineFollow_FeedbackFresh(now_ms) == 0U)
      {
        line_follow.state = LINE_FOLLOW_WAIT_FEEDBACK;
        break;
      }

      line_count = LineFollow_ComputeError(line_mask, &line_error);
      if (line_count == 0U)
      {
        line_follow.valid_line_count = 0U;
        break;
      }

      if ((line_follow.valid_line_count == 0U) ||
          (LineFollow_ErrorWithinJumpLimit(line_error, line_follow.last_error) != 0U))
      {
        LineFollow_RecordError(line_error);
        if (line_follow.valid_line_count < LINE_FOLLOW_START_VALID_SAMPLES)
        {
          line_follow.valid_line_count++;
        }
      }
      else
      {
        line_follow.valid_line_count = 1U;
        LineFollow_RecordError(line_error);
      }

      if (line_follow.valid_line_count >= LINE_FOLLOW_START_VALID_SAMPLES)
      {
        LineFollow_StartRun(now_ms, line_error);
      }
      break;

    case LINE_FOLLOW_RUN:
      if (LineFollow_FeedbackFresh(now_ms) == 0U)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
        break;
      }
      if ((uint32_t)(now_ms - line_follow.state_start_ms) >= LINE_FOLLOW_RUN_TIMEOUT_MS)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
        break;
      }
      LineFollow_UpdateBCSpeedSegment();
      if (LineFollow_HasReturnedToA(line_mask) != 0U)
      {
        LineFollow_StartStopA(now_ms);
        if (LineFollow_SendSpeedCommand(0, 0) != 0U)
        {
          line_follow.state = LINE_FOLLOW_FAULT;
        }
        break;
      }
      line_count = LineFollow_ComputeError(line_mask, &line_error);
      if ((line_count != 0U) &&
          (LineFollow_ErrorWithinJumpLimit(line_error, line_follow.last_error) != 0U))
      {
        LineFollow_RecordError(line_error);
        if (LineFollow_SendTrackingCommand(line_error) != 0U)
        {
          line_follow.state = LINE_FOLLOW_FAULT;
        }
      }
      else
      {
        LineFollow_StartSearch(now_ms);
        if (LineFollow_SendSearchCommand() != 0U)
        {
          line_follow.state = LINE_FOLLOW_FAULT;
        }
      }
      break;

    case LINE_FOLLOW_SEARCH:
      if (LineFollow_FeedbackFresh(now_ms) == 0U)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
        break;
      }
      if ((uint32_t)(now_ms - line_follow.state_start_ms) >= LINE_FOLLOW_RUN_TIMEOUT_MS)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
        break;
      }
      LineFollow_UpdateBCSpeedSegment();
      if (LineFollow_HasReturnedToA(line_mask) != 0U)
      {
        LineFollow_StartStopA(now_ms);
        if (LineFollow_SendSpeedCommand(0, 0) != 0U)
        {
          line_follow.state = LINE_FOLLOW_FAULT;
        }
        break;
      }

      line_count = LineFollow_ComputeError(line_mask, &line_error);
      if ((line_count != 0U) &&
          (LineFollow_ErrorWithinJumpLimit(line_error, line_follow.last_error) != 0U))
      {
        LineFollow_RecordError(line_error);
        line_follow.state = LINE_FOLLOW_RUN;
        if (LineFollow_SendTrackingCommand(line_error) != 0U)
        {
          line_follow.state = LINE_FOLLOW_FAULT;
        }
      }
      else if (LineFollow_SendSearchCommand() != 0U)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
      }
      break;

    case LINE_FOLLOW_STOP_A:
      if (LineFollow_SendSpeedCommand(0, 0) != 0U)
      {
        line_follow.state = LINE_FOLLOW_FAULT;
        break;
      }
      if (LineFollow_MotorsStopped() != 0U)
      {
        if (line_follow.stop_settle_start_ms == 0U)
        {
          line_follow.stop_settle_start_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - line_follow.stop_settle_start_ms) >= LINE_FOLLOW_STOP_SETTLE_TIME_MS)
        {
          line_follow.state = LINE_FOLLOW_DONE;
        }
      }
      else
      {
        line_follow.stop_settle_start_ms = 0U;
      }
      if ((uint32_t)(now_ms - line_follow.state_start_ms) >= LINE_FOLLOW_STOP_TIMEOUT_MS)
      {
        line_follow.state = LINE_FOLLOW_DONE;
      }
      break;

    case LINE_FOLLOW_DONE:
      Set_moto_current(0, 0, 0, 0);
      break;

    case LINE_FOLLOW_FAULT:
    default:
      Set_moto_current(0, 0, 0, 0);
      break;
  }
}

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  CanTxBuf[0]=0x02;
  CanTxBuf[1]=0x02;
  CanTxBuf[2]=0xFD;
  CanTxBuf[3]=0xFE;  
  CanTxBuf[4]=0x01;
  CanTxBuf[5]=0x01;  
  CanTxBuf[6]=0x01;
  CanTxBuf[7]=0x01;    
  CAN_Filter_Init();
  Set_moto_current(0, 0, 0, 0);
  if (HAL_UART_Receive_IT(&huart3, &lora_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

#if 0

HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);//����CH1��PWM�����
               // ????????
							 
//  HAL_CAN_Start(&hcan);//����CAN1
//  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�
//  USART1->SR &= ~(1 << 5);
  HAL_UART_Receive_IT(&huart1,RxBuf,1);
	HAL_UART_Receive_IT(&huart2, &Uart2RxByte, 1); //g

#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*��ʼ��PID����*/
  
  for(int i=0; i<2; i++) //g
  {	
    pid_init(&Motor_pid[i]);
    Motor_pid[i].f_param_init(&Motor_pid[i], PID_Speed,
                              MOTOR_SPEED_PID_MAX_OUTPUT,
                              MOTOR_SPEED_PID_INTEGRAL_LIMIT,
                              MOTOR_SPEED_PID_DEADBAND,
                              MOTOR_SPEED_PID_CONTROL_PERIOD,
                              MOTOR_SPEED_PID_MAX_ERROR,
                              0,
                              MOTOR_SPEED_PID_KP,
                              MOTOR_SPEED_PID_KI,
                              MOTOR_SPEED_PID_KD);
  }


#if 0
  while (1)
  {

if (DutyUpdateFlag)
{
    DutyUpdateFlag = 0; //g
    duty = PendingDuty; //g
   
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,duty); //g
}
//    HAL_UART_Transmit(&huart1,TxBuf,sizeof(TxBuf),100);//HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//    if( HAL_UART_Receive(&huart1,RxBuf,sizeof(RxBuf),500)== HAL_OK)  //HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//    {
//       HAL_Delay(10);
//       HAL_UART_Transmit(&huart1,RxBuf,sizeof(RxBuf),100);
 //   }
      
//     CAN_Send_Msg(CanTxBuf,8);
				if (should_send_stop_cmd == 1)
			{
					// ���Ӽ�¼��ʱ�俪ʼ���Ƿ��Ѿ���ȥ��5000���루5�룩
					if (HAL_GetTick() - command_timer_start >= 5000)
					{
							uint8_t tx_buffer[1]; 
							tx_buffer[0] = 0x00; // ׼������ֹͣ����
//							HAL_UART_Transmit(&huart2, tx_buffer, 1, 100);	//0916 ���˾�ע�� HTG
							
							should_send_stop_cmd = 0; // �����־λ����ֹ�ظ�����
					}
			}
			if (ChangeMode == 0x00)
      {

					if (decelerating) {
										// �𲽼�С�ٶ�Ŀ��ֵ
										if (Target_SpeedL > 0) Target_SpeedL -= DECAY_STEP*3;
										if (Target_SpeedR > 0) Target_SpeedR -= DECAY_STEP*3;

										// ȷ���ٶȲ��Ḻ��
										if (Target_SpeedL < new_speedL) Target_SpeedL = new_speedL;
										if (Target_SpeedR < new_speedR) Target_SpeedR = new_speedR;

										// ���� PID �ٶȵ���
										Motor_pid[0].target = -Target_SpeedR;
										speed_rpm[0] = CanRxBuf1[2] << 8 | CanRxBuf1[3];
										Motor_pid[0].f_cal_pid(&Motor_pid[0], speed_rpm[0]);

										Motor_pid[1].target = Target_SpeedL;
										speed_rpm[1] = CanRxBuf2[2] << 8 | CanRxBuf2[3];
										Motor_pid[1].f_cal_pid(&Motor_pid[1], speed_rpm[1]);

										Motor_pid[2].target = Target_Speed3; //g
										speed_rpm[2] = CanRxBuf3[2] << 8 | CanRxBuf3[3]; //g
										Motor_pid[2].f_cal_pid(&Motor_pid[2], speed_rpm[2]); //g

										// ���͵����
										Set_moto_current(Motor_pid[0].output, Motor_pid[1].output, Motor_pid[2].output, 0x0000); //g

										// ����Ƿ����Ŀ���ٶ�
										if (Target_SpeedL == new_speedL && Target_SpeedR == new_speedR) {
												decelerating = 0;  // ֹͣ����
										}
										HAL_Delay(50);  // ���Ƽ���Ƶ��
								}
					Motor_pid[0].target = -Target_SpeedR;
				  speed_rpm[0] = CanRxBuf1[2]<<8|CanRxBuf1[3]; //
					Motor_pid[0].f_cal_pid(&Motor_pid[0],speed_rpm[0]);    //�����趨ֵ����PID���㡣
					
					Motor_pid[1].target = Target_SpeedL;
					speed_rpm[1] = CanRxBuf2[2]<<8|CanRxBuf2[3];
					Motor_pid[1].f_cal_pid(&Motor_pid[1],speed_rpm[1]);
					
					Motor_pid[2].target = Target_Speed3; //g
					speed_rpm[2] = CanRxBuf3[2]<<8|CanRxBuf3[3]; //g
					Motor_pid[2].f_cal_pid(&Motor_pid[2],speed_rpm[2]); //g    //�����趨ֵ����PID���㡣
					
					Set_moto_current(   Motor_pid[0].output,   //��PID�ļ�����ͨ��CAN���͵����
															Motor_pid[1].output,
															Motor_pid[2].output, //g
															0x0000);
				
                                      HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
				
				 HAL_Delay(10);      //PID����Ƶ��100HZ
				 ChangeMode = 0x00;
      }
	}
}
   
#endif

  uint32_t control_tick = HAL_GetTick();
  while (1)
  {
    LoraProtocol_Task(HAL_GetTick());
    if ((uint32_t)(HAL_GetTick() - control_tick) >= 10U)
    {
      control_tick += 10U;
      line_sensor_raw = LineSensor_ReadRaw();
      LineFollow_Control();
    }
  }
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  
  /* USER CODE END CAN_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1439;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_M1_Pin */
  GPIO_InitStruct.Pin = LORA_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_M1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_AUX_Pin */
  GPIO_InitStruct.Pin = LORA_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRACK_R_D01_Pin TRACK_R_D02_Pin TRACK_R_D03_Pin TRACK_R_D04_Pin */
  GPIO_InitStruct.Pin = TRACK_R_D01_Pin|TRACK_R_D02_Pin|TRACK_R_D03_Pin|TRACK_R_D04_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TRACK_L_D01_Pin TRACK_L_D02_Pin TRACK_L_D03_Pin TRACK_L_D04_Pin */
  GPIO_InitStruct.Pin = TRACK_L_D01_Pin|TRACK_L_D02_Pin|TRACK_L_D03_Pin|TRACK_L_D04_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	
	uint8_t tx_buffer[1];

  if (huart->Instance == USART3)
  {
    LoraProtocol_OnRxByte(lora_rx_byte);
    HAL_UART_Receive_IT(&huart3, &lora_rx_byte, 1U);
    return;
  }

  if ( huart->Instance == USART1)
  {
      if (Motor3RxPending) //g
      { //g
          Motor3OutputRpm = (int16_t)(((uint16_t)Motor3RxBuf[0] << 8) | Motor3RxBuf[1]); //g
          if (Motor3OutputRpm > MOTOR3_MAX_OUTPUT_RPM) //g
          { //g
              Motor3OutputRpm = MOTOR3_MAX_OUTPUT_RPM; //g
          } //g
          else if (Motor3OutputRpm < -MOTOR3_MAX_OUTPUT_RPM) //g
          { //g
              Motor3OutputRpm = -MOTOR3_MAX_OUTPUT_RPM; //g
          } //g
          Target_Speed3 = (int16_t)(Motor3OutputRpm * MOTOR3_GEAR_RATIO); //g
          Motor_pid[2].iout = 0; //g
          Motor3RxPending = 0; //g
          RxCnt = 0; //g
          HAL_UART_Receive_IT(&huart1,RxBuf,1); //g
          return; //g
      } //g
      CurrentStatus = RxBuf[0];
      if (CurrentStatus == 0xA5) //g
      { //g
          Motor3RxPending = 1; //g
          HAL_UART_Receive_IT(&huart1,Motor3RxBuf,2); //g
          return; //g
      } //g
      if (CurrentStatus == 0x55)
      {
            HAL_UART_Receive_IT(&huart1,RxBuf,4); 
      }
      else   
      {
            HAL_UART_Receive_IT(&huart1,RxBuf,1); 
            if (RxCnt==4)  RxFlag = 0x11;
            else
            RxFlag = 0x00;            
      }
          if (RxFlag == 0x11)
          {	
	
              switch( CurrentStatus )
					{
						
						
						case 0x41: {


																																	decelerating = 0;  // ȡ������ͣ��

																																	int16_t new_speedL = RxBuf[1] * SpeedStep * 2;//ԭΪ1.8 //0915ԭΪ2
																																	int16_t new_speedR = RxBuf[1] * SpeedStep * 2;//ԭΪ1.8

																																	// �����ٶȱ仯��
																																	int16_t delta_speedL = Target_SpeedL - new_speedL;
																																	int16_t delta_speedR = Target_SpeedR - new_speedR;

																																	// �𲽵ݼ�
																																	if (delta_speedL > DECAY_STEP) {
																																			decelerating = 1; // ��������ģʽ
																																	} else {
																																			Target_SpeedL = new_speedL;
																																	}

																																	if (delta_speedR > DECAY_STEP) {
																																			decelerating = 1; // ��������ģʽ
																																	} else {
																																			Target_SpeedR = new_speedR;
																																	}

																																	Pid_en = 0x01;
																																	LastStatus = CurrentStatus;
																																	ChangeMode = 0x00;
																																	break;
																															}




						case 0x42:  
                                                                 
																																	decelerating = 1;  // �������ٹ���
																																	LastStatus = CurrentStatus;
																																	CurrentStatus = 0x00;
																																	ChangeMode = 0x00;
																																	break;
						
						
                                                                
						case 0x44: 	
																											Target_SpeedL = RxBuf[1]*SpeedStep*2;
                                                      Target_SpeedR = RxBuf[1]*SpeedStep;
                                                      Pid_en = 0x01;
                                                      LastStatus = CurrentStatus;
                                                      ChangeMode = 0x00;
																											rep();
                                                      break;//��ת
                        
						case 0x43: 	
																											Target_SpeedL = RxBuf[1]*SpeedStep;	
																											Target_SpeedR = RxBuf[1]*SpeedStep*1.5; //0916 ��ת�ٶ���΢��һ�㣬��ֹ���� 2->>1.5
																											LastStatus = CurrentStatus;
																											Pid_en = 0x01;
																											ChangeMode = 0x00;
																											rep();
																											break;//��ת
						case 0x4B: 	
													Target_SpeedL = RxBuf[1]*SpeedStep;
																											Target_SpeedR = -RxBuf[1]*SpeedStep;
																											Pid_en = 0x01;
																											LastStatus = CurrentStatus;
																											ChangeMode = 0x00;
																											break;//�ҵ�ͷ
						case 0x4A: 	
													Target_SpeedL = -RxBuf[1]*SpeedStep;
																											Target_SpeedR = RxBuf[1]*SpeedStep;
																											Pid_en = 0x01;
																											LastStatus = CurrentStatus;
																											ChangeMode = 0x00;
																											break;//���ͷ
						case 0x45: 	
													Target_SpeedL = -RxBuf[1]*SpeedStep;
																											Target_SpeedR = -RxBuf[1]*SpeedStep;
																											Pid_en = 0x01;
																											LastStatus = CurrentStatus;
																											ChangeMode = 0x00;
																											rep();
																											break;//����
						
							  case 0x46: 
																											LED0_Turn();
																											LastStatus = CurrentStatus;
																											CurrentStatus = 0x00;
																											ChangeMode = 0x00;
																											break;//���ڵƲ���
							  case 0x47:
                tx_buffer[0] = 0x47;
                HAL_UART_Transmit(&huart2, tx_buffer, 1, 100); // ͨ������2���� 0x01
								should_send_stop_cmd = 1;                    // ���ñ�־λ
								command_timer_start = HAL_GetTick();         // ��¼��ǰʱ��
                break;
            case 0x4E:
                tx_buffer[0] = 0x4E;
                HAL_UART_Transmit(&huart2, tx_buffer, 1, 100); // ͨ������2���� 0x02
								should_send_stop_cmd = 1;                    // ���ñ�־λ
								command_timer_start = HAL_GetTick();         // ��¼��ǰʱ��
                break;

            case 0x4F:
                tx_buffer[0] = 0x4F;
                HAL_UART_Transmit(&huart2, tx_buffer, 1, 100); // ͨ������2���� 0x03
								should_send_stop_cmd = 1;                    // ���ñ�־λ
								command_timer_start = HAL_GetTick();         // ��¼��ǰʱ��
                break;
            // 第三电机命令由独立USART1协议处理 //g
//							case 0x49: 	
//																										  Target_SpeedL = 0x0000;
//																											Target_SpeedR = 0x0000;
//																											LastStatus = CurrentStatus;
//																											pwmM1 = Target_SpeedO;
//																											ChangeMode = 0x01;
//																											break;//չ��
//            case 0x48: 	
//                                                               Target_SpeedL = 0x0000;
//                                                                Target_SpeedR = 0x0000;
//                                                                LastStatus = CurrentStatus;
//																																pwmM1 = Target_SpeedC;
//																															ChangeMode = 0x01;
//																																break;//���� 
//						
//            
//						case 0x4D: 	
//                                                               Target_SpeedL = 0x0000;
//                                                                Target_SpeedR = 0x0000;
//                                                                LastStatus = CurrentStatus;
//																																pwmM1 = Target_SpeedD;
//																																ChangeMode = 0x01;
//																																break;//չ����һ���Ƕ�
//						case 0x49: 	// չ�� (λ��-0)   0917�����޸ģ������PWMͨ�Ÿ�ΪCANͨ�ţ�ID�ŷֱ�Ϊ1��2��ID������λ���н������ã���������0��1000��2000����λֵ������λ��ֵͨ��ʵ�ʲ��������� RHS
//						{

//							uint8_t cmd_motor1[] = {0x55, 0xAA, 0x04, 0x01, 0x21, 0x37, 0x00, 0x00, 0x5D};
//							uint8_t cmd_motor2[] = {0x55, 0xAA, 0x04, 0x02, 0x21, 0x37, 0x00, 0x00, 0x5E};

//							HAL_UART_Transmit(&huart2, cmd_motor1, sizeof(cmd_motor1), 100);
//						
//							HAL_UART_Transmit(&huart2, cmd_motor2, sizeof(cmd_motor2), 100);

//							LastStatus = CurrentStatus;
//							break;
//						}

//            case 0x48: 	// ���� (λ��-2000)
//						{
//							uint8_t cmd_motor1[] = {0x55, 0xAA, 0x04, 0x01, 0x21, 0x37, 0xD0, 0x07, 0x34};
//							uint8_t cmd_motor2[] = {0x55, 0xAA, 0x04, 0x02, 0x21, 0x37, 0xD0, 0x07, 0x35};

//							HAL_UART_Transmit(&huart2, cmd_motor1, sizeof(cmd_motor1), 100);
//					
//							HAL_UART_Transmit(&huart2, cmd_motor2, sizeof(cmd_motor2), 100);

//							LastStatus = CurrentStatus;
//							break;
//						}
//						
//            
//						case 0x4D: 	// չ����һ���Ƕ� (λ��-1000)
//						{
//							uint8_t cmd_motor1[] = {0x55, 0xAA, 0x04, 0x01, 0x21, 0x37, 0xE8, 0x03, 0x48};
//							uint8_t cmd_motor2[] = {0x55, 0xAA, 0x04, 0x02, 0x21, 0x37, 0xE8, 0x03, 0x49};

//							HAL_UART_Transmit(&huart2, cmd_motor1, sizeof(cmd_motor1), 100);
//							
//							HAL_UART_Transmit(&huart2, cmd_motor2, sizeof(cmd_motor2), 100);

//							LastStatus = CurrentStatus;
//							break;
				//		}
						default:    		                    break;
                      }
               







					}
      RxFlag = 0x00;
      RxCnt=0;

			HAL_UART_Receive_IT(&huart1,RxBuf,1);  
  }
	else if (huart->Instance == USART2)//16
 {
	 
	 
	 
	  if (Uart2RxByte >= 25 && Uart2RxByte <= 125)
        {
            PendingDuty = Uart2RxByte;
//            DutyUpdateFlag = 1;
					duty = PendingDuty; //g
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,duty); //g
        }
        HAL_UART_Receive_IT(&huart2, &Uart2RxByte, 1);
    //    // 1. ���֡ͷ�Ƿ���ȷ (AA 55)
//    if (huart2_rx_buffer[0] == 0xAA && huart2_rx_buffer[1] == 0x55)
//    {
//      // 2. �������ID����������֡�ĵ�3���ֽ� (0-indexed)
//      uint8_t motor_id = huart2_rx_buffer[3];

//      // 3. ������ǰʵ��λ�ã����ڵ�8�͵�9���ֽ�
//      // �����ǵ��ֽ���ǰ�����ֽ��ں�
//      int16_t current_position = (int16_t)(huart2_rx_buffer[9] << 8 | huart2_rx_buffer[8]);

//      // 4. ���ݵ��ID������������λ�ô����Ӧ��ȫ�ֱ���
//      if (motor_id == 1)
//      {
//        motor1_current_position = current_position;
//      }
//      else if (motor_id == 2)
//      {
//        motor2_current_position = current_position;
//      }
//    }

//    // 5. ���������Ƿ���Ч���������������������жϣ���׼��������һ�����ݰ�
//    HAL_UART_Receive_IT(&huart2, huart2_rx_buffer, UART2_RX_BUFFER_SIZE);
  }

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */

}

void LED0_Turn(void)
{
   HAL_GPIO_TogglePin( LED0_GPIO_Port, LED0_Pin);
}

void rep(void)
{
if (decelerating) {
				decelerating = 0;
					// ��ѡ������Ӧ�õ�ǰ�׶��ٶ�
				Target_SpeedL = Target_SpeedL;
				Target_SpeedR = Target_SpeedR;
				}
}


/**
 * @brief  ???????????(???????)
 * @param  target: ??CCR?
 */
void Actuator_Move_To(uint16_t target)
{
  actuator_target_ccr = target;
  actuator_move_flag = 1;
}
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
