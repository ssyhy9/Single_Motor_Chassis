/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *  V2.0.0     Jan-26-2021     YW              1. modify to fit this project
  *  V3.0.0     Oct-27-2021     SPY             1. AGV control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "bsp_can.h"

/* CAN send and receive ID */
/**
  * motor_chassis[0]: MA motor1 3508
  * motor_chassis[1]: MA motor2 3508
  * motor_chassis[2]: MA motor3 3508
  * motor_chassis[3]: MA motor4 3508
  * motor_chassis[4]: MB motor1 2006
  * motor_chassis[5]: MB motor2 2006
  * motor_chassis[6]: MB motor3 2006
  * motor_chassis[7]: MB motor4 2006
  */
typedef enum
{
    CAN_CHASSIS_ID = 0x200,
    CAN_3508_ID = 0x201,

} can_msg_id_e;

typedef struct {
	CAN_RxHeaderTypeDef MotorRxMsgHeader;
	int16_t AngRaw;
	int16_t SpdRaw;
	int16_t TorqueCurrentRaw;
	int8_t TemperatureRaw;
} Motor_RxMsg_Raw;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
} motor_measure_t;

typedef struct {
	CAN_TxHeaderTypeDef MotorTxMsgHeader;
	uint8_t DATA[8];
} Motor_TxMsg;

typedef struct {
	int16_t CurrentSet_Ang;
	int16_t CurrentSet_Spd;
} Board_CMD;

typedef struct {
	Motor_RxMsg_Raw MotorRxMsgRaw;
	Motor_TxMsg MotorTxMsg;
	Board_CMD CenterBoardCMD;
} Wheel_Board_Msg;


extern const motor_measure_t *get_side_board_data_point(void);

extern void APP_Send_Msg_to_Motor(uint16_t speed);

#endif
