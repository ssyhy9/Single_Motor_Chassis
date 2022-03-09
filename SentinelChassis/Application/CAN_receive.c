
#include "main.h"
#include "CAN_receive.h"

motor_measure_t Motor_Message;

extern CAN_HandleTypeDef hcan1;

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/**
 * @brief:        Send Data to the APP layer to do data processing when receiving data from CAN port
 * @param[in]:    hcan: The CAN port that have data transmitted in
 * @param[out]:   rx_data: The data received from HAL layer, to be transmitted to the APP layer
 * @retvel:       None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //Get Data From HAL Layer

    get_motor_measure(&Motor_Message, rx_data);
}


void APP_Send_Msg_to_Motor(uint16_t speed) {
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t DATA[8] = { 0 };

	TxHeader.StdId = CAN_CHASSIS_ID;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	DATA[0] = speed >> 8;
	DATA[1] = speed;

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, DATA, &send_mail_box);
}

/**
  * @brief          return the side board data point
  * @param[in]      none
  * @retval         side board data point
  */
const motor_measure_t *get_side_board_data_point(void)
{
    return &Motor_Message;
}
