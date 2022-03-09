/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *  V2.0.0     Jan-26-2021     YW              1. modify to fit this project
  *  V3.0.0     Oct-27-2021     SPY             1. AGV control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

//disable chassis_power_control
#define CHASSIS_POWER_CONTROL_LIB_NEED

#include <math.h>
#include "chassis_task.h"
//#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "user_lib.h"
#include "pid.h"  //sgersgser
#include "remote_control.h"
#include "CAN_receive.h"

#ifndef CHASSIS_POWER_CONTROL_LIB_NEED
#include "chassis_power_control.h"
#endif

//deadband limit for channel 0, 1, 4 of remote controller that used in chassis control
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }											        \

//chassis move data
chassis_motor_t motor_system;

//prototype of all functions
static void chassis_init(chassis_motor_t *chassis_move_init);

static void chassis_feedback_update(chassis_motor_t *chassis_move_update);

void chassis_rc_to_control_vector(chassis_motor_t *chassis_move_rc_to_vector);

static void chassis_control_loop(chassis_motor_t *chassis_move_control_loop);


//#if INCLUDE_uxTaskGetStackHighWaterMark
//uint32_t chassis_high_water;				//transplanted from DJI, though I don't know what it used for
//#endif


/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms), including:
  * 				->chassis initialization
  * 				->motor and rc data updating (in while(1))
  *					->set all the "set_point" according to AGV movement calculation
  *					->PID control
  *					->send all current command through CAN
  * @param[in]      pvParameters: null
  * @retval         none
  */
void chassis_task(void *pvParameters)
{
    //wait for a period initially
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    //chassis initialization
    chassis_init(&motor_system);

    while (1)
    {

        //it was included in "chassis_set_control", temporarily added for testing
        chassis_rc_to_control_vector(&motor_system);

        //chassis control pid calculate
        chassis_control_loop(&motor_system);

        //send command current to motors via CAN1
        APP_Send_Msg_to_Motor(motor_system.give_current);
       // APP_Send_Msg_to_Motor(1000);

        //OS delay
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

//#if INCLUDE_uxTaskGetStackHighWaterMark
//        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }//end of "while(1)"
}

/**
  * @brief          "chassis_move" initialization, including:
  *                 ->pid initialization
  *                 ->3508 chassis motors data pointer initialization
  *                 ->2006 chassis motor data pointer initialization
  * @param[out]     chassis_move_init: "chassis_move" pointer
  * @retval         none
  */
static void chassis_init(chassis_motor_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis m3508 motor speed PID
    const static float32_t m3508_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
    
    //get RC data pointer
    chassis_move_init->chassis_RC = get_remote_control_point();

	//get CAN data pointer
	chassis_move_init->wheel_measure = get_side_board_data_point();

	//initialize m3508 speed PID
	PID_init(&chassis_move_init->m3508_speed_pid, PID_POSITION, m3508_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);

    //update data
    chassis_feedback_update(chassis_move_init);

}

/**
  * @brief          update some chassis measured data
  *                 such as rc data, euler angle, self-rotation speed, motor speed and robot speed
  * @param[out]     chassis_move_update: "chassis_move" pointer
  * @retval         none
  */
static void chassis_feedback_update(chassis_motor_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    //update the values return from remote controller
    chassis_move_update->chassis_RC = get_remote_control_point();
}

/**
  * @brief          according to the channel value of remote controller, calculate
  *                 chassis vertical and horizontal speed set-point
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" pointer
  * @retval         none
  */
void chassis_rc_to_control_vector(chassis_motor_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    
    int16_t x_channel;			//original data return from remote controller's channel 1
    int16_t y_channel;			//original data return from remote controller's channel 0
    int16_t wz_channel;

    chassis_feedback_update(chassis_move_rc_to_vector);

	//original data return from remote controller's channel 4

    //update the values get from remote controller
    //pay attention to the sign of "y" here
    x_channel = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL];
    y_channel = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL];
    wz_channel = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];

    //dead zone limit, because some remote control need be calibrated
    //the value of joy stick is not zero in middle place
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], x_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], y_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

    chassis_move_rc_to_vector->x_channel_origin = x_channel;
    chassis_move_rc_to_vector->y_channel_origin = y_channel;
    chassis_move_rc_to_vector->wz_channel_origin = wz_channel;
    chassis_move_rc_to_vector->x_set = x_channel * CHASSIS_X_RC_SEN;
    chassis_move_rc_to_vector->y_set = y_channel * CHASSIS_Y_RC_SEN;
    chassis_move_rc_to_vector->wz_set = wz_channel * CHASSIS_WZ_RC_SEN;

    chassis_move_rc_to_vector->speed = M3508_MOTOR_RPM_TO_VECTOR * chassis_move_rc_to_vector->wheel_measure->speed_rpm;

    chassis_move_rc_to_vector->speed_set = chassis_move_rc_to_vector->x_set;
}

/**
  * @brief          control loop. According to control set-point, calculate
  *                 motor current, and motor currents will be sent to motors
  * @param[out]     chassis_move_control_loop: "chassis_move" pointer
  * @retval         none
  */
static void chassis_control_loop(chassis_motor_t *chassis_move_control_loop)
{
	//limit the maximum speed of m3508
//	if (chassis_move_control_loop->speed > MAX_WHEEL_SPEED){
//		chassis_move_control_loop->speed_set = MAX_WHEEL_SPEED;
//	}
	chassis_move_control_loop->current = chassis_move_control_loop->wheel_measure->given_current;

	//speed-closed-loop of spd_motor
	chassis_move_control_loop->give_current = 1.5 * (int16_t)PID_calc(&chassis_move_control_loop->m3508_speed_pid, \
			chassis_move_control_loop->speed, chassis_move_control_loop->speed_set);
}

