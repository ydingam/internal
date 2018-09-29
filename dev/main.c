/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @brief This file is based on the MAPLE MINI example from ChibiOS
 *
 * @file main.c
 * @author Edward Zhang
 * @date 2018-09-26
 */

#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "canBusProcess.h"
#include "math_misc.h"

#include "shell.h"
#include "chprintf.h"
#include <stdlib.h>

static RC_Ctl_t* rc;

static int16_t motor_output[4];

static int16_t motor_speed_sp[4];
static int16_t motor_speed[4];
static float motor_error_int[4];

//These are the parameters of PID controller
const float chassis_kp = 8;
const float chassis_ki = 0.06;
const float chassis_kd = 0;

static lpfilterStruct speed_lpf[4];

static void drive_meccanum(const int16_t strafe, const int16_t drive, const int16_t rotation)
{
    //TODO drive the four meccanum wheel individually by using meccanum wheel kinematics
    motor_speed_sp[FL_WHEEL] = strafe + drive + rotation;
    motor_speed_sp[FR_WHEEL] = strafe - drive + rotation;
    motor_speed_sp[BR_WHEEL] = -strafe - drive + rotation;
    motor_speed_sp[BL_WHEEL] = -strafe + drive + rotation;
}

static int16_t pid_control(const int16_t setPoint, const int16_t current, float* error_int)
{
    int16_t error = setPoint - current;
    static int16_t previous_error = 0;

    //========TODO complete the PID control THD_FUNCTION===========
    int16_t output = chassis_kp * error;

    *error_int += error * chassis_ki;
    if(*error_int > 10000)
        *error_int = 10000;
    else if(*error_int < -10000)
        *error_int = -10000;


    output += *error_int + chassis_kd * (error - previous_error);
    //=============================================================
    previous_error = error;

    if(output > 10000)
        output = 10000;
    else if(output < -10000)
        output = -10000;

    return output;
}

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
	int16_t strafe = 0, drive = 0, rotation = 0;
    Encoder_canStruct* encoder = can_getEncoder();

	while(true)
	{
		strafe = (rc->channel0 - 1024)*8000.0f/1320.0f;
        drive = (rc->channel1 - 1024)*8000.0f/1320.0f;
        rotation = (rc->channel2 - 1024)*8000.0f/1320.0f;

        drive_meccanum(strafe, drive, rotation);

        motor_output[FL_WHEEL] =
            pid_control(motor_speed_sp[FL_WHEEL], encoder[FL_WHEEL].speed_rpm, &motor_error_int[FL_WHEEL]);
        motor_output[FR_WHEEL] =
            pid_control(motor_speed_sp[FR_WHEEL], encoder[FR_WHEEL].speed_rpm, &motor_error_int[FR_WHEEL]);
        motor_output[BR_WHEEL] =
            pid_control(motor_speed_sp[BR_WHEEL], encoder[BR_WHEEL].speed_rpm, &motor_error_int[BR_WHEEL]);
        motor_output[BL_WHEEL] =
            pid_control(motor_speed_sp[BL_WHEEL], encoder[BL_WHEEL].speed_rpm, &motor_error_int[BL_WHEEL]);

	    can_motorSetCurrent(0x200,
		    motor_output[FL_WHEEL], motor_output[FR_WHEEL], motor_output[BR_WHEEL], motor_output[BL_WHEEL]);

		chThdSleepMilliseconds(2);
	}
}

/*
 * Application entry point.
 */
int main(void)
{

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  RC_init();
  can_processInit();

  rc = RC_get();

  uint8_t i;
  for (i = 0; i < 4; i++) {
      lpfilter_init(&speed_lpf[i], 500, 50);
  }

  chThdCreateStatic(motor_ctrl_thread_wa, sizeof(motor_ctrl_thread_wa),
		  	  	  	 NORMALPRIO, motor_ctrl_thread, NULL);

  /*
   * Normal main() thread activity
   */
  while (true)
  {
    chThdSleepMilliseconds(500);
  }
}
