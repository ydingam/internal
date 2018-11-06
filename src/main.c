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
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-11
 */

#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "canBusProcess.h"
#include "chassis_task.h"
#include "configure.h"

#include "chassis_task.h"

#include <stdlib.h>


int16_t motor_output[4];        //Torque command for motors

pid_s_t wheel_pid[4];


static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
    (void) p;
  //	int16_t strafe = 0, drive = 0, rotation = 0;   //move direction for chassis
  //  Encoder_canStruct* encoder = can_getEncoder(); //Pointer to motor encoder feedbakc
  //  static float motor_error_int[4]; //error integrators for the four motors
    for(int i=0;i<4;i++)
    {
      pid_init(&wheel_pid[i],7.5f,0.03f,0.0f,1000.0f,12000.0f);
    }

	while(true)
	{
        /*
            //NOTE: A special question for you: how we decide this value
            //"12000/1320"
    		strafe = (rc->?? - 1024)*12000.0f/1320.0f;
            drive = (rc->?? - 1024)*12000.0f/1320.0f;
            rotation = (rc->?? - 1024)*12000.0f/1320.0f;
        */


    chassis_task(wheel_pid);

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

    //rc = RC_get();

    chThdCreateStatic(motor_ctrl_thread_wa, sizeof(motor_ctrl_thread_wa),
		  	  	  	 NORMALPRIO, motor_ctrl_thread, NULL);

    /*
    * Normal main() thread activity
    */
    while (true)
    {
        palTogglePad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
