#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "dbus.h"

static int16_t motor_final_output;

const float kp_angle = 1.0f;     //Proportional for angle
const float ki_angle = 0.1f;     //Integration for angle
const float kd_angle = 0.1f;     //Derivative for angle

const float kp_speed = 1.0f;
const float ki_speed = 1.0f;
const float kd_speed = 1.0f;

//all to be modified

volatile float errorSum_angle = 0;
volatile float preError_angle = 0;

volatile float errorSum_speed = 0;
volatile float preError_speed = 0;

static float angle_pid_control(const float setPoint, const float currentPoint)
{
    float output;
    float error = setPoint - currentPoint;
    errorSum_angle += error;
    float errorDiff = error - preError_angle;
    preError_angle = error;

    if(errorSum_angle > 10){
          errorSum_angle = 10;
    }else if(errorSum_angle < -10){
          errorSum_angle = -10;
    }
    //limit Sum
    //to be changed: the range

    int16_t pidp = error*kp_angle;
    int16_t pidi = ki_angle*errorSum_angle;
    int16_t pidd = kd_angle*errorDiff;

    output = pidp + pidi + pidd;

    if (currentPoint - setPoint <= 10 || setPoint - currentPoint >= -10){
      output = 0;
    }
    //to be changed: the range

    if(output > 10000)
        output = 10000;
    else if(output < -10000)
        output = -10000;
    //to be changed: the range

    return output;
    //return a float so that it can be received by speed_pid_control()
}

static int16_t speed_pid_control(const float setPoint_fromAngle, const float currentPoint_fromMotor)
{
    int16_t output;
    float error = setPoint_fromAngle - currentPoint_fromMotor;
    errorSum_speed += error;
    float errorDiff = error - preError_speed;
    preError_speed = error;

    if(errorSum_speed > 10){
          errorSum_speed = 10;
    }else if(errorSum_speed < -10){
          errorSum_speed = -10;
    }

    int16_t pidp = error*kp_speed;
    int16_t pidi = ki_speed*errorSum_speed;
    int16_t pidd = kd_speed*errorDiff;

    output = (int)(pidp + pidi + pidd);
    //speed_pid_control should out put a current value to be passed to motor_set_current()

    if(output > 10000)
        output = 10000;
    else if(output < -10000)
        output = -10000;
    // all the ranges above are to be changed
    return output;
}

/**
 * a simple function to link two PID controller
 *
 * setPoint_forAngle: from open CV
 * currentPoint_fromMotor: from Motor feedback
 * currentSpeed_fromMotor: from Motor feedback
 */
static int16_t pid_control_all(const float setPoint_forAngle,
                               const float currentPoint_fromMotor,
                               const float currentSpeed_fromMotor)
{
    int16_t output;
    int16_t setPoint_fromAngle = angle_pid_control(setPoint_forAngle, currentPoint_fromMotor);
    //setPoint_fromAngle: a speed setPoint to be passed to speed_pid_control()

    output = speed_pid_control(setPoint_fromAngle, currentSpeed_fromMotor);
    return output;
}

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
    (void) p;
    volatile Encoder_canStruct* encoder = can_getEncoder();
    volatile RC_Ctl_t* rc = RC_get();

	while(true)
	{
	  //todo(DONE): the first parameter of PIDcontrol can be changed by the switch of RC

	  float setPoint = 0.0f;

	  switch(rc->s1){
	  case RC_S_UP:
	    setPoint = 1.0f;
	    break;
	  case RC_S_MIDDLE:
	    setPoint = 2.0f;
	    break;
	  case RC_S_DOWN:
	    setPoint = 3.0f;
	    break;
	  default:
	    break;
	  }

	  motor_final_output = pid_control_all(setPoint,
	                                       ((encoder+0)->radian_angle),
	                                       ((encoder+0)->speed_rpm));

	  can_motorSetCurrent(0x200, motor_final_output,0,0,0);
	  chThdSleepMilliseconds(10);
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
