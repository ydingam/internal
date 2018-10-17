#ifndef _DBUS_H_
#define _DBUS_H_

#include "ch.h"
#include "hal.h"


#include <string.h>
#include <stdlib.h>
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN              ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET           ((uint16_t)1024)
#define RC_CH_VALUE_MAX              ((uint16_t)1684)
#define DBUS_BUFFER_SIZE             ((uint8_t)18)

#define UART_DBUS                     &UARTD2

typedef enum{
	RC_S_DUMMY = 0,
	RC_S_UP = 1,
	RC_S_DOWN = 2,
	RC_S_MIDDLE = 3,
} rc_switch_t;

typedef enum{
	RC_UNCONNECTED = 0,
	RC_LOCKED,
	RC_UNLOCKING,
	RC_UNLOCKED
} rc_state_t;

typedef struct{
	int16_t channel0;
	int16_t channel1;
	int16_t channel2;
	int16_t channel3;
	uint8_t  s1;
	uint8_t  s2;
}RC_Ctl_t;

RC_Ctl_t* RC_get(void);
void RC_init(void);

#endif
