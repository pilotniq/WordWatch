/*
  ble.h
	*/

#ifndef BLE_H
#define BLE_H

#define APP_TIMER_PRESCALER             0                                                    /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                                                    /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                    /**< Size of timer operation queues. */

void erl_ble_init( const char *deviceName );

#endif
