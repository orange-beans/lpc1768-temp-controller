#ifndef __MAIN_H
#define __MAIN_H

//****** Define Constants ******//
#define REALTIME_TICK_S     0x01
#define COMMAND_S           0x02

#define LARGE_STACK_SIZE    4096
#define MEDIUM_STACK_SIZE   2048
#define SMALL_STACK_SIZE    1024
#define MINI_SMALL_SIZE     512

// This interval needs to be larger than MAX31855 poll interval
#define REALTIME_INTERVAL   200

#endif /* __MAIN_H */
