#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "driver/can.h"
#include "core_config.h"   // Core Configuration

void can_init(void);
//void can_send_message(can_message_t *msg);
void can_receive_task(void *arg);

#endif // CAN_COMMUNICATION_H