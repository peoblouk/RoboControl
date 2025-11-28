#include "can_communication.h"

void can_init(void) {
    // Initialize CAN hardware
}

void can_receive_task(void *arg) {
    while (1) {
        // Receive CAN messages
        //vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void can_start(void) {
    can_init();
    xTaskCreatePinnedToCore(can_receive_task, "can_rx", 4096, NULL, 6, NULL, CORE_COMM);       // jádro pro komunikaci
}