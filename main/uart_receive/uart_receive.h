// ===============================
// UART COMMAND INTERFACE TASK
#include "driver/uart.h"
#include "esp_log.h"
#include "robot_io.h"
#include <string.h>

#define UART_PORT_NUM      UART_NUM_1
#define UART_TX_PIN        GPIO_NUM_10   // libovolný pin s funkcí TX
#define UART_RX_PIN        GPIO_NUM_9   // libovolný pin s funkcí RX
#define UART_BAUD_RATE     115200
#define BUF_SIZE           256
