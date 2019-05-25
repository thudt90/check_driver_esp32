/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

// static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive
and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
// static QueueHandle_t uart0_queue;

// Both definition are same and valid
//static uart_isr_handle_t *handle_console;
static intr_handle_t handle_console;

// Receive buffer to collect incoming data
uint8_t rxbuf[256];
// Register to collect data length
uint16_t urxlen;


static void echo_task()
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    //uint8_t c = 'a';

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        // uart_write_bytes(EX_UART_NUM, (const char *) &c, len);
        uart_write_bytes(EX_UART_NUM, (const char *) data, len);
        // Write data to UART, end with a break signal.
        //uart_write_bytes_with_break(EX_UART_NUM, "test break\n",strlen("test break\n"), 100);
    }
}

/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
static void IRAM_ATTR uart_intr_handle(void *arg)
{
  uint16_t rx_fifo_len, status;
  uint16_t i = 0;

  status = UART0.int_st.val; // read UART interrupt Status
  rx_fifo_len = UART0.status.rxfifo_cnt; // read number of bytes in UART buffer

  while(rx_fifo_len){
   rxbuf[i++] = UART0.fifo.rw_byte; // read all bytes
   rx_fifo_len--;
 }

 // after reading bytes from buffer clear UART interrupt status
 uart_clear_intr_status(EX_UART_NUM, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

// a test code or debug code to indicate UART receives successfully,
// you can redirect received byte as echo also
 uart_write_bytes(EX_UART_NUM, (const char*) "RX Done\n", strlen("RX Done\n"));
 uart_write_bytes(EX_UART_NUM, (const char*) rxbuf, i);

}

/* setting interupt rx uart */
void uart_setting(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    // esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // release the pre registered UART handler/subroutine
    uart_isr_free(EX_UART_NUM);

    // register new UART subroutine
    uart_isr_register(EX_UART_NUM,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console);

    // enable RX interrupt
    uart_enable_rx_intr(EX_UART_NUM);

    // uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    //Set uart pattern detect function.
    // uart_enable_pattern_det_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 10000, 10, 10);
    //Reset the pattern queue length to record at most 20 pattern positions.
    // uart_pattern_queue_reset(EX_UART_NUM, 20);
}


void app_main()
{
    // esp_log_level_set(TAG, ESP_LOG_INFO);

    uart_setting();
     // Configure a temporary buffer for the incoming data
    // uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    //Create a task to handler UART event from ISR
    // xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    //xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);

    while(1)
    {
        vTaskDelay(1000);
    }
}
