/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

//#include "driver/rmt.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "soc/rmt_struct.h"

#include "hal/gpio_types.h"
//#include "driver/gpio_filter.h"

#include "driver/gpio.h"
#include "portmacro.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/unistd.h>


#include "myRMT.c"


#define EXAMPLE_IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       03
#define EXAMPLE_IR_RX_GPIO_NUM       47
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200     // Tolerance for parsing RMT symbols into bit stream
#define buf_size 160     //buffer size



// EV1527 Timing (in microseconds) - adjust these based on your receiver/remote
#define EV1527_T_US      350 // Base timing period (example: 315us)
#define EV1527_T_TOLERANCE_US (EV1527_T_US / 3) // Tolerance (~30%)
//#define EV1527_SHORT_PULSE_US (EV1527_T_US) // ~210us   //TODO correct this vals
//#define EV1527_LONG_PULSE_US  (EV1527_T_US*3) // ~420us

#define GPIO_INPUT_PIN_SEL  ((1ULL<<0) | (1ULL<<EXAMPLE_IR_RX_GPIO_NUM))
#define tBits 24

static const char *TAG = "renMain";

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}



void app_main(void)
{
    ESP_LOGI(TAG, "create RMT RX channel"); //create RMT RX channel
    rmt_rx_channel_config_t rx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = buf_size, // amount of RMT symbols that the channel can store at a time
        .gpio_num = EXAMPLE_IR_RX_GPIO_NUM,
    };
    rmt_channel_handle_t rx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel)); //reg new channel
    ESP_LOGI(TAG, "register RX done callback");
    
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    // the following timing requirement is based on NEC protocol //TODO: make it rmt base
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 3100,     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
        .signal_range_max_ns = 12000000, // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
        .flags.en_partial_rx=1,
    };


    ESP_LOGI(TAG, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = EXAMPLE_IR_TX_GPIO_NUM,
        .flags.invert_out=0,
        .flags.io_loop_back=1,
    };
    
    //TODO<: why its here? and check if it has any usage
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;	//interrupt of rising edge
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;//|(1ULL <<0 ) ;    //bit mask of the pins, use GPIO4/5 here
    io_conf.mode = GPIO_MODE_INPUT;//set as input mode
    io_conf.pull_up_en = 0;    //enable pull-up mode
    io_conf.pull_down_en=0;    //enable pull-up mode
    gpio_config(&io_conf);
    //TODO>
    
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    ESP_LOGI(TAG, "enable RMT TX and RX channels");
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols[buf_size]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data;
    // ready to receive
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    printf("1stsymb:%d\n",raw_symbols[0].duration0);
    
    while (1) {
        // wait for RX done signal
                    int bef=uxQueueMessagesWaiting(receive_queue);
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(20)) == pdPASS) {
            // parse the receive symbols and print the result
            //debug_routine(rx_data.received_symbols, rx_data.num_symbols);
            uint32_t sag=3;

            sag=decode_ev1527_signal(rx_data.received_symbols,rx_data.num_symbols);
                        int aft=uxQueueMessagesWaiting(receive_queue);
            
            
            // start receive again
            int t=rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config);

            printf("sag:%"PRIX32"\nbef:%d aft:%d t:%d num:%d \n",sag,bef,aft,t,rx_data.num_symbols);
        } else {


			
            rmt_symbol_word_t frame[tBits + 1]; // 1 sync at the end
            uint32_t code = 0x2E1412;
            
            for (int i = 0; i < tBits; i++) {
                bool bit = (code >> (tBits -1 - i)) & 0x1;
                frame[i] = ev_get_symbol(bit);
            }
            // Sync pulse (gap)
            frame[tBits].level0 = 1;
            frame[tBits].duration0 = 350; // sync short high
            frame[tBits].level1 = 0;
            frame[tBits].duration1 = 10000; // long low gap

            rmt_transmit_config_t tx_config = {
                .loop_count = 10 // send whole frame 10 times
            };

            rmt_encoder_handle_t copy_encoder = NULL;
            rmt_copy_encoder_config_t copy_encoder_cfg = {};
            ESP_ERROR_CHECK(
                rmt_new_copy_encoder(&copy_encoder_cfg, &copy_encoder));


            //ESP_ERROR_CHECK(rmt_enable(tx_channel));
            ESP_ERROR_CHECK(
                rmt_transmit(tx_channel, copy_encoder,frame, sizeof(frame), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_channel, 1000));
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    }
}
