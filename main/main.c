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
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "hal/gpio_types.h"
#include "driver/gpio_filter.h"
#include "hal/glitch_filter_types.h"

#include "driver/gpio.h"
#include "portmacro.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/unistd.h>

#include "driver/gpio_filter.h"
#include "driver/rmt.h"
#include "driver/rmt_types_legacy.h"
#include "esp_private/glitch_filter_priv.h"
#include "soc/clk_tree_defs.h"
#include "soc/rmt_struct.h"

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
#define	expected_high_min	((EV1527_T_US-EV1527_T_TOLERANCE_US)*3)
#define	expected_high_max	((EV1527_T_US+EV1527_T_TOLERANCE_US)*3)
#define	expected_low_min	(EV1527_T_US-EV1527_T_TOLERANCE_US)
#define	expected_low_max	(EV1527_T_US+EV1527_T_TOLERANCE_US)

//#define EV1527_SYNC_LOW_US  (EV1527_T_US * 31) // Sync low duration (long pause) ~9450us
#define EV1527_SYNC_LOW_US  (EV1527_T_US * 25) // Sync low duration (long pause) ~9450us //temporaray to test
#define EV1527_FRAME_MAX_LEN_US (EV1527_T_US * 31 * 28) // Max frame length ~26ms

#define GPIO_INPUT_PIN_SEL  ((1ULL<<0) | (1ULL<<EXAMPLE_IR_RX_GPIO_NUM))
#define tBits 24


typedef struct {
    uint32_t high_us;
    uint32_t low_us;
} ev_bit_t;

const ev_bit_t ev_sym[2] = {
    {330, 1000}, // 0
    {1000, 330}  // 1
};


static const char *TAG = "renNEC";


rmt_symbol_word_t ev_get_symbol(bool bit)
{
    rmt_symbol_word_t s;
    s.level0 = 1;
    s.duration0 = ev_sym[bit].high_us;
    s.level1 = 0;
    s.duration1 = ev_sym[bit].low_us;
    return s;
}


/**
 * @brief Decode RMT symbols and print the result
 */
static void debug_routine(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
	if(symbol_num<24){printf("debug routine\n");return;}
    printf("frame start---%d \r\n",symbol_num);
    for (size_t i = 0; i < symbol_num; i++) {
		if((rmt_nec_symbols[i].duration0>100) && (rmt_nec_symbols[i].duration1>100))
        printf("i:%d.{%d:%d},{%d:%d}\r\n",i, rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
               rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    printf("---frame end: \n");
}

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}



// --- EV1527 Decoder Function ---
// Decodes RMT symbols into an EV1527 32-bit value (24 address + 4 data)
static uint32_t decode_ev1527_signal(rmt_symbol_word_t *symbols, size_t num_symbols)
{
    if (num_symbols <= 24) {
        return 0; // No symbols received
    }

    uint32_t decoded_code = 0;
    uint8_t bit_count = 0;

    // Find the first non-sync low pulse
    uint8_t symbol_index = 0;
    while (symbol_index < num_symbols)
    {
		if (symbols[symbol_index].level0 == 0 && symbols[symbol_index].duration0 < EV1527_SYNC_LOW_US) {
        	symbol_index++;
        	}else if (symbols[symbol_index].level1 == 0 && symbols[symbol_index].duration1 < EV1527_SYNC_LOW_US) {
			symbol_index++;
			}else {
				
				
				
				
				symbol_index++; // Move past the sync low

    while ((symbol_index < num_symbols) && (bit_count < 24)) { // EV1527 is 28 bits total //TODO this passes ev code by 4 bits,make it go until hit the long 
    //pulse and also check 24 and 28 bits respectively according to dataSheet and implement these codes separately  
        int duration_us = symbols[symbol_index].duration0;
        bool level = symbols[symbol_index].level0;

//TODO should make it such to check both pulse sides, not just one
		//printf("bit%d ", bit_count);
        //if (level == 1) { // HIGH Pulse
            if (is_Low(duration_us)) {
                // This is a short HIGH pulse (start of '0')
                // Append a '0' bit
                decoded_code <<= 1;
                decoded_code |= !level;
                bit_count++;
            } else if (is_High(duration_us)) {
                // This is a long HIGH pulse (start of '1')
                // Append a '1' bit
                decoded_code <<= 1;
                decoded_code |= level;
                bit_count++;
            } else {
                ESP_LOGW(TAG, "Unexpected HIGH pulse duration: %d us. Tolerances: [%d-%d], [%d-%d] bit:%d start:%d",
                         duration_us, expected_high_min, expected_high_max, expected_low_min, expected_low_max,symbol_index,bit_count);
                //return 0; // Unexpected duration
                decoded_code=0;
                break;
            }
            printf("%d: %"PRIx32"\n",symbol_index,decoded_code);
        if (level == 1) { // HIGH Pulse
        ESP_LOGW(TAG, "high");
        } else { // LOW Pulse
        ESP_LOGW(TAG, "low should never come here!!!!!!!!!!!!!!!!!!!!");
/*        //TODO ommit this part by incloding "level" in decoded_code 1 2 lines up
             if (duration_us >= expected_low_min && duration_us <= expected_low_max) {
                // This is a short LOW pulse (end of '0')
                // Do nothing, bit was already added on the HIGH pulse
            } else if (duration_us >= expected_high_min && duration_us <= expected_high_max) {
                // This is a long LOW pulse (end of '1')
                // Do nothing, bit was already added on the HIGH pulse
            } else {
                ESP_LOGW(TAG, "Unexpected LOW pulse duration: %d us. Tolerances: [%d-%d], [%d-%d]",
                         duration_us, expected_high_min, expected_high_max, expected_low_min, expected_low_max);
                return 0; // Unexpected duration
            }*/
        }

        symbol_index++; // Move to the next symbol
    }
				
				
				
				
				    if (bit_count == 24) {
        // Final check: is the *last* symbol a long low (sync gap)?
        // We expect the next symbol to be a long low pause.
        // This is a simplified check; a more robust one would measure the gap duration.
        // For now, we assume 28 bits means a valid frame if we reached here.
        ESP_LOGI(TAG, "sexyyyy");
        return decoded_code;
    } 
			//break;
		}
		
	} 
    if (symbol_index >= num_symbols) {
        ESP_LOGW(TAG, "Could not find sync pulse or signal too short.%d",symbol_index);
        return 0; // Sync pulse not found or too short
    }

    // After sync low, we expect a HIGH pulse
    

    if (bit_count == 24) {
        // Final check: is the *last* symbol a long low (sync gap)?
        // We expect the next symbol to be a long low pause.
        // This is a simplified check; a more robust one would measure the gap duration.
        // For now, we assume 28 bits means a valid frame if we reached here.
        return decoded_code;
    } else {
        ESP_LOGW(TAG, "Incomplete frame. Expected 28 bits, got %d.", bit_count);
        return 0; // Not a full frame
    }
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
