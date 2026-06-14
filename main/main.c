/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "driver/rmt.h"
#include "driver/rmt_types_legacy.h"
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
#include "myBLE.h"


static const char *mainTAG = "renMain";
    rmt_channel_handle_t rx_channel = NULL;
    rmt_channel_handle_t tx_channel = NULL; 

static bool my_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    //edata->flags.is_last==true
    return high_task_wakeup == pdTRUE;
}

void ble_callback_handler(uint8_t *data,void *userdata){
	int *my_val=(int *)userdata;
	uint32_t tmp=*(uint32_t*)data;
	ESP_LOGI(mainTAG,"called back @main data:%"PRIX32" userdata%d",tmp,*my_val);
	rmt_send(tmp,tx_channel);
	
}

void app_main(void)
{
  

    // should tinker with partial flags later
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 3100,     // this flag seems to be in ns , the duration for signal is in us,  ns < us
        .signal_range_max_ns = 32000000, // 12000 us
        .flags.en_partial_rx=1,
    };
    
    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols[buf_size]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data;
    
    QueueHandle_t receive_queue = xQueueCreate(2, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = my_rmt_rx_done_callback,
    };
    
   
    setup_rmt(&rx_channel,&tx_channel);
    setup_BLE();
    int t=100;
    reg_ble_callback(ble_callback_handler,&t);
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
     
    while (1) {
        // wait for RX done signal
        //int bef=uxQueueMessagesWaiting(receive_queue);
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(20)) == pdPASS) {
            // parse the receive symbols and print the result
            //debug_routine(rx_data.received_symbols, rx_data.num_symbols);
            
            uint32_t res=decode_ev1527_signal(rx_data.received_symbols,rx_data.num_symbols);
            //int aft=uxQueueMessagesWaiting(receive_queue);


			if(res>0){
				ESP_LOGI(TAG, "res:%"PRIX32"\n" ,res);
				uint8_t *dat=(uint8_t*)&res;
				//if(dat[0]=='9')
				//{ESP_LOGI(TAG, "huhu");}
				//ESP_LOGI(TAG, "hehe %d,%d,%c,%hhu",dat[0],dat[1],(char) dat[0],dat[1]);
				
				/*for (int ee=0; ee<10; ee++) {
					ESP_LOGI(TAG, "%d.%d=%c,",ee,dat[ee],(char) dat[ee]);
					BYTE tmp=(res >> (bitLen -1 - (ee*3))) & 0xf;
					BYTE tmp2=(res >> (bitLen -1 - ee)) & 0x1;
					ESP_LOGI(TAG, "->%d=%d,",tmp, tmp2);
				
					}*/

				//debug_routine(rx_data.received_symbols, rx_data.num_symbols);
            
				ble_send(dat,5);
			}
			
			if(rx_data.flags.is_last){
				(void) rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config);
			}
            //printf(" bef:%d aft:%d num:%d \n",bef,aft,rx_data.num_symbols);
        } else {

			//rmt_send(0x2E1412,tx_channel);
        }
    }
}
