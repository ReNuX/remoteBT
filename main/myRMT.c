/*
 * myRMT.c
 *
 *  Created on: Jun 2, 2026
 *      Author: ReNuX
 */

#include <stdint.h>
#include <sys/_intsup.h>
#include "esp_log.h"
#include "hal/rmt_types.h"
#include "stdio.h"

// EV1527 Timing (in microseconds) - adjust these based on your receiver/remote
#define EV1527_T_US      350 // Base timing period (example: 315us)
#define EV1527_T_TOLERANCE_US (EV1527_T_US / 3) // Tolerance (~30%)
//#define EV1527_SHORT_PULSE_US (EV1527_T_US) // ~210us   //TODO corre

#define	expected_high_min	((EV1527_T_US-EV1527_T_TOLERANCE_US)*3)
#define	expected_high_max	((EV1527_T_US+EV1527_T_TOLERANCE_US)*3)
#define	expected_low_min	(EV1527_T_US-EV1527_T_TOLERANCE_US)
#define	expected_low_max	(EV1527_T_US+EV1527_T_TOLERANCE_US)

#define EV1527_SYNC_LOW_US  (EV1527_T_US * 25) // Sync low duration (long pause) ~9450us //temporaray to test
#define EV1527_FRAME_MAX_LEN_US (EV1527_T_US * 31 * 28) // Max frame length ~26ms

static const char *TAG = "renRTM";


typedef struct {
    uint32_t high_us;
    uint32_t low_us;
} ev_bit_t;

const ev_bit_t ev_sym[2] = {
    {330, 1000}, // 0
    {1000, 330}  // 1
};


rmt_symbol_word_t ev_get_symbol(_Bool bit)
{
    rmt_symbol_word_t s;
    s.level0 = 1;
    s.duration0 = ev_sym[bit].high_us;
    s.level1 = 0;
    s.duration1 = ev_sym[bit].low_us;
    return s;
}




_Bool is_Low(short num){
	return (num >= expected_low_min) && (num <= expected_low_max);
}

_Bool is_High(short num){
	return (num >= expected_high_min) && (num <= expected_high_max);
}



// --- EV1527 Decoder Function ---
// Decodes RMT symbols into an EV1527 32-bit value (24 address + 4 data)
static uint32_t decode_ev1527_signal(rmt_symbol_word_t *symbols, uint8_t num_symbols)
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
        _Bool level = symbols[symbol_index].level0;

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
				//Unexpected HIGH pulse duration: %d us. Tolerances: [%d-%d], [%d-%d] bit:%d start:%d
                ESP_LOGW(TAG, "Unexpected H d: %d us. Tol: [%d-%d], [%d-%d] bit:%d start:%d",
                         duration_us, expected_high_min, expected_high_max, expected_low_min, expected_low_max,symbol_index,bit_count);
                //return 0; // Unexpected duration
                decoded_code=0;
                break;
            }
            printf("%d: %"PRIx32"\n",symbol_index,decoded_code);
        if (level == 1) { // HIGH Pulse
        //ESP_LOGW(TAG, "high");
        } else { // LOW Pulse
        //ESP_LOGW(TAG, "low should never come here!!!!!!!!!!!!!!!!!!!!");
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
