/*
 * myRMT.c
 *
 *  Created on: Jun 2, 2026
 *      Author: ReNuX
 */

#include <stdint.h>
#include <sys/_intsup.h>

#define EV1527_T_US      350 // Base timing period (example: 315us)
#define EV1527_T_TOLERANCE_US (EV1527_T_US / 3) // Tolerance (~30%)
//#define EV1527_SHORT_PULSE_US (EV1527_T_US) // ~210us   //TODO corre

#define	expected_high_min	((EV1527_T_US-EV1527_T_TOLERANCE_US)*3)
#define	expected_high_max	((EV1527_T_US+EV1527_T_TOLERANCE_US)*3)
#define	expected_low_min	(EV1527_T_US-EV1527_T_TOLERANCE_US)
#define	expected_low_max	(EV1527_T_US+EV1527_T_TOLERANCE_US)

_Bool is_Low(short num){
	return (num >= expected_low_min) && (num <= expected_low_max);
}

_Bool is_High(short num){
	return (num >= expected_high_min) && (num <= expected_high_max);
}
