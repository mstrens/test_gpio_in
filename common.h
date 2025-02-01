/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef COMMON_COMMON_H_
#define COMMON_COMMON_H_

#include <stdint.h>
#include "cybsp.h"
#include "cy_utils.h"

#include "SEGGER_RTT.h"

// riding modes
#define OFF_MODE                                  0 // not used
#define POWER_ASSIST_MODE                         1
#define TORQUE_ASSIST_MODE                        2
#define CADENCE_ASSIST_MODE                       3
#define eMTB_ASSIST_MODE                          4
#define HYBRID_ASSIST_MODE						  5
#define CRUISE_MODE                               6
#define WALK_ASSIST_MODE                          7
#define TORQUE_SENSOR_CALIBRATION_MODE            8								   

// error codes
/*
#define ERROR_OVERVOLTAGE							1 // E01 (E06 blinking for XH18)
#define ERROR_TORQUE_SENSOR                       	2 // E02
#define ERROR_CADENCE_SENSOR			          	3 // E03
#define ERROR_MOTOR_BLOCKED                       	4 // E04
#define ERROR_THROTTLE								5 // E05 (E03 blinking for XH18)
#define ERROR_OVERTEMPERATURE						6 // E06
#define ERROR_BATTERY_OVERCURRENT                 	7 // E07 (E04 blinking for XH18)
#define ERROR_SPEED_SENSOR							8 // E08
#define ERROR_WRITE_EEPROM  					  	9 // E09 shared (E08 blinking for XH18)
#define ERROR_MOTOR_CHECK                       	9 // E09 shared (E08 blinking for XH18)
// note: controller works with no less than 15 V so give error code if voltage is too low
*/										   

// walk assist
//#define WALK_ASSIST_THRESHOLD_SPEED_X10           80  // 80 -> 8.0 kph, this is the maximum speed limit from which walk assist can be activated

// cruise
//#define CRUISE_THRESHOLD_SPEED_X10                90  // 90 -> 9.0 kph, this is the minimum speed limit from which cruise can be activated

// optional ADC function
#define NOT_IN_USE                                0
#define TEMPERATURE_CONTROL                       1
#define THROTTLE_CONTROL                          2

// cadence sensor
//#define STANDARD_MODE                             0
//#define ADVANCED_MODE                             1
//#define CALIBRATION_MODE                          2

extern volatile uint32_t system_ticks ;

#define AUTOMATIC_ROTATION (0) // 1 means we are just testing using a very slow rotation flux discarding hall values
                                // 0 is the code for production (detect hall changes and adapat flux for rotation)
// when AUTOMATIC_ROTATION is 1, we have to avoid calling ebike_app_init() and ebike_app_controler() in main.c
// we also have to check the debug messages being printed in main




uint16_t map_ui16(uint16_t in, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
uint8_t map_ui8(uint8_t in, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);
uint8_t ui8_max(uint8_t value_a, uint8_t value_b);
uint8_t ui8_min(uint8_t value_a, uint8_t value_b);
uint16_t filter(uint16_t ui16_new_value, uint16_t ui16_old_value, uint8_t ui8_alpha);
void crc16(uint8_t ui8_data, uint16_t *ui16_crc);
void lights_set_state(uint8_t ui8_state) ; // moved by mstrens from another tsdz2 file

bool take_action(uint32_t index, uint32_t interval);
#endif /* COMMON_COMMON_H_ */
