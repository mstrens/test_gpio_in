/*
 * config.h
 *
 *  Automatically created by TSDS2 Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
/* Duplicated for easier manual update
#define OFF_MODE                                  0
#define POWER_ASSIST_MODE                         1
#define TORQUE_ASSIST_MODE                        2
#define CADENCE_ASSIST_MODE                       3
#define eMTB_ASSIST_MODE                          4
#define HYBRID_ASSIST_MODE						  5
#define CRUISE_MODE                               6
#define WALK_ASSIST_MODE                          7
#define TORQUE_SENSOR_CALIBRATION_MODE            8								   
*/
/*
Index of the data too display
ui16_battery_SOC_percentage_x10          1
ui16_battery_voltage_calibrated_x10      2
ui8_battery_current_filtered_x10         3
ui16_battery_power_filtered_x10          4
ui16_adc_throttle                        5
ui16_adc_torque                          6
ui8_pedal_cadence_RPM                    7
ui16_human_power_filtered_x10            8
ui16_adc_pedal_torque_delta              9
ui32_wh_x10                              10
ui16_motor_speed_erps                    11
ui16_duty_cycle_percent                  12
*/



#define MOTOR_TYPE 0              // 0 = 48V, 1 = 36V ; is used to change the FOC_ANGLE_MULTIPLIER in main.h
#define TORQUE_SENSOR_CALIBRATED 0
#define MOTOR_ACCELERATION  35
#define MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION 0 // when enabled (1), assistance is provided when the pedal is pressed more than the thershold herafer
#define ASSISTANCE_WITHOUT_PEDAL_ROTATION_THRESHOLD 20
#define PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100 67    // used to calculate the correct ratio between the assistance factor and the human power + for total power
#define PEDAL_TORQUE_ADC_MAX 300                    // value from ADC when max weigth is apply on one pedal
#define STARTUP_BOOST_TORQUE_FACTOR 300
#define MOTOR_BLOCKED_COUNTER_THRESHOLD 2
#define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10 5  // mstrens it was 30 for tsdz2, reduce it for testing
#define MOTOR_BLOCKED_ERPS_THRESHOLD 10       // electric RPS mstrens it was 20 for tsdz2, TSDZ8 has 4 poles instead of 8, so 2 more ticks for the same speed

#define STARTUP_BOOST_CADENCE_STEP 20       
#define BATTERY_CURRENT_MAX        5          // A mstrens: it was 13 for tsdz2, reduce for testing
#define TARGET_MAX_BATTERY_POWER 500
#define TARGET_MAX_BATTERY_CAPACITY 500
#define BATTERY_CELLS_NUMBER 10               // mstrens 10 for 36V battery and 13 for 48V battery
#define MOTOR_DECELERATION 35
#define BATTERY_LOW_VOLTAGE_CUT_OFF 30          // Volt mstrens : it was 39, changed to 10V for testing// could use 30 for 36 batt and 39 for 48V battery
#define ACTUAL_BATTERY_VOLTAGE_PERCENT 100
#define ACTUAL_BATTERY_CAPACITY_PERCENT 90
#define LI_ION_CELL_OVERVOLT 4.35
#define LI_ION_CELL_RESET_SOC_PERCENT 4.10
#define LI_ION_CELL_VOLTS_FULL 4.10
#define LI_ION_CELL_VOLTS_3_OF_4 3.85
#define LI_ION_CELL_VOLTS_2_OF_4 3.60
#define LI_ION_CELL_VOLTS_1_OF_4 3.35
#define LI_ION_CELL_VOLTS_5_OF_6 3.94
#define LI_ION_CELL_VOLTS_4_OF_6 3.76
#define LI_ION_CELL_VOLTS_3_OF_6 3.60
#define LI_ION_CELL_VOLTS_2_OF_6 3.44
#define LI_ION_CELL_VOLTS_1_OF_6 3.26
#define LI_ION_CELL_VOLTS_EMPTY 3.10
#define WHEEL_PERIMETER 2200
#define WHEEL_MAX_SPEED 25         // km/h
#define ENABLE_LIGHTS 1
#define ENABLE_WALK_ASSIST 1       // 1 enable
#define ENABLE_BRAKE_SENSOR 1
#define ENABLE_THROTTLE 1          // 1 enable
#define ENABLE_TEMPERATURE_LIMIT 0
#define ENABLE_STREET_MODE_ON_STARTUP 1
#define ENABLE_SET_PARAMETER_ON_STARTUP 0
#define ENABLE_ODOMETER_COMPENSATION 0
#define STARTUP_BOOST_ON_STARTUP 1
#define TORQUE_SENSOR_ADV_ON_STARTUP 0
#define LIGHTS_CONFIGURATION_ON_STARTUP 0
#define RIDING_MODE_ON_STARTUP 3     // CADENCE_ASSIST_MODE= 3
#define LIGHTS_CONFIGURATION_1 1
#define LIGHTS_CONFIGURATION_2 9
#define LIGHTS_CONFIGURATION_3 10
#define STREET_MODE_POWER_LIMIT_ENABLED 1
#define STREET_MODE_POWER_LIMIT 500
#define STREET_MODE_SPEED_LIMIT 25
#define STREET_MODE_THROTTLE_ENABLED 0
#define STREET_MODE_CRUISE_ENABLED 0
#define ADC_THROTTLE_MIN_VALUE 47             // It is ADC 8 bits ; For tsdz2, it was 47, for tsdz8 it could be 45; we keep 47; this is mapped to 0
#define ADC_THROTTLE_MAX_VALUE 176            // It is ADC 8 bits ; For TSDZ2, it was 176, for tsdz8 it could be 180; we keep 176 ; this is mapped to 255
#define MOTOR_TEMPERATURE_MIN_VALUE_LIMIT 65
#define MOTOR_TEMPERATURE_MAX_VALUE_LIMIT 95
#define ENABLE_TEMPERATURE_ERROR_MIN_LIMIT 0
#define ENABLE_VLCD6 0
#define ENABLE_VLCD5 1
#define ENABLE_XH18 0
#define ENABLE_DISPLAY_WORKING_FLAG 1
#define ENABLE_DISPLAY_ALWAYS_ON 0
#define ENABLE_WHEEL_MAX_SPEED_FROM_DISPLAY 0 // allow to change the max speed from the display
#define DELAY_MENU_ON 50
#define COASTER_BRAKE_ENABLED 0
#define COASTER_BRAKE_TORQUE_THRESHOLD 30
#define ENABLE_AUTO_DATA_DISPLAY 1
#define STARTUP_ASSIST_ENABLED 0

#define AUTO_DATA_NUMBER_DISPLAY 6         // number of data to display in sequence (max 6)
#define DELAY_DISPLAY_DATA_1 50
#define DELAY_DISPLAY_DATA_2 50
#define DELAY_DISPLAY_DATA_3 50
#define DELAY_DISPLAY_DATA_4 50
#define DELAY_DISPLAY_DATA_5 50
#define DELAY_DISPLAY_DATA_6 50
#define DISPLAY_DATA_1 2             // ui16_battery_SOC_percentage_x10
#define DISPLAY_DATA_2 3             // ui16_battery_voltage_calibrated_x10
#define DISPLAY_DATA_3 2             // ui32_wh_x10 
#define DISPLAY_DATA_4 3              // ui8_pedal_cadence_RPM
#define DISPLAY_DATA_5 2              // ui16_battery_power_filtered_x10
#define DISPLAY_DATA_6 3              // ui16_human_power_filtered_x10
/*
Index of the data too display
ui16_battery_SOC_percentage_x10          1
ui16_battery_voltage_calibrated_x10      2
ui8_battery_current_filtered_x10         3
ui16_battery_power_filtered_x10          4
ui16_adc_throttle                        5
ui16_adc_torque                          6
ui8_pedal_cadence_RPM                    7
ui16_human_power_filtered_x10            8
ui16_adc_pedal_torque_delta              9
ui32_wh_x10                              10
ui16_motor_speed_erps                    11
ui16_duty_cycle_percent                  12
*/



#define POWER_ASSIST_LEVEL_1 50
#define POWER_ASSIST_LEVEL_2 100
#define POWER_ASSIST_LEVEL_3 160
#define POWER_ASSIST_LEVEL_4 260
#define TORQUE_ASSIST_LEVEL_1 50
#define TORQUE_ASSIST_LEVEL_2 80
#define TORQUE_ASSIST_LEVEL_3 120
#define TORQUE_ASSIST_LEVEL_4 160
#define CADENCE_ASSIST_LEVEL_1 80
#define CADENCE_ASSIST_LEVEL_2 100
#define CADENCE_ASSIST_LEVEL_3 130
#define CADENCE_ASSIST_LEVEL_4 160
#define EMTB_ASSIST_LEVEL_1 60
#define EMTB_ASSIST_LEVEL_2 100
#define EMTB_ASSIST_LEVEL_3 140
#define EMTB_ASSIST_LEVEL_4 180
#define WALK_ASSIST_LEVEL_1 30
#define WALK_ASSIST_LEVEL_2 35
#define WALK_ASSIST_LEVEL_3 40
#define WALK_ASSIST_LEVEL_4 45
#define WALK_ASSIST_THRESHOLD_SPEED_X10 60
#define WALK_ASSIST_DEBOUNCE_ENABLED 0
#define WALK_ASSIST_DEBOUNCE_TIME 60
#define CRUISE_TARGET_SPEED_LEVEL_1 15
#define CRUISE_TARGET_SPEED_LEVEL_2 18
#define CRUISE_TARGET_SPEED_LEVEL_3 21
#define CRUISE_TARGET_SPEED_LEVEL_4 24
#define CRUISE_MODE_WALK_ENABLED 0
#define CRUISE_THRESHOLD_SPEED 10
#define PEDAL_TORQUE_ADC_OFFSET 150        // to be tested; it is the ADC value when no pressured is applied on the pedal
#define UNITS_TYPE 0
#define ASSIST_THROTTLE_MIN_VALUE 0        // adc values are mapped between min and max values
#define ASSIST_THROTTLE_MAX_VALUE 255
#define STREET_MODE_WALK_ENABLED 1
#define DATA_DISPLAY_ON_STARTUP 1
#define FIELD_WEAKENING_ENABLED 0
#define PEDAL_TORQUE_ADC_OFFSET_ADJ 20
#define PEDAL_TORQUE_ADC_RANGE_ADJ 20
#define PEDAL_TORQUE_ADC_ANGLE_ADJ 36
#define PEDAL_TORQUE_PER_10_BIT_ADC_STEP_ADV_X100 34  // to be tested
#define SOC_PERCENT_CALC 0
#define STARTUP_BOOST_AT_ZERO 0
#define ENABLEC850 0
#define STREET_MODE_THROTTLE_LEGAL 0
#define BRAKE_TEMPERATURE_SWITCH 0
#define eMTB_BASED_ON_POWER 1
#define SMOOTH_START_ENABLED 1
#define SMOOTH_START_SET_PERCENT 35
#define TEMPERATURE_SENSOR_TYPE 0
#define CRUISE_MODE_ENABLED 1
#define THROTTLE_MODE 4               // see below: when 0, throttle ADC is not used and converted; perhaps it is filled by java based on other parameters
#define STREET_MODE_THROTTLE_MODE 4   // see below: with the display, it is probably possible to switch from THOTTLE_MODE to STREET_MODE_TROTTLE_MODE
#define ASSIST_LEVEL_1_OF_5_PERCENT 60
#define ALTERNATIVE_MILES 0

#endif /* CONFIG_H_ */
// THROTTLE_MODE and STREEMODE_THROTTLE_MODE can be Disabled (=0), pedaling (1), 6KM/h Only (2) , 6KM/h & pedaling (3), Unconditionnal (4)
// Only with brake sensors enabled<br>\nSet Optional ADC to Throttle\n<html>");
// for street mode, Trottle mode must be at the same or higher level