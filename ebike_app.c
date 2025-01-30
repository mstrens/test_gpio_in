/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */
#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"
#include "ebike_app.h"
#include "main.h"
#include "motor.h"
#include "common.h"
#include "uart.h"



volatile struct_configuration_variables m_configuration_variables;

// display menu
uint8_t ui8_assist_level = ECO;
uint8_t ui8_assist_level_temp = ECO;
uint8_t ui8_assist_level_01_flag = 0;
 uint8_t ui8_riding_mode_temp = 0;
uint8_t ui8_lights_flag = 0;
 uint8_t ui8_lights_on_5s = 0;
 uint8_t ui8_menu_flag = 0;
 uint8_t ui8_menu_index = 0;
 uint8_t ui8_data_index = 0;
uint8_t ui8_menu_counter = 0;
 uint8_t ui8_display_function_code = 0;
 uint8_t ui8_display_function_code_temp = 0;
 uint8_t ui8_menu_function_enabled = 0;
 uint8_t ui8_display_data_enabled = 0;
 uint16_t ui16_display_data = 0;
 uint16_t ui16_data_value = 0;
 uint8_t ui8_auto_display_data_flag = 0;
 uint8_t ui8_auto_display_data_status = 0;
 uint8_t ui8_auto_data_number_display = AUTO_DATA_NUMBER_DISPLAY;
 uint16_t ui16_display_data_factor = 0;
uint8_t ui8_delay_display_function = DELAY_MENU_ON;
 uint8_t ui8_display_data_on_startup = DATA_DISPLAY_ON_STARTUP;
 uint8_t ui8_set_parameter_enabled_temp = ENABLE_SET_PARAMETER_ON_STARTUP;
 uint8_t ui8_auto_display_data_enabled_temp = ENABLE_AUTO_DATA_DISPLAY;
 uint8_t ui8_street_mode_enabled_temp = ENABLE_STREET_MODE_ON_STARTUP;
 uint8_t ui8_torque_sensor_adv_enabled_temp = TORQUE_SENSOR_ADV_ON_STARTUP;
 uint8_t ui8_assist_without_pedal_rotation_temp = MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION;
 uint8_t ui8_walk_assist_enabled_array[2] = {ENABLE_WALK_ASSIST,STREET_MODE_WALK_ENABLED};
 uint8_t ui8_display_battery_soc = 0;
 uint8_t ui8_display_riding_mode = 0;
 uint8_t ui8_display_lights_configuration = 0;
 uint8_t ui8_display_alternative_lights_configuration = 0;
 uint8_t ui8_display_torque_sensor_flag_1 = 0;
 uint8_t ui8_display_torque_sensor_flag_2 = 0;
 uint8_t ui8_display_torque_sensor_value = 0;
 uint8_t ui8_display_torque_sensor_step = 0;
 uint8_t ui8_display_function_status[3][5];
 uint8_t ui8_lights_configuration_2 = LIGHTS_CONFIGURATION_2;
 uint8_t ui8_lights_configuration_3 = LIGHTS_CONFIGURATION_3;
 uint8_t ui8_lights_configuration_temp = LIGHTS_CONFIGURATION_ON_STARTUP;

// system
 uint8_t ui8_riding_mode_parameter = 0;
volatile uint8_t ui8_system_state = NO_ERROR;
volatile uint8_t ui8_motor_enabled = 1; 
 uint8_t ui8_assist_without_pedal_rotation_threshold = ASSISTANCE_WITHOUT_PEDAL_ROTATION_THRESHOLD;
 uint8_t ui8_lights_state = 0;
uint8_t ui8_lights_button_flag = 0;
 uint8_t ui8_field_weakening_erps_delta = 0;
 uint8_t ui8_optional_ADC_function = OPTIONAL_ADC_FUNCTION;
 uint8_t ui8_walk_assist_level = 0;

// battery
 uint16_t ui16_battery_voltage_filtered_x10 = 0;
 uint16_t ui16_battery_voltage_calibrated_x10 = 0;
volatile uint16_t ui16_battery_voltage_soc_filtered_x10 = 0;
 uint16_t ui16_battery_power_x10 = 0;															  
 uint16_t ui16_battery_power_filtered_x10 = 0;
 uint16_t ui16_actual_battery_capacity = (uint16_t)(((uint32_t) TARGET_MAX_BATTERY_CAPACITY * ACTUAL_BATTERY_CAPACITY_PERCENT) / 100);
 uint32_t ui32_wh_x10 = 0;
 uint32_t ui32_wh_sum_x10 = 0;
volatile uint32_t ui32_wh_x10_offset = 0;
 uint32_t ui32_wh_since_power_on_x10 = 0;
volatile uint16_t ui16_battery_SOC_percentage_x10 = 0;
volatile uint8_t ui8_battery_SOC_init_flag = 0;
 uint8_t ui8_battery_state_of_charge = 0;

// power control
 uint8_t ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;        // 194
 uint8_t ui8_duty_cycle_ramp_up_inverse_step_default = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
 uint8_t ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;         //73
 uint8_t ui8_duty_cycle_ramp_down_inverse_step_default = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73
 uint16_t ui16_battery_voltage_filtered_x1000 = 0;
 uint16_t ui16_battery_no_load_voltage_filtered_x10 = 0;
 uint8_t ui8_battery_current_filtered_x10 = 0;
 uint8_t ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX; // In tdsz2 it was 112 = 18A; it is updated by the program based on riding mode parameters
 uint8_t ui8_adc_battery_current_target = 0;
 uint8_t ui8_duty_cycle_target = 0;
 uint16_t ui16_duty_cycle_percent = 0;
volatile uint8_t ui8_adc_motor_phase_current_max = ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX; // In tdsz2 it was 187
 uint8_t ui8_error_battery_overcurrent = 0;
 uint8_t ui8_adc_battery_overcurrent = (uint8_t)(ADC_10_BIT_BATTERY_CURRENT_MAX + ADC_10_BIT_BATTERY_EXTRACURRENT); //In tdsz2 it was 112 + 50
 uint8_t ui8_adc_battery_current_max_temp_1 = 0;
 uint8_t ui8_adc_battery_current_max_temp_2 = 0;
 uint32_t ui32_adc_battery_power_max_x1000_array[2];

// Motor ERPS
 uint16_t ui16_motor_speed_erps = 0;

// cadence sensor
volatile uint16_t ui16_cadence_ticks_count_min_speed_adj = CADENCE_SENSOR_CALC_COUNTER_MIN; //4270 
 uint8_t ui8_pedal_cadence_RPM = 0;
 uint8_t ui8_motor_deceleration = MOTOR_DECELERATION;

// torque sensor
 uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100 = PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100;  // 67
 uint8_t ui8_pedal_torque_per_10_bit_ADC_step_calc_x100 = PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100;  //
 uint16_t ui16_adc_pedal_torque_offset = PEDAL_TORQUE_ADC_OFFSET;      // 150
 uint16_t ui16_adc_pedal_torque_offset_init = PEDAL_TORQUE_ADC_OFFSET; // 150
 uint16_t ui16_adc_pedal_torque_offset_cal = PEDAL_TORQUE_ADC_OFFSET;  // 150
 uint16_t ui16_adc_pedal_torque_offset_min = PEDAL_TORQUE_ADC_OFFSET - ADC_TORQUE_SENSOR_OFFSET_THRESHOLD; //150-30
 uint16_t ui16_adc_pedal_torque_offset_max = PEDAL_TORQUE_ADC_OFFSET + ADC_TORQUE_SENSOR_OFFSET_THRESHOLD; // 150 + 30
 uint8_t ui8_adc_pedal_torque_offset_error = 0;
volatile uint16_t ui16_adc_coaster_brake_threshold = 0;
 uint16_t ui16_adc_pedal_torque = 0;
 uint16_t ui16_adc_pedal_torque_delta = 0;
 uint16_t ui16_adc_pedal_torque_delta_temp = 0;
 uint16_t ui16_adc_pedal_torque_delta_no_boost = 0;
 uint16_t ui16_pedal_torque_x100 = 0;
 uint16_t ui16_human_power_x10 = 0;
 uint16_t ui16_human_power_filtered_x10 = 0;
 uint8_t ui8_torque_sensor_calibrated = TORQUE_SENSOR_CALIBRATED;
 uint16_t ui16_pedal_weight_x100 = 0;
 uint16_t ui16_pedal_torque_step_temp = 0;
 uint8_t ui8_torque_sensor_calibration_flag = 0;
 uint8_t ui8_torque_sensor_calibration_started = 0;
 uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100_array[2];
 uint8_t ui8_eMTB_based_on_power = eMTB_BASED_ON_POWER;

// wheel speed sensor
 uint16_t ui16_wheel_speed_x10 = 0;
 uint8_t ui8_wheel_speed_max_array[2] = {WHEEL_MAX_SPEED,STREET_MODE_SPEED_LIMIT};

// wheel speed display
uint8_t ui8_display_ready_flag = 0;
uint8_t ui8_startup_counter = 0;
 uint16_t ui16_oem_wheel_speed_time = 0;
 uint8_t ui8_oem_wheel_diameter = 0;
 uint32_t ui32_odometer_compensation_mm = ZERO_ODOMETER_COMPENSATION;

// throttle control
 uint8_t ui8_adc_throttle_assist = 0;
 uint8_t ui8_throttle_adc_in = 0;
 uint8_t ui8_throttle_mode_array[2] = {THROTTLE_MODE,STREET_MODE_THROTTLE_MODE}; // this variable is never updated (so based only on config)

// cruise control
 uint8_t ui8_cruise_threshold_speed_x10_array[2] = {CRUISE_OFFROAD_THRESHOLD_SPEED_X10,CRUISE_STREET_THRESHOLD_SPEED_X10};
 uint8_t ui8_cruise_button_flag = 0;

// walk assist
 uint8_t ui8_walk_assist_flag = 0;
 uint8_t ui8_walk_assist_speed_target_x10 = 0;
 uint8_t ui8_walk_assist_duty_cycle_counter = 0;
 uint8_t ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MIN;
 uint8_t ui8_walk_assist_duty_cycle_max = WALK_ASSIST_DUTY_CYCLE_MIN;
 uint8_t ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN;
 uint16_t ui16_walk_assist_wheel_speed_counter = 0;
 uint16_t ui16_walk_assist_erps_target = 0;
 uint16_t ui16_walk_assist_erps_min = 0;
 uint16_t ui16_walk_assist_erps_max = 0;
 uint8_t ui8_walk_assist_speed_flag = 0;

// startup boost
 uint8_t ui8_startup_boost_at_zero = STARTUP_BOOST_AT_ZERO;
 uint8_t ui8_startup_boost_flag = 0;
 uint8_t ui8_startup_boost_enabled_temp = STARTUP_BOOST_ON_STARTUP;
 uint16_t ui16_startup_boost_factor_array[120];

// smooth start
 uint8_t ui8_smooth_start_flag = 0;
 uint8_t ui8_smooth_start_counter = 0;
 uint8_t ui8_smooth_start_counter_set = 0;

// startup assist
 uint8_t ui8_startup_assist_flag = 0;
 uint8_t ui8_startup_assist_adc_battery_current_target = 0;

// motor temperature control
 uint16_t ui16_adc_motor_temperature_filtered = 0;
 uint16_t ui16_motor_temperature_filtered_x10 = 0;
 uint8_t ui8_motor_temperature_max_value_to_limit_array[2] = {MOTOR_TEMPERATURE_MAX_VALUE_LIMIT, (uint8_t)(MOTOR_TEMPERATURE_MAX_VALUE_LIMIT + 50)};
 uint8_t ui8_motor_temperature_min_value_to_limit_array[2] = {MOTOR_TEMPERATURE_MIN_VALUE_LIMIT, (uint8_t)(MOTOR_TEMPERATURE_MIN_VALUE_LIMIT + 50)};

 uint8_t ui8_default_flash_state;


/* this part of definition has been commented for debugging because there is already a uart.c file with some definitions (used for first tests)
// UART
volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[UART_RX_BUFFER_LEN];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[UART_TX_BUFFER_LEN];
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;


*/

// array for oem display
 uint8_t ui8_data_index_array[DATA_INDEX_ARRAY_DIM] = {DISPLAY_DATA_1,DISPLAY_DATA_2,DISPLAY_DATA_3,DISPLAY_DATA_4,DISPLAY_DATA_5,DISPLAY_DATA_6};
 uint8_t ui8_delay_display_array[DATA_INDEX_ARRAY_DIM] = {DELAY_DISPLAY_DATA_1,DELAY_DISPLAY_DATA_2,DELAY_DISPLAY_DATA_3,DELAY_DISPLAY_DATA_4,DELAY_DISPLAY_DATA_5,DELAY_DISPLAY_DATA_6};

// array for riding parameters
 uint8_t  ui8_riding_mode_parameter_array[8][5] = {
	{POWER_ASSIST_LEVEL_OFF, POWER_ASSIST_LEVEL_ECO, POWER_ASSIST_LEVEL_TOUR, POWER_ASSIST_LEVEL_SPORT, POWER_ASSIST_LEVEL_TURBO},
	{TORQUE_ASSIST_LEVEL_0, TORQUE_ASSIST_LEVEL_1, TORQUE_ASSIST_LEVEL_2, TORQUE_ASSIST_LEVEL_3, TORQUE_ASSIST_LEVEL_4},
	{CADENCE_ASSIST_LEVEL_0, CADENCE_ASSIST_LEVEL_1, CADENCE_ASSIST_LEVEL_2, CADENCE_ASSIST_LEVEL_3, CADENCE_ASSIST_LEVEL_4},
	{EMTB_ASSIST_LEVEL_0, EMTB_ASSIST_LEVEL_1, EMTB_ASSIST_LEVEL_2, EMTB_ASSIST_LEVEL_3, EMTB_ASSIST_LEVEL_4},
	{POWER_ASSIST_LEVEL_OFF, POWER_ASSIST_LEVEL_ECO, POWER_ASSIST_LEVEL_TOUR, POWER_ASSIST_LEVEL_SPORT, POWER_ASSIST_LEVEL_TURBO},
	{CRUISE_TARGET_SPEED_LEVEL_0, CRUISE_TARGET_SPEED_LEVEL_1, CRUISE_TARGET_SPEED_LEVEL_2, CRUISE_TARGET_SPEED_LEVEL_3, CRUISE_TARGET_SPEED_LEVEL_4},
	{WALK_ASSIST_LEVEL_0, WALK_ASSIST_LEVEL_1, WALK_ASSIST_LEVEL_2, WALK_ASSIST_LEVEL_3, WALK_ASSIST_LEVEL_4},
	{0, 0, 0, 0, 0}
	};
		
// communications functions
//static void uart_receive_package(void);
//static void uart_send_package(void);

// system functions
static void get_battery_voltage(void);
static void get_pedal_torque(void);
static void calc_wheel_speed(void);
static void calc_cadence(void);

static void ebike_control_lights(void);
static void ebike_control_motor(void);
static void check_system(void);

static void set_motor_ramp(void);
static void apply_startup_boost(void);
static void apply_smooth_start(void);

static void apply_power_assist(void);
static void apply_torque_assist(void);
static void apply_cadence_assist(void);
static void apply_emtb_assist(void);
static void apply_hybrid_assist(void);
static void apply_cruise(void);
static void apply_walk_assist(void);
static void apply_throttle(void);
static void apply_temperature_limiting(void);
static void apply_speed_limit(void);

// functions for oem display
static void calc_oem_wheel_speed(void);
static void apply_torque_sensor_calibration(void);

// battery soc percentage x10 calculation
static void check_battery_soc(void);

uint16_t calc_battery_soc_x10(uint16_t ui16_battery_soc_offset_x10, uint16_t ui16_battery_soc_step_x10, uint16_t ui16_cell_volts_max_x100, uint16_t ui16_cell_volts_min_x100);


void ebike_app_init(void)
{
	// minimum value for these displays
	#if ENABLE_VLCD6 || ENABLE_850C
	if (ui8_delay_display_function < 70) {
		ui8_delay_display_function = 70;
	}
	#endif
	
	// set low voltage cutoff (16 bit) ; 39V => 390*100/87= 448adcfor 48V battery
	ui16_adc_voltage_cut_off = ((uint32_t) m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 * 100U) / BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
	
	// check if assist without pedal rotation threshold is valid (safety)
	if (ui8_assist_without_pedal_rotation_threshold > 100) {
		ui8_assist_without_pedal_rotation_threshold = 100;
	}
	// set duty cycle ramp up inverse step default
	ui8_duty_cycle_ramp_up_inverse_step_default = map_ui8((uint8_t) MOTOR_ACCELERATION, //35
				(uint8_t) 0,
				(uint8_t) 100,
				(uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT, //194
				(uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN); //24
	
	// set duty cycle ramp down inverse step default
	ui8_duty_cycle_ramp_down_inverse_step_default = map_ui8((uint8_t) MOTOR_DECELERATION, // 35
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,    // 73
                (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);       // 9
	
	// Smooth start counter set
	ui8_smooth_start_counter_set = map_ui8((uint8_t) SMOOTH_START_SET_PERCENT, //35
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) 255,
                (uint8_t) SMOOTH_START_RAMP_MIN);                             //30
	
	// set pedal torque per 10_bit DC_step x100 advanced (calibrated) or default(not calibrated)
	ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_DEFAULT] = PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100; // 67
	if (ui8_torque_sensor_calibrated) {
		ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_ADVANCED] = PEDAL_TORQUE_PER_10_BIT_ADC_STEP_ADV_X100; //34
	}
	else {
		ui8_pedal_torque_per_10_bit_ADC_step_x100_array[TORQUE_STEP_ADVANCED] = PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100;
	}
	
	// parameters status on startup
	// set parameters on startup                         Currently m_configuration_variables are taken from eeprom.c (to be changed later on)
	ui8_display_function_status[0][OFF] = m_configuration_variables.ui8_set_parameter_enabled;
	// auto display data on startup
	ui8_display_function_status[1][OFF] = m_configuration_variables.ui8_auto_display_data_enabled;
	// street mode on startup
	ui8_display_function_status[0][ECO] = m_configuration_variables.ui8_street_mode_enabled;
	// startup boost on startup
	ui8_display_function_status[1][ECO] = m_configuration_variables.ui8_startup_boost_enabled;
	// torque sensor adv on startup
	ui8_display_function_status[2][ECO] = m_configuration_variables.ui8_torque_sensor_adv_enabled;
	// assist without pedal rotation on startup
	ui8_display_function_status[1][TURBO] = m_configuration_variables.ui8_assist_without_pedal_rotation_enabled;
	// system error enabled on startup
	ui8_display_function_status[2][TURBO] = m_configuration_variables.ui8_assist_with_error_enabled;
	// riding mode on startup
	ui8_display_riding_mode = m_configuration_variables.ui8_riding_mode;
	// lights configuration on startup
	ui8_display_lights_configuration = m_configuration_variables.ui8_lights_configuration;
	
	// percentage remaining battery capacity x10 at power on
	ui16_battery_SOC_percentage_x10 = ((uint16_t) m_configuration_variables.ui8_battery_SOC_percentage_8b) << 2;
		 
	// battery SOC checked at power on
	if (ui16_battery_SOC_percentage_x10) {
		// calculate watt-hours x10 at power on
		ui32_wh_x10_offset = ((uint32_t)(1000 - ui16_battery_SOC_percentage_x10) * ui16_actual_battery_capacity) / 100;
		
		ui8_battery_SOC_init_flag = 1;
	}

	// make startup boost array This array start with a high value and decrease gradually
	ui16_startup_boost_factor_array[0] = STARTUP_BOOST_TORQUE_FACTOR; //300
	uint8_t ui8_i;
	for (ui8_i = 1; ui8_i < 120; ui8_i++)
	{
		uint16_t ui16_temp = (ui16_startup_boost_factor_array[ui8_i - 1] * STARTUP_BOOST_CADENCE_STEP) >> 8;//delta*20/256
		ui16_startup_boost_factor_array[ui8_i] = ui16_startup_boost_factor_array[ui8_i - 1] - ui16_temp;
	}
	
	// enable data displayed on startup
	#if DATA_DISPLAY_ON_STARTUP
	ui8_display_data_enabled = 1;
	#endif

	// calculate max adc battery current from the received battery current limit // 13*100/16 = 81
	ui8_adc_battery_current_max_temp_1 = (uint8_t)((uint16_t)(m_configuration_variables.ui8_battery_current_max * 100U) 
		/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100); //16 means 0,16A per adc step (so for TSDZ2 it was 13 * 100 /16 = 81 adc steps)

	// calculate the max adc battery power from the power limit received in offroad mode // 500 *100*1000/16
	ui32_adc_battery_power_max_x1000_array[OFFROAD_MODE] = (uint32_t)((uint32_t)TARGET_MAX_BATTERY_POWER * 100U * 1000U)
		/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100; //16 means 0,16A per adc step
	
	// calculate the max adc battery power from the received power limit in street mode
	ui32_adc_battery_power_max_x1000_array[STREET_MODE] = (uint32_t)((uint32_t)STREET_MODE_POWER_LIMIT * 100U * 1000U)
		/ BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100; //16
	
	// set max motor phase current // used in motor.c to perform some checks // 
	uint16_t ui16_temp = ui8_adc_battery_current_max_temp_1 * ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;//187 (if 30A); temp = 81*187 = 15147
	ui8_adc_motor_phase_current_max = (uint8_t)(ui16_temp / ADC_10_BIT_BATTERY_CURRENT_MAX); //112 (if 18A) so 15147/112 = 135 for TSDZ2
	// limit max motor phase current if higher than configured hardware limit (safety)
	if (ui8_adc_motor_phase_current_max > ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX) { //187
		ui8_adc_motor_phase_current_max = ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX;  //187
	}
}


void ebike_app_controller(void) // is called every 25ms by main()
{
	// calculate motor ERPS = electrical rotation per sec ; ui16_hall_counter_total is the number of tick (4usec/tick) for a full electric rotation
    // 0x8000 was the value for TSDZ2; 
	// TSDZ8 should test on a value that is 2 * because there is 4 poles instead of 8 and so it takes more ticks for the same mecanical speed
	//if ((ui16_hall_counter_total >= 0x8000 ) || (ui16_hall_counter_total < 10)) { For TSDZ2
	// > 0X8000 = >32000 ; *4 usec = 0,131 sec per electric rotation ; for TSDZ2 * 8 = 1 sec per rotation = 60 rotations mecanical /sec
	// normally this should not happens because there is already a check in motor.c that set ui16_hall_counter_total = 0xffff when enlapsed time is more than a value
	// So, we should not exceed a uint16_t variable
	if ((ui16_hall_counter_total >= 0xF000 ) || (ui16_hall_counter_total < 10)) {
        ui16_motor_speed_erps = 0;  // speed is 0 if number of ticks is to high
    }
	else 
	{
        //ui16_motor_speed_erps = (uint16_t)(HALL_COUNTER_FREQ >> 2) / (uint16_t)(ui16_tmp >> 2); // 250000/nrOfTicks; so in sec
		ui16_motor_speed_erps = ((uint32_t) HALL_COUNTER_FREQ) / ui16_hall_counter_total; // 250000/nrOfTicks; so rotation in sec
	}
	// calculate the wheel speed
	calc_wheel_speed();
	
	// calculate the cadence and set limits from wheel speed
	calc_cadence();

	// Calculate filtered Battery Voltage (mV)
    get_battery_voltage(); // get a ui16_adc_voltage filtered value and convert it in mv with ui16_battery_voltage_filtered_x1000
	
    // Calculate filtered Battery Current (Ampx10)
    ui8_battery_current_filtered_x10 = (uint8_t)(((uint16_t) ui8_adc_battery_current_filtered * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100) / 10);
	
	// get pedal torque ; calculate ui16_pedal_torque_x100 and ui16_human_power_x10 (human power)
	get_pedal_torque();
	
	// send/receive data, ebike control lights, calc oem wheelspeed, 
	// check system, check battery soc, every 4 cycles (25ms * 4)
	
    static uint8_t ui8_counter;
	switch (ui8_counter++ & 0x03) {
		case 0: 
			#if (DEBUG_ON_UART != 1)
			uart_receive_package();
			#endif
			break;
		case 1:
			ebike_control_lights();
			calc_oem_wheel_speed();
			break;
		case 2:
			#if (DEBUG_ON_UART != 1)
			uart_send_package();
			#endif
			break;
		case 3:
			check_system();
			check_battery_soc();
			break;
	}
	/*
    static uint8_t ui8_counter;
	if (ui8_counter++ & 0x03) {
		check_system(); // to be include in the switch here above when the first test are done and display is used
	}
	*/
	// use received data and sensor input to control motor
    ebike_control_motor();

    /*------------------------------------------------------------------------

     NOTE: regarding function call order

     Do not change order of functions if not absolutely sure it will
     not cause any undesirable consequences.

     ------------------------------------------------------------------------*/
}


static void ebike_control_motor(void) // is called every 25ms by ebike_app_controller()
{
    // reset control variables (safety)
    ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;     // 194
    ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;  //73
    ui8_adc_battery_current_target = 0;
    ui8_duty_cycle_target = 0;
	
	// field weakening enabled
	#if FIELD_WEAKENING_ENABLED
	if ((ui16_motor_speed_erps > MOTOR_SPEED_FIELD_WEAKENING_MIN)
		&& (ui8_adc_battery_current_filtered < ui8_controller_adc_battery_current_target)
		&& (!ui8_adc_throttle_assist)) {
			ui8_field_weakening_erps_delta = ui16_motor_speed_erps - MOTOR_SPEED_FIELD_WEAKENING_MIN;
			ui8_fw_hall_counter_offset_max = ui8_field_weakening_erps_delta >> 5;
			if (ui8_fw_hall_counter_offset_max > FW_HALL_COUNTER_OFFSET_MAX) {
				ui8_fw_hall_counter_offset_max = FW_HALL_COUNTER_OFFSET_MAX;
			}
			ui8_field_weakening_enabled = 1;
	}
	else {
		ui8_field_weakening_enabled = 0;
	}
	#endif
	
	// for testing, we force the 4 parameters used to control the motor
	#if (CALIBRATE_HALL_SENSORS == 1) || ( TEST_WITH_FIXED_PARAMETERS == 1) // for testing we use here fixed parameters (! to fix duty cycle, we set a fixed value to PWM_DUTY_CYCLE_MAX)
	ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;     // 194
    ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;  //73
    ui8_adc_battery_current_target = (1); // set on 8 for testing but does not really matter because we do not reach this value
											// 1 adc10bits = 0,16A so 8 = 1A = 1/0.16=6 : but there is an offset of about 2
    // set duty cycle target
	if (ui8_adc_battery_current_target) {
		ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;  // set to 80 in main.h
	}
	else {
		ui8_duty_cycle_target = 0;
	}
	ui8_riding_mode_parameter =  20; // if it is set on on , it means that there is no assist and motor stays/goes off
	
	#else // not ( CALIBRATE_HALL_SENSORS or TEST_WITH_FIXED_PARAMETERS ): we are not testing with a dummy current and so we calculate the 4 parameters depending on the riding mode

    // select riding mode and calculate ui8_adc_battery_current_target and ui8_duty_cycle_target is 255 (or 0)
    //        It also adapt the ramp up and down inverse step that has an impact on how fast the motor react to a change.
	switch (m_configuration_variables.ui8_riding_mode) {
		case POWER_ASSIST_MODE: apply_power_assist(); break;
		case TORQUE_ASSIST_MODE: apply_torque_assist(); break;
		case CADENCE_ASSIST_MODE: apply_cadence_assist(); break;
		case eMTB_ASSIST_MODE: apply_emtb_assist(); break;
		case HYBRID_ASSIST_MODE: apply_hybrid_assist(); break;
		case CRUISE_MODE: apply_cruise(); break;
		case WALK_ASSIST_MODE: apply_walk_assist(); break;
		case TORQUE_SENSOR_CALIBRATION_MODE: apply_torque_sensor_calibration(); break;
    }
	#endif

    // select optional ADC function
	#if (OPTIONAL_ADC_FUNCTION == THROTTLE_CONTROL)   // in some cases, it can increase the target current and change duty_cyle and ramp up/down
	if (ui8_throttle_mode_array[m_configuration_variables.ui8_street_mode_enabled]) { //  0 means that Throttle is "disabled"
		apply_throttle();
	}
	#elif (OPTIONAL_ADC_FUNCTION == TEMPERATURE_CONTROL)
	apply_temperature_limiting();
	#endif
	
    // speed limit :  reduce ui8_adc_battery_current_target progressively (up to 0) when close to speed limit (or exceed)
    apply_speed_limit();
	
	// Check battery Over-current (read current here in case PWM interrupt for some error was disabled)
	//the resistance/gain in TSDZ8 is 4X smaller than in TSDZ2; still ADC is 12 bits instead of 10; so ADC 12bits TSDZ8 = ADC 10 bits TSDZ2
	// in TSDZ2, we used only the 8 lowest bits of adc; 1 adc step = 0,16A
	// In tsdz8, the resistance is (I expect) 0.003 Ohm ; So 1A => 0,003V => 0,03V (gain aop is 10)*4096/5Vcc = 24,576 steps
	//      SO 1 adc step = 1/24,576 = 0,040A
	// For 10 A, TSDZ2 should gives 10/0,16 = 62 steps
	// For 10 A, TSDZ8 shoud give 10*24,576 steps
	// to convert TSDZ8 steps in the same units as TSDZ2, we shoud take ADC *62/245,76 = 0,25 and divide by 4 (or >>2)
	// current is available in gr0 ch1 result 8 in queue 0 p2.8 and/or in gr0 ch0 result in 12 (p2.8)
	// here we take the average of the 2 conversions and so use >>3 instead of >>2
	uint8_t ui8_temp_adc_current = ((XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 8 ) & 0x00FF) +
    							    (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 12 ) & 0x00FF)) >>3  ;  
	
    if ( ui8_temp_adc_current > ui8_adc_battery_overcurrent){ // 112+50 in tsdz2 (*0,16A) => 26A
        ui8_error_battery_overcurrent = ERROR_BATTERY_OVERCURRENT ;
    }    
    /*
    // Read in assembler to ensure data consistency (conversion overrun)
	// E07 (E04 blinking for XH18)
	#ifndef __CDT_PARSER__ // avoid Eclipse syntax check
	__asm
        ld a, 0x53eb // ADC1->DB5RL
		cp a, _ui8_adc_battery_overcurrent
		jrc 00011$
		mov _ui8_error_battery_overcurrent+0, #ERROR_BATTERY_OVERCURRENT
	00011$:
	__endasm;
	#endif
    */
	if (ui8_error_battery_overcurrent) {
		ui8_system_state = ui8_error_battery_overcurrent;
	}
	
    // reset control parameters if... (safety)
    if ((ui8_brake_state)
	  ||(ui8_system_state == ERROR_MOTOR_BLOCKED)
	  ||(ui8_system_state == ERROR_MOTOR_CHECK)
	  ||(ui8_system_state == ERROR_BATTERY_OVERCURRENT)
	  ||(ui8_battery_SOC_saved_flag)
	  ||(!ui8_motor_enabled)
	  ||(!ui8_assist_level)
	  ||(!ui8_riding_mode_parameter)
	  ||((ui8_system_state != NO_ERROR)&&(!m_configuration_variables.ui8_assist_with_error_enabled))) {
		ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
        ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN; // 73
        ui8_controller_adc_battery_current_target = 0;
        ui8_controller_duty_cycle_target = 0;
    }
	else { // motor can run (no safety issue)
        // limit max current if higher than configured hardware limit (safety) 
        if (ui8_adc_battery_current_max > ADC_10_BIT_BATTERY_CURRENT_MAX) {
            ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
        }
		
		// set limit battery overcurrent
		ui8_adc_battery_overcurrent = ui8_adc_battery_current_max + ADC_10_BIT_BATTERY_EXTRACURRENT; // 8A= 50 extracurrent
		
        // limit target current if higher than max value (safety)
        if (ui8_adc_battery_current_target > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }

        // limit target duty cycle ramp up inverse step if lower than min value (safety)
        if (ui8_duty_cycle_ramp_up_inverse_step < PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN) {  // 24
            ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;    // 24
        }

        // limit target duty cycle ramp down inverse step if lower than min value (safety)
        if (ui8_duty_cycle_ramp_down_inverse_step < PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN) {  // 9
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;    // 9
        }
		
        // set duty cycle ramp up in controller
        ui8_controller_duty_cycle_ramp_up_inverse_step = ui8_duty_cycle_ramp_up_inverse_step;

        // set duty cycle ramp down in controller
        ui8_controller_duty_cycle_ramp_down_inverse_step = ui8_duty_cycle_ramp_down_inverse_step;

        // set target battery current in controller
        ui8_controller_adc_battery_current_target = ui8_adc_battery_current_target;

        // set target duty cycle in controller
        ui8_controller_duty_cycle_target = ui8_duty_cycle_target;
	}
	
    // check if the motor should be enabled or disabled
	// stop the motor e.g. if erps = 0 and current target and duty cycle are both 0
    if (ui8_motor_enabled
		&& ((ui8_system_state == ERROR_BATTERY_OVERCURRENT)
			|| (ui8_battery_SOC_saved_flag)
			|| ((ui16_motor_speed_erps == 0) && (!ui8_adc_battery_current_target) && (!ui8_g_duty_cycle))
			)) {
        ui8_motor_enabled = 0;
        motor_disable_pwm();
    }
	else if (!ui8_motor_enabled
			&& (ui16_motor_speed_erps < 50) // enable the motor only if it rotates slowly or is stopped
			&& (ui8_adc_battery_current_target) // there is a current target
			&& (!ui8_brake_state)) {            // brake is not activated
		ui8_motor_enabled = 1;
		ui8_g_duty_cycle = PWM_DUTY_CYCLE_STARTUP; // 30  (256 = 100% so 30 = about 12%)
		//ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
		//ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
		ui8_fw_hall_counter_offset = 0;
		motor_enable_pwm();
	}
}


// calculate motor ramp depending on speed and cadence
static void set_motor_ramp(void)
{
	uint8_t ui8_tmp;
	if (ui16_wheel_speed_x10 >= 200) {
        ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;     // 24
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN; // 9
    }
	else {
        ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t)(ui16_wheel_speed_x10>>2),
                (uint8_t)10, //  for 4 kph, wheel_speed_x10 = 40; so when divided by 4 = 10  
                (uint8_t)50, //  for 20 kph , wheel_speed_x10 = 200; so when divided by 4 = 150  
                (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default, //194 At low speed, we will change slowly
                (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN); // 24    At high speed we will change faster
        ui8_tmp = map_ui8(ui8_pedal_cadence_RPM,
                (uint8_t)20, // 20 rpm
                (uint8_t)70, // 70 rpm
                (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default, // 194 At low cadence, we will change slowly
                (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN); // 24     At high cadence  we will change faster
        if (ui8_tmp < ui8_duty_cycle_ramp_up_inverse_step) {
            ui8_duty_cycle_ramp_up_inverse_step = ui8_tmp;       // take the lowest (to change faster)
		}
        ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t)(ui16_wheel_speed_x10>>2),
                (uint8_t)10, // 10*4 = 40 -> 4 kph
                (uint8_t)50, // 50*4 = 200 -> 20 kph
                (uint8_t)ui8_duty_cycle_ramp_down_inverse_step_default, // 73
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);  //9
        ui8_tmp = map_ui8(ui8_pedal_cadence_RPM,
                (uint8_t)20, // 20 rpm
                (uint8_t)70, // 70 rpm
                (uint8_t)ui8_duty_cycle_ramp_down_inverse_step_default, // 73
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);  //9
        if (ui8_tmp < ui8_duty_cycle_ramp_down_inverse_step) {
            ui8_duty_cycle_ramp_down_inverse_step = ui8_tmp;  // take the lowest (to change faster)
		}
    }
}


// calculate startup boost & new pedal torque delta
static void apply_startup_boost(void)
{
	// startup boost mode
	switch (ui8_startup_boost_at_zero) {
		case CADENCE:
			ui8_startup_boost_flag = 1;
			break;
		case SPEED:
			if (!ui16_wheel_speed_x10) {
				ui8_startup_boost_flag = 1;
			}
			else if (ui8_pedal_cadence_RPM > 45) {
				ui8_startup_boost_flag = 0;
			}
			break;
	}
	// pedal torque delta & startup boost
	if (ui8_startup_boost_flag) {
		uint32_t ui32_temp = ((uint32_t)(ui16_adc_pedal_torque_delta * ui16_startup_boost_factor_array[ui8_pedal_cadence_RPM])) / 100;
		ui16_adc_pedal_torque_delta += (uint16_t) ui32_temp;
	}
}


// calculate smooth start & new pedal torque delta
static void apply_smooth_start(void)
{
	if ((!ui8_pedal_cadence_RPM)&&(!ui16_motor_speed_erps)) {
		ui8_smooth_start_flag = 1;
		ui8_smooth_start_counter = ui8_smooth_start_counter_set;
	}
	else if (ui8_smooth_start_flag) {
		if (ui8_smooth_start_counter > 0) {
			ui8_smooth_start_counter--;
		}
		else {
			ui8_smooth_start_flag = 0;
		}
		// pedal torque delta & smooth start
		uint16_t ui16_temp = 100 - ((ui8_smooth_start_counter * 100) / ui8_smooth_start_counter_set);
		ui16_adc_pedal_torque_delta = (ui16_adc_pedal_torque_delta * ui16_temp) / 100;
	}
}


static void apply_power_assist(void)
{
	uint8_t ui8_power_assist_multiplier_x50 = ui8_riding_mode_parameter;
	
	// startup boost
	if (m_configuration_variables.ui8_startup_boost_enabled) {
		apply_startup_boost();
	}
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (m_configuration_variables.ui8_assist_without_pedal_rotation_enabled) {
		if ((!ui8_pedal_cadence_RPM) &&
		   (ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
  if ((ui8_pedal_cadence_RPM)||(ui8_startup_assist_adc_battery_current_target)) {
	// calculate torque on pedals + torque startup boost
    uint32_t ui32_pedal_torque_x100 = (uint32_t)(ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100);
	
    // calculate power assist by multiplying human power with the power assist multiplier
    uint32_t ui32_power_assist_x100 = (((uint32_t)(ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x50))
            * ui32_pedal_torque_x100) >> 9; // see note below

    /*------------------------------------------------------------------------

     NOTE: regarding the human power calculation

     (1) Formula: pedal power = torque * rotations per second * 2 * pi
     (2) Formula: pedal power = torque * rotations per minute * 2 * pi / 60
     (3) Formula: pedal power = torque * rotations per minute * 0.1047
     (4) Formula: pedal power = torque * 100 * rotations per minute * 0.001047
     (5) Formula: pedal power = torque * 100 * rotations per minute / 955
     (6) Formula: pedal power * 100  =  torque * 100 * rotations per minute * (100 / 955)
     (7) Formula: assist power * 100  =  torque * 100 * rotations per minute * (100 / 955) * (ui8_power_assist_multiplier_x50 / 50)
     (8) Formula: assist power * 100  =  torque * 100 * rotations per minute * (2 / 955) * ui8_power_assist_multiplier_x50
     (9) Formula: assist power * 100  =  torque * 100 * rotations per minute * ui8_power_assist_multiplier_x50 / 480

     ------------------------------------------------------------------------*/

    // calculate target current
    uint32_t ui32_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;
	
    // set battery current target in ADC steps
    uint16_t ui16_adc_battery_current_target = (uint16_t)ui32_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
    // set motor acceleration / deceleration
	set_motor_ramp();
	
    // set battery current target
    if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) {
        ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    }
	else {
        ui8_adc_battery_current_target = ui16_adc_battery_current_target;
    }
	
	#if STARTUP_ASSIST_ENABLED
	// set startup assist battery current target
	if (ui8_startup_assist_flag) {
		if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
			ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
		}
		ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
	}
	else {
		ui8_startup_assist_adc_battery_current_target = 0;
    }
	#endif
	
    // set duty cycle target
    if (ui8_adc_battery_current_target) {
        ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
    }
	else {
        ui8_duty_cycle_target = 0;
    }
  }
}


 static void apply_torque_assist(void)
{
	// smooth start
	#if SMOOTH_START_ENABLED
	apply_smooth_start();
	#endif
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (m_configuration_variables.ui8_assist_without_pedal_rotation_enabled) {
		if ((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if (m_configuration_variables.ui8_assist_with_error_enabled) {
		ui8_pedal_cadence_RPM = 1;
	}
    // calculate torque assistance
    if (((ui16_adc_pedal_torque_delta)&&(ui8_pedal_cadence_RPM))
	  ||(ui8_startup_assist_adc_battery_current_target)) {
        // get the torque assist factor
        uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter;

        // calculate torque assist target current
        uint16_t ui16_adc_battery_current_target_torque_assist = ((uint16_t) ui16_adc_pedal_torque_delta
                * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR; //120

        // set motor acceleration / deceleration (adapt the ramp up and down inverse step) based on wheel speed and cadence (to react faster when running fast)
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_torque_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist;
        }
		
		#if STARTUP_ASSIST_ENABLED
		// set startup assist battery current target
		if (ui8_startup_assist_flag) {
			if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
				ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
			}
			ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
		}
		else {
			ui8_startup_assist_adc_battery_current_target = 0;
		}
		#endif
		
		// set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}// at this point, ui8_adc_battery_current_target is set and ui8_duty_cycle_target is 255 (or 0)


 static void apply_cadence_assist(void)
{
    if (ui8_pedal_cadence_RPM) {
		// simulated pedal torque delta
		ui16_adc_pedal_torque_delta = (uint16_t)((ui8_riding_mode_parameter + ui8_pedal_cadence_RPM) >> 2);
		
		// smooth start
		apply_smooth_start();
		
        // set cadence assist current target
		uint16_t ui16_adc_battery_current_target_cadence_assist = ui16_adc_pedal_torque_delta;
		
		// restore pedal torque delta
		ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque_delta_temp;
		
		// set motor acceleration / deceleration
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_cadence_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = ui16_adc_battery_current_target_cadence_assist;
        }
		
		// set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}


 static void apply_emtb_assist(void)
{
#define eMTB_ASSIST_DENOMINATOR_MIN			10
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (m_configuration_variables.ui8_assist_without_pedal_rotation_enabled) {
		if ((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if (m_configuration_variables.ui8_assist_with_error_enabled) {
		ui8_pedal_cadence_RPM = 1;
	}
	
	if (((ui16_adc_pedal_torque_delta)&&(ui8_pedal_cadence_RPM))
	  ||(ui8_startup_assist_adc_battery_current_target)) {
		
		// get the eMTB assist denominator torque based
		uint16_t ui16_eMTB_assist_denominator = (508 - (ui8_riding_mode_parameter << 1));
		// get the eMTB assist denominator power based
		if (ui8_eMTB_based_on_power) {
			if (ui16_eMTB_assist_denominator >= ui8_pedal_cadence_RPM) {
				ui16_eMTB_assist_denominator -= ui8_pedal_cadence_RPM;
			}
			else {
				ui16_eMTB_assist_denominator = 0;
			}
		}
		ui16_eMTB_assist_denominator += eMTB_ASSIST_DENOMINATOR_MIN;
		
		// eMTB pedal torque delta calculation (progressive)
		uint16_t ui16_eMTB_adc_pedal_torque_delta = (uint16_t)((uint32_t)((ui16_adc_pedal_torque_delta * ui16_adc_pedal_torque_delta) + ui16_eMTB_assist_denominator)
			/ ui16_eMTB_assist_denominator);
		
		// set eMTB assist target current
		uint16_t ui16_adc_battery_current_target_eMTB_assist = ui16_eMTB_adc_pedal_torque_delta;
		
        // set motor acceleration / deceleration
		set_motor_ramp();
		
        // set battery current target
        if (ui16_adc_battery_current_target_eMTB_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }
		else {
            ui8_adc_battery_current_target = ui16_adc_battery_current_target_eMTB_assist;
        }
		
		#if STARTUP_ASSIST_ENABLED
		// set startup assist battery current target
		if (ui8_startup_assist_flag) {
			if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
				ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
			}
			ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
		}
		else {
			ui8_startup_assist_adc_battery_current_target = 0;
		}
		#endif
		
        // set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }
		else {
            ui8_duty_cycle_target = 0;
        }
    }
}


 static void apply_hybrid_assist(void)
{
	uint16_t ui16_adc_battery_current_target_power_assist;
	uint16_t ui16_adc_battery_current_target_torque_assist;
	uint16_t ui16_adc_battery_current_target;
	
	// smooth start
	#if SMOOTH_START_ENABLED
	apply_smooth_start();
	#endif
	
	// check for assist without pedal rotation when there is no pedal rotation
	if (m_configuration_variables.ui8_assist_without_pedal_rotation_enabled) {
		if ((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (120 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if ((ui8_pedal_cadence_RPM)||(ui8_startup_assist_adc_battery_current_target)) {
		// calculate torque assistance
		if (ui16_adc_pedal_torque_delta) {
			// get the torque assist factor
			uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter_array[TORQUE_ASSIST_MODE - 1][ui8_assist_level];
		
			// calculate torque assist target current
			ui16_adc_battery_current_target_torque_assist = ((uint16_t) ui16_adc_pedal_torque_delta * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;
		}
		else {
			ui16_adc_battery_current_target_torque_assist = 0;
		}
	
		// calculate power assistance
		// get the power assist multiplier
		uint8_t ui8_power_assist_multiplier_x50 = ui8_riding_mode_parameter;

		// calculate power assist by multiplying human power with the power assist multiplier
		uint32_t ui32_power_assist_x100 = (((uint32_t)(ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x50))
				* ui16_pedal_torque_x100) >> 9; // see note below
	
		// calculate power assist target current x100
		uint32_t ui32_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;
	
		// calculate power assist target current
		ui16_adc_battery_current_target_power_assist = (uint16_t)ui32_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
		// set battery current target in ADC steps
		if (ui16_adc_battery_current_target_power_assist > ui16_adc_battery_current_target_torque_assist) {
			ui16_adc_battery_current_target = ui16_adc_battery_current_target_power_assist;
		}
		else {
			ui16_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist;
		}
		// set motor acceleration / deceleration
		set_motor_ramp();
	
		// set battery current target
		if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) {
			ui8_adc_battery_current_target = ui8_adc_battery_current_max;
		}
		else {
			ui8_adc_battery_current_target = ui16_adc_battery_current_target;
		}
		
		#if STARTUP_ASSIST_ENABLED
		// set startup assist battery current target
		if (ui8_startup_assist_flag) {
			if (ui8_adc_battery_current_target > ui8_startup_assist_adc_battery_current_target) {
				ui8_startup_assist_adc_battery_current_target = ui8_adc_battery_current_target;
			}
			ui8_adc_battery_current_target = ui8_startup_assist_adc_battery_current_target;
		}
		else {
			ui8_startup_assist_adc_battery_current_target = 0;
		}
		#endif
		
		// set duty cycle target
		if (ui8_adc_battery_current_target) {
			ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
		}
		else {
			ui8_duty_cycle_target = 0;
		}
	}
}

// when wheel speed is less than predefined, apply cadence mode, once wheel speed is reached, PID adapts duty_cycle to keep the speed 
 static void apply_cruise(void)
{
#define CRUISE_PID_KP							4
#define CRUISE_PID_KD							6
#define CRUISE_PID_KI_X16						10
#define CRUISE_PID_INTEGRAL_LIMIT				1000
#define CRUISE_PID_OUTPUT_LIMIT					1000
	static int16_t i16_error;
	static int16_t i16_last_error;
	static int16_t i16_integral;
	static int16_t i16_derivative;
	static int16_t i16_control_output;
	static uint16_t ui16_wheel_speed_target_x10;
	
	static uint8_t ui8_cruise_PID_initialize = 0;
	static uint8_t ui8_cruise_assist_flag = 0;
	static uint8_t ui8_riding_mode_cruise = 0;
	static uint8_t ui8_riding_mode_cruise_temp = 0;
	static uint8_t ui8_cruise_threshold_speed_x10;
	
#if CRUISE_MODE_ENABLED
	// set cruise speed threshold
	ui8_cruise_threshold_speed_x10 = ui8_cruise_threshold_speed_x10_array[m_configuration_variables.ui8_street_mode_enabled]; // probably 100 = 10 km/h
	
	// verify riding mode change
	if (ui8_riding_mode_cruise_temp == ui8_riding_mode_cruise) {
		// enable cruise assist
		ui8_cruise_assist_flag = 1;
	}
	else {
		// for next verify riding mode change
		ui8_riding_mode_cruise_temp = ui8_riding_mode_cruise;
	}
	
	#if STREET_MODE_CRUISE_ENABLED
		#if CRUISE_MODE_WALK_ENABLED
			#if ENABLE_BRAKE_SENSOR
				if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(ui8_cruise_assist_flag)
					&&((ui8_pedal_cadence_RPM)||(ui8_cruise_button_flag)))
			#else
				if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
					&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))
			#endif
		#else
			if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
				&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))
		#endif
	#else
		#if CRUISE_MODE_WALK_ENABLED
			#if ENABLE_BRAKE_SENSOR
			if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
				&&(!m_configuration_variables.ui8_street_mode_enabled)
				&&(ui8_cruise_assist_flag)
				&&((ui8_pedal_cadence_RPM)||(ui8_cruise_button_flag)))
			#else
			if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
				&&(!m_configuration_variables.ui8_street_mode_enabled)
				&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))
			#endif
		#else
			if ((ui16_wheel_speed_x10 >= ui8_cruise_threshold_speed_x10)
				&&(!m_configuration_variables.ui8_street_mode_enabled)
				&&(ui8_cruise_assist_flag)&&(ui8_pedal_cadence_RPM))
		#endif
	#endif
	{ // part of the IF
		
		// for verify riding mode change
		ui8_riding_mode_cruise = CRUISE_MODE;
		
		// initialize cruise PID controller if not yet done
		if (ui8_cruise_PID_initialize) {
			ui8_cruise_PID_initialize = 0;
			
			// reset PID variables
			i16_error = 0;
			i16_last_error = 0;
			i16_integral = 500; // initialize integral to a value so the motor does not start from zero
			i16_derivative = 0;
			i16_control_output = 0;
			
			// set cruise speed target
			ui16_wheel_speed_target_x10 = (uint16_t) ui8_riding_mode_parameter_array[CRUISE_MODE - 1][ui8_assist_level] * 10;
		}
		
		// calculate error
		i16_error = (ui16_wheel_speed_target_x10 - ui16_wheel_speed_x10);
		
		// calculate integral
		i16_integral = i16_integral + i16_error;
		
		// limit integral
		if (i16_integral > CRUISE_PID_INTEGRAL_LIMIT) {
			i16_integral = CRUISE_PID_INTEGRAL_LIMIT;
		}
		else if (i16_integral < 0) {
			i16_integral = 0;
		}
		
		// calculate derivative
		i16_derivative = i16_error - i16_last_error;
		
		// save error to last error
		i16_last_error = i16_error;

		// calculate control output ( output =  P I D )
		i16_control_output = (CRUISE_PID_KP * i16_error)
							+ ((CRUISE_PID_KI_X16 * i16_integral) >> 4)
							+ (CRUISE_PID_KD * i16_derivative);
		
		// limit control output to just positive values
		if (i16_control_output < 0) {
			i16_control_output = 0;
		}
		
		// limit control output to the maximum value
		if (i16_control_output > CRUISE_PID_OUTPUT_LIMIT) {
			i16_control_output = CRUISE_PID_OUTPUT_LIMIT;
		}
		
		// set motor acceleration / deceleration
        ui8_duty_cycle_ramp_up_inverse_step = CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;   // 244
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73
		
		// set battery current target
		ui8_adc_battery_current_target = ui8_adc_battery_current_max;
		
		// set duty cycle target  |  map the control output to an appropriate target PWM value
		ui8_duty_cycle_target = map_ui8((uint8_t) (i16_control_output >> 2),
			(uint8_t) 0,					// minimum control output from PID
			(uint8_t) 250,					// maximum control output from PID
			(uint8_t) 0,					// minimum duty cycle
			(uint8_t) PWM_DUTY_CYCLE_MAX);	// maximum duty cycle
	}
	else {
		// disable cruise assist
		ui8_cruise_assist_flag = 0;
		ui8_cruise_PID_initialize = 1; // will force a pid initialise later on
		
		// for verify riding mode change
		ui8_riding_mode_cruise = CADENCE_ASSIST_MODE;
				
		// applies cadence assist up to cruise speed threshold
		ui8_riding_mode_parameter = ui8_riding_mode_parameter_array[CADENCE_ASSIST_MODE - 1][ui8_assist_level];
		apply_cadence_assist();
	}
#endif
}


static void apply_walk_assist(void)
{
	if (m_configuration_variables.ui8_assist_with_error_enabled) {
		// get walk assist duty cycle target
		ui8_walk_assist_duty_cycle_target = ui8_riding_mode_parameter + 20;
	}
	else {
		// get walk assist speed target x10
		ui8_walk_assist_speed_target_x10 = ui8_riding_mode_parameter;
		
		// set walk assist duty cycle target
		if ((!ui8_walk_assist_speed_flag)&&(!ui16_motor_speed_erps)) {
			ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_STARTUP; // 50
			ui8_walk_assist_duty_cycle_max = WALK_ASSIST_DUTY_CYCLE_STARTUP;    // 50 
			ui16_walk_assist_wheel_speed_counter = 0;
			ui16_walk_assist_erps_target = 0;
		}
		else if (ui8_walk_assist_speed_flag) {
			if (ui16_motor_speed_erps < ui16_walk_assist_erps_min) {
				ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN; // 4
				
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target < ui8_walk_assist_duty_cycle_max) {
						ui8_walk_assist_duty_cycle_target++;
					}
					else {
						ui8_walk_assist_duty_cycle_max++;
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps < ui16_walk_assist_erps_target) {
				ui8_walk_assist_adj_delay = (ui16_motor_speed_erps - ui16_walk_assist_erps_min) * WALK_ASSIST_ADJ_DELAY_MIN; // 4
				
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target < ui8_walk_assist_duty_cycle_max) {
						ui8_walk_assist_duty_cycle_target++;
					}
					
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps < ui16_walk_assist_erps_max) {
				ui8_walk_assist_adj_delay = (ui16_walk_assist_erps_max - ui16_motor_speed_erps) * WALK_ASSIST_ADJ_DELAY_MIN; // 4
			
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MIN) { //40
						ui8_walk_assist_duty_cycle_target--;
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
			else if (ui16_motor_speed_erps >= ui16_walk_assist_erps_max) {
				ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_MIN; // 4
			
				if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
					if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MIN) { // 40
						ui8_walk_assist_duty_cycle_target--;
						ui8_walk_assist_duty_cycle_max--;
					}
					ui8_walk_assist_duty_cycle_counter = 0;
				}
			}
		}
		else {
			ui8_walk_assist_adj_delay = WALK_ASSIST_ADJ_DELAY_STARTUP; //10
			
			if (ui8_walk_assist_duty_cycle_counter++ > ui8_walk_assist_adj_delay) {
				if (ui16_wheel_speed_x10) {
					if (ui16_wheel_speed_x10 > WALK_ASSIST_WHEEL_SPEED_MIN_DETECT_X10) { //42 = 4,2 km/h
						ui8_walk_assist_duty_cycle_target--;
					}
					
					if (ui16_walk_assist_wheel_speed_counter++ >= 10) {
						ui8_walk_assist_duty_cycle_max += 10;
					
						// set walk assist erps target
						ui16_walk_assist_erps_target = ((ui16_motor_speed_erps * ui8_walk_assist_speed_target_x10) / ui16_wheel_speed_x10);
						ui16_walk_assist_erps_min = ui16_walk_assist_erps_target - WALK_ASSIST_ERPS_THRESHOLD; //20
						ui16_walk_assist_erps_max = ui16_walk_assist_erps_target + WALK_ASSIST_ERPS_THRESHOLD; //20
					
						// set walk assist speed flag
						ui8_walk_assist_speed_flag = 1;
					}
				}
				else {
					if ((ui8_walk_assist_duty_cycle_max + 10) < WALK_ASSIST_DUTY_CYCLE_MAX) { //130
						ui8_walk_assist_duty_cycle_target++;
						ui8_walk_assist_duty_cycle_max++;
					}
				}
				ui8_walk_assist_duty_cycle_counter = 0;
			}
		}
	}

	if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MAX) {  //130
		ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MAX;    //130
	}
	
	// set motor acceleration / deceleration
	ui8_duty_cycle_ramp_up_inverse_step = WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;	// 244
	ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;  // 73
	
	// set battery current target
	ui8_adc_battery_current_target = ui8_min(WALK_ASSIST_ADC_BATTERY_CURRENT_MAX, ui8_adc_battery_current_max);
	// set duty cycle target
	ui8_duty_cycle_target = ui8_walk_assist_duty_cycle_target;  // so here duty_cycle_target can be less than 255
}


static void apply_torque_sensor_calibration(void)  // when used, this is called every 25 ms
{
#define PEDAL_TORQUE_ADC_STEP_MIN_VALUE		160 //  20 << 3
#define PEDAL_TORQUE_ADC_STEP_MAX_VALUE		800 // 100 << 3
	
	static uint8_t ui8_step_counter;
	
	if (ui8_torque_sensor_calibration_started) {
		// increment pedal torque step temp
		if (ui8_step_counter++ & 0x01) {
			ui16_pedal_torque_step_temp++;   // increment once every 50 msec
		}
		if (ui16_pedal_torque_step_temp > PEDAL_TORQUE_ADC_STEP_MAX_VALUE) {
			ui16_pedal_torque_step_temp = PEDAL_TORQUE_ADC_STEP_MIN_VALUE; // alternate from MIN up to MAX
		}
		// calculate pedal torque 10 bit ADC step x100
		ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui16_pedal_torque_step_temp >> 3;
		ui8_pedal_torque_per_10_bit_ADC_step_x100_array[m_configuration_variables.ui8_torque_sensor_adv_enabled] = 
																				ui8_pedal_torque_per_10_bit_ADC_step_x100;
		
		// pedal weight (from LCD3)
		ui16_pedal_weight_x100 = (uint16_t)(((uint32_t) ui16_pedal_torque_x100 * 100) / 167);
		//uint16_t ui16_adc_pedal_torque_delta_simulation = 100; // weight 20kg
		//uint16_t ui16_adc_pedal_torque_delta_simulation = 110; // weight 25kg
		//ui16_pedal_weight_x100 = (uint16_t)(((uint32_t) ui8_pedal_torque_per_10_bit_ADC_step_x100 * ui16_adc_pedal_torque_delta_simulation * 100) / 167);
	}
	else {	
		ui16_pedal_torque_step_temp = PEDAL_TORQUE_ADC_STEP_MIN_VALUE;
	}
}


static void apply_throttle(void)
{
    // map adc value from 0 to 255
	//Next line has been moved from motor.c to here to save time in irq 1 ; >>2 because we use 10 bits instead of 12 bits
	ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0x0FFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5
        
    ui8_throttle_adc_in = map_ui8((uint8_t)(ui16_adc_throttle >> 2), // from 10 bits to 8 bits; then remap from 0 to 255
            (uint8_t) ADC_THROTTLE_MIN_VALUE, // 47
            (uint8_t) ADC_THROTTLE_MAX_VALUE, // 176
            (uint8_t) 0,
            (uint8_t) 255);
	// / set throttle assist
	if (ui8_throttle_adc_in) {  
		ui8_adc_throttle_assist = ui8_throttle_adc_in; // mstrens this could be done in all cases
	}
	else {
		ui8_adc_throttle_assist = 0;
	}
	
	// throttle mode pedaling ; Set assist on 0 when other conditions does not match
	switch (ui8_throttle_mode_array[m_configuration_variables.ui8_street_mode_enabled]) {
        case PEDALING: // Throttle assist only when pedaling
			if (!ui8_pedal_cadence_RPM) {
				ui8_adc_throttle_assist = 0;
			}
          break;
        case W_O_P_6KM_H_AND_PEDALING: // Set assist on 0 when when speed is more than 6km/h and not pedaling 
			if ((ui16_wheel_speed_x10 > WALK_ASSIST_THRESHOLD_SPEED_X10) // 60 =  6km/h
			  &&(!ui8_pedal_cadence_RPM)) {
				ui8_adc_throttle_assist = 0;
			}
          break;
		default:
          break;
	}
	
	// map ADC throttle value from 0 to max battery current
    uint8_t ui8_adc_battery_current_target_throttle = map_ui8((uint8_t) ui8_adc_throttle_assist,
            (uint8_t) 0,
            (uint8_t) 255,
            (uint8_t) 0,
            (uint8_t) ui8_adc_battery_current_max); // in TSDZ2 it was 112 for 18A
	
    if (ui8_adc_battery_current_target_throttle > ui8_adc_battery_current_target) {
        // set motor acceleration / deceleration
        if (ui16_wheel_speed_x10 >= 255) {  // when more than 25 km/h set 
            ui8_duty_cycle_ramp_up_inverse_step = THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN; // 48
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN; // 9
        }  // those tests are not required because the map function already set the limits
		else {
            ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t) 40,
                    (uint8_t) 255,
                    (uint8_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT, // 244
                    (uint8_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);    // 48

            ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t) 40,
                    (uint8_t) 255,
                    (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT, // 73
                    (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);    // 9
        }

		// set battery current target
		if (ui8_adc_battery_current_target_throttle > ui8_adc_battery_current_max) {
			ui8_adc_battery_current_target = ui8_adc_battery_current_max;
		}
		else {
			ui8_adc_battery_current_target = ui8_adc_battery_current_target_throttle;
		}
		// set duty cycle target
		ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
    }// end of current_target_trottle > current_target
}


static void apply_temperature_limiting(void)
{
    // next line has bee moved from motor.c to here to save time in irq 1
	ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0xFFFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5    
	// get ADC measurement
    uint16_t ui16_temp = ui16_adc_throttle;

    // filter ADC measurement to motor temperature variable
    ui16_adc_motor_temperature_filtered = filter(ui16_temp, ui16_adc_motor_temperature_filtered, 8);

    // convert ADC value
    ui16_motor_temperature_filtered_x10 = (uint16_t)(((uint32_t) ui16_adc_motor_temperature_filtered * 10000) / 2048);

	#if (TEMPERATURE_SENSOR_TYPE == TMP36)
	if (ui16_motor_temperature_filtered_x10 > 500) {
		ui16_motor_temperature_filtered_x10 = ui16_motor_temperature_filtered_x10 - 500;
	}
	else {
		ui16_motor_temperature_filtered_x10 = 0;
	}
	#endif
				
    // min temperature value can not be equal or higher than max temperature value
    if (ui8_motor_temperature_min_value_to_limit_array[TEMPERATURE_SENSOR_TYPE] >= ui8_motor_temperature_max_value_to_limit_array[TEMPERATURE_SENSOR_TYPE]) {
        ui8_adc_battery_current_target = 0;
    }
	else {
        // adjust target current if motor over temperature limit
        ui8_adc_battery_current_target = map_ui16((uint16_t) ui16_motor_temperature_filtered_x10,
				(uint16_t) ((uint8_t)ui8_motor_temperature_min_value_to_limit_array[TEMPERATURE_SENSOR_TYPE] * (uint8_t)10U),
				(uint16_t) ((uint8_t)ui8_motor_temperature_max_value_to_limit_array[TEMPERATURE_SENSOR_TYPE] * (uint8_t)10U),
				ui8_adc_battery_current_target,
				0);
	}
}


static void apply_speed_limit(void)
{
	if (m_configuration_variables.ui8_wheel_speed_max > 0U) {
		uint16_t speed_limit_low  = (uint16_t)((uint8_t)(m_configuration_variables.ui8_wheel_speed_max - 2U) * (uint8_t)10U); // casting literal to uint8_t ensures usage of MUL X,A
		uint16_t speed_limit_high = (uint16_t)((uint8_t)(m_configuration_variables.ui8_wheel_speed_max + 2U) * (uint8_t)10U);

        ui8_adc_battery_current_target = (uint8_t)map_ui16(ui16_wheel_speed_x10,
                speed_limit_low,
                speed_limit_high,
                ui8_adc_battery_current_target,
                0U);
		
		if (ui16_wheel_speed_x10 > speed_limit_high) {
			ui8_duty_cycle_target = 0;
		}
    }
}


static void calc_wheel_speed(void)
{
    // calc wheel speed (km/h x10)
    if (ui16_wheel_speed_sensor_ticks) {
        uint16_t ui16_tmp = ui16_wheel_speed_sensor_ticks;
        // rps = PWM_CYCLES_SECOND / ui16_wheel_speed_sensor_ticks (rev/sec)
        // km/h*10 = rps * ui16_wheel_perimeter * ((3600 / (1000 * 1000)) * 10)
        // !!!warning if PWM_CYCLES_SECOND is not a multiple of 1000
		// mstrens : PWM_CYCLES_SECOND = 19047
        ui16_wheel_speed_x10 = (uint16_t)(((uint32_t) m_configuration_variables.ui16_wheel_perimeter * ((PWM_CYCLES_SECOND/1000)*36U)) / ui16_tmp);
	}
	else {
		ui16_wheel_speed_x10 = 0;
    }
}


static void calc_cadence(void)
{
    // get the cadence sensor ticks
    uint16_t ui16_cadence_sensor_ticks_temp = ui16_cadence_sensor_ticks;

    // adjust cadence sensor ticks counter min depending on wheel speed
    ui16_cadence_ticks_count_min_speed_adj = map_ui16(ui16_wheel_speed_x10,
            40,
            400,
            CADENCE_SENSOR_CALC_COUNTER_MIN,  // 4270     // at 4 km/h
            CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED); //341  // at 40 km/h

    // calculate cadence in RPM and avoid zero division
    // !!!warning if PWM_CYCLES_SECOND > 21845
	// * 3 because 60 sec in 1 min and 20 magnetic in one pedal rotation
    if (ui16_cadence_sensor_ticks_temp) {
        ui8_pedal_cadence_RPM = (uint8_t)((PWM_CYCLES_SECOND * 3U) / ui16_cadence_sensor_ticks_temp);
		
		if (ui8_pedal_cadence_RPM > 120) {
			ui8_pedal_cadence_RPM = 120;
		}
	}
	else {
        ui8_pedal_cadence_RPM = 0;
	}
	
	/*-------------------------------------------------------------------------------------------------

     NOTE: regarding the cadence calculation

     Cadence is calculated by counting how many ticks there are between two LOW to HIGH transitions.

     Formula for calculating the cadence in RPM:

     (1) Cadence in RPM = (60 * PWM_CYCLES_SECOND) / CADENCE_SENSOR_NUMBER_MAGNETS) / ticks

     (2) Cadence in RPM = (PWM_CYCLES_SECOND * 3) / ticks

     -------------------------------------------------------------------------------------------------*/
}


static void get_battery_voltage(void)
{
#define READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT   2

    /*---------------------------------------------------------
     NOTE: regarding filter coefficients

     Possible values: 0, 1, 2, 3, 4, 5, 6
     0 equals to no filtering and no delay, higher values
     will increase filtering but will also add a bigger delay.
     ---------------------------------------------------------*/

    static uint16_t ui16_adc_battery_voltage_accumulated;
	
    // low pass filter the voltage readed value, to avoid possible fast spikes/noise
    ui16_adc_battery_voltage_accumulated -= ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
    ui16_adc_battery_voltage_accumulated += ui16_adc_voltage;
	ui16_battery_voltage_filtered_x1000 = (ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT) * BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
}


 static void get_pedal_torque(void)
{
#define TOFFSET_CYCLES 120 // 3sec (25ms*120)
	
	static uint8_t toffset_cycle_counter = 0;
	uint16_t ui16_temp = 0;
	// this has been moved from motor.c to here in order to save time in the irq; >>2 is to go from ADC 12 bits to 10 bits like TSDZ2
	ui16_adc_torque   = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 2 ) & 0xFFFF) >> 2; // torque gr0 ch7 result 2 in bg p2.2
        
    if (toffset_cycle_counter < TOFFSET_CYCLES) {   // at start up get average on 3 sec
        uint16_t ui16_tmp = ui16_adc_torque; // ui16_adc_torque is captured in motor.h ; it could best be captured here
        ui16_adc_pedal_torque_offset_init = filter(ui16_tmp, ui16_adc_pedal_torque_offset_init, 2);
        toffset_cycle_counter++;
		
		// check that offset after TOFFSET_CYCLES is _30/+30 regarding the calibration value (if any)
		if ((toffset_cycle_counter == TOFFSET_CYCLES)&&(ui8_torque_sensor_calibrated)) {
			if ((ui16_adc_pedal_torque_offset_init > ui16_adc_pedal_torque_offset_min)&& 
			  (ui16_adc_pedal_torque_offset_init < ui16_adc_pedal_torque_offset_max)) {
				ui8_adc_pedal_torque_offset_error = 0;
			}
			else {
				ui8_adc_pedal_torque_offset_error = 1;
			}
		}
		
        ui16_adc_pedal_torque = ui16_adc_pedal_torque_offset_init;
		ui16_adc_pedal_torque_offset_cal = ui16_adc_pedal_torque_offset_init + ADC_TORQUE_SENSOR_CALIBRATION_OFFSET; // 6
	}
	else {  // after 3 sec
		// torque sensor offset adjustment
		ui16_adc_pedal_torque_offset = ui16_adc_pedal_torque_offset_cal;
		if (ui8_pedal_cadence_RPM) {
			ui16_adc_pedal_torque_offset -= ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ;  // 20
			ui16_adc_pedal_torque_offset += ADC_TORQUE_SENSOR_OFFSET_ADJ;         // 20 same value, so no impact
		}
		
		// calculate coaster brake threshold
		#if COASTER_BRAKE_ENABLED
		if (ui16_adc_pedal_torque_offset > COASTER_BRAKE_TORQUE_THRESHOLD) {
			//ui16_adc_coaster_brake_threshold = ui16_adc_pedal_torque_offset - COASTER_BRAKE_TORQUE_THRESHOLD;
			ui16_adc_coaster_brake_threshold = ui16_adc_pedal_torque_offset_cal - COASTER_BRAKE_TORQUE_THRESHOLD;
		}
		else {
			ui16_adc_coaster_brake_threshold = 0;
		}
		#endif
		
        // get adc pedal torque
        ui16_adc_pedal_torque = ui16_adc_torque;
    }
	
    // calculate the delta value calibration
    if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_offset) {
		// adc pedal torque delta remapping
		if ((ui8_torque_sensor_calibrated)&&(m_configuration_variables.ui8_torque_sensor_adv_enabled)) {
			// adc pedal torque delta adjustment
			ui16_temp = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset_init;
			ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset
				- ((ADC_TORQUE_SENSOR_DELTA_ADJ	* ui16_temp) / ADC_TORQUE_SENSOR_RANGE_TARGET); // -(14*temp)/160
			
			// adc pedal torque range adjusment
			ui16_temp = (ui16_adc_pedal_torque_delta * ADC_TORQUE_SENSOR_RANGE_INGREASE_X100) / 10; //
			ui16_adc_pedal_torque_delta = (((ui16_temp
				* ((ui16_temp / ADC_TORQUE_SENSOR_ANGLE_COEFF + ADC_TORQUE_SENSOR_ANGLE_COEFF_X10) / ADC_TORQUE_SENSOR_ANGLE_COEFF)) / 50) // 100
				* (100 + PEDAL_TORQUE_ADC_RANGE_ADJ)) / 200; // 100
			
			// adc pedal torque angle adjusment
			if (ui16_adc_pedal_torque_delta < ADC_TORQUE_SENSOR_RANGE_TARGET_MAX) {
				ui16_temp = (ui16_adc_pedal_torque_delta * ADC_TORQUE_SENSOR_RANGE_TARGET_MAX)
					/ (ADC_TORQUE_SENSOR_RANGE_TARGET_MAX - (((ADC_TORQUE_SENSOR_RANGE_TARGET_MAX - ui16_adc_pedal_torque_delta) * 10)
					/ PEDAL_TORQUE_ADC_ANGLE_ADJ));
				
				ui16_adc_pedal_torque_delta = ui16_temp;
			}
		}
		else {
			ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset;
		}
		ui16_adc_pedal_torque_delta = (ui16_adc_pedal_torque_delta + ui16_adc_pedal_torque_delta_temp) >> 1;
	}
	else {
		ui16_adc_pedal_torque_delta = 0;
    }
	ui16_adc_pedal_torque_delta_temp = ui16_adc_pedal_torque_delta; // used to filter the value
	
	// for cadence sensor check
	ui16_adc_pedal_torque_delta_no_boost = ui16_adc_pedal_torque_delta;
	
    // calculate torque on pedals
    ui16_pedal_torque_x100 = ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_x100;
	
	// calculate human power x10
	if ((ui8_torque_sensor_calibrated)&&(m_configuration_variables.ui8_torque_sensor_adv_enabled)) {
		ui16_human_power_x10 = (uint16_t)(((uint32_t)ui16_adc_pedal_torque_delta * ui8_pedal_torque_per_10_bit_ADC_step_calc_x100 * ui8_pedal_cadence_RPM) / 96);
	}
	else {
		ui16_human_power_x10 = (uint16_t)(((uint32_t)ui16_pedal_torque_x100 * ui8_pedal_cadence_RPM) / 96); // see note below
	}
	
	/*------------------------------------------------------------------------

    NOTE: regarding the human power calculation
    
    (1) Formula: power = torque * rotations per second * 2 * pi
    (2) Formula: power = torque * rotations per minute * 2 * pi / 60
    (3) Formula: power = torque * rotations per minute * 0.1047
    (4) Formula: power = torque * 100 * rotations per minute * 0.001047
    (5) Formula: power = torque * 100 * rotations per minute / 955
    (6) Formula: power * 10  =  torque * 100 * rotations per minute / 96
    
	------------------------------------------------------------------------*/
}

/* not used when eeprom is not used
struct_configuration_variables* get_configuration_variables(void)
{
    return &m_configuration_variables;
}
*/

static void check_system(void)
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E09 ERROR_MOTOR_CHECK (E08 blinking for XH18)
// E09 shared with ERROR_WRITE_EEPROM
#define MOTOR_CHECK_TIME_GOES_ALONE_TRESHOLD         	60 // 60 * 100ms = 6.0 seconds (100msec because we perform the check only once on 4 ticks of 25ms)
#define MOTOR_CHECK_ERPS_THRESHOLD                  	10 // for TSDZ2 with 8 poles :20 ERPS 
                                                           // for TSDZ8 with 4 poles we have 2 less erps for the mecanical speed
static uint8_t ui8_riding_torque_mode = 0;
static uint8_t ui8_motor_check_goes_alone_timer = 0U;
	
	// riding modes that use the torque sensor
	if (((m_configuration_variables.ui8_riding_mode == POWER_ASSIST_MODE)
	  ||(m_configuration_variables.ui8_riding_mode == TORQUE_ASSIST_MODE)
	  ||(m_configuration_variables.ui8_riding_mode == HYBRID_ASSIST_MODE)
	  ||(m_configuration_variables.ui8_riding_mode == eMTB_ASSIST_MODE))
		&& (ui8_adc_throttle_assist == 0U)) {
			ui8_riding_torque_mode = 1;
	}
	else {
		ui8_riding_torque_mode = 0;
	}
	// Check if the motor goes alone and with current or duty cycle target = 0 (safety)
	if ((ui16_motor_speed_erps > MOTOR_CHECK_ERPS_THRESHOLD)
		&&((ui8_riding_torque_mode) || (m_configuration_variables.ui8_riding_mode == CADENCE_ASSIST_MODE))
		&& (ui8_adc_battery_current_target == 0U || ui8_duty_cycle_target == 0U)) {
			ui8_motor_check_goes_alone_timer++;
	}
	else {
		ui8_motor_check_goes_alone_timer = 0;
	}
	if (ui8_motor_check_goes_alone_timer > MOTOR_CHECK_TIME_GOES_ALONE_TRESHOLD) {
		ui8_system_state = ERROR_MOTOR_CHECK;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E02 ERROR_TORQUE_SENSOR
    // check torque sensor
    if (ui8_riding_torque_mode) {
		if ((ui16_adc_pedal_torque_offset > 300)
		  ||(ui16_adc_pedal_torque_offset < 10)
		  ||(ui16_adc_pedal_torque > 500)
		  ||(ui8_adc_pedal_torque_offset_error)) {
			// set torque sensor error code
			ui8_system_state = ERROR_TORQUE_SENSOR;
		}
		else if (ui8_system_state == ERROR_TORQUE_SENSOR) {
			// reset error code
			ui8_system_state = NO_ERROR;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E03 ERROR_CADENCE_SENSOR
#define CHECK_CADENCE_SENSOR_COUNTER_THRESHOLD          250 // 250 * 100ms = 25 seconds
#define CADENCE_SENSOR_RESET_COUNTER_THRESHOLD         	80  // 80 * 100ms = 8.0 seconds
#define ADC_TORQUE_SENSOR_DELTA_THRESHOLD				(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET >> 1) + 20)
	static uint8_t ui8_check_cadence_sensor_counter;
	static uint8_t ui8_error_cadence_sensor_counter;
	
	// check cadence sensor
	if ((ui16_adc_pedal_torque_delta_no_boost > ADC_TORQUE_SENSOR_DELTA_THRESHOLD)
	  &&(!ui8_startup_assist_flag)&&(ui8_riding_torque_mode)
	  &&((ui8_pedal_cadence_RPM > 130)||(!ui8_pedal_cadence_RPM))) {
		ui8_check_cadence_sensor_counter++;
	}
	else {
		ui8_check_cadence_sensor_counter = 0;
	}
	
	if (ui8_check_cadence_sensor_counter > CHECK_CADENCE_SENSOR_COUNTER_THRESHOLD) {
		// set cadence sensor error code
		ui8_system_state = ERROR_CADENCE_SENSOR;
	}
	else if (ui8_system_state == ERROR_CADENCE_SENSOR) {
		// increment cadence sensor error reset counter
        ui8_error_cadence_sensor_counter++;

        // check if the counter has counted to the set threshold for reset
        if (ui8_error_cadence_sensor_counter > CADENCE_SENSOR_RESET_COUNTER_THRESHOLD) {
            // reset cadence sensor error code
            if (ui8_system_state == ERROR_CADENCE_SENSOR) {
				ui8_system_state = NO_ERROR;
				ui8_error_cadence_sensor_counter = 0;
            }
		}
	}
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E08 ERROR_SPEED_SENSOR
#define CHECK_SPEED_SENSOR_COUNTER_THRESHOLD          125 // 125 * 100ms = 12.5 seconds
#define MOTOR_ERPS_SPEED_THRESHOLD	                  180
	static uint16_t ui16_check_speed_sensor_counter;
	static uint16_t ui16_error_speed_sensor_counter;
	
	// check speed sensor
	if ((ui16_motor_speed_erps > MOTOR_ERPS_SPEED_THRESHOLD)  // 180
	  &&(m_configuration_variables.ui8_riding_mode != WALK_ASSIST_MODE)
	  &&(m_configuration_variables.ui8_riding_mode != CRUISE_MODE)
	  &&(!m_configuration_variables.ui8_assist_with_error_enabled)) {
		ui16_check_speed_sensor_counter++;
	}
	else {
		ui16_check_speed_sensor_counter = 0;
	}
	
	if (ui16_wheel_speed_x10) {
		ui16_check_speed_sensor_counter = 0;
	}
	
	if (ui16_check_speed_sensor_counter > CHECK_SPEED_SENSOR_COUNTER_THRESHOLD) { //125
		
		// set speed sensor error code
		ui8_system_state = ERROR_SPEED_SENSOR;
	}
	else if (ui8_system_state == ERROR_SPEED_SENSOR) {
		// increment speed sensor error reset counter
        ui16_error_speed_sensor_counter++;

        // check if the counter has counted to the set threshold for reset
        if (ui16_error_speed_sensor_counter > CHECK_SPEED_SENSOR_COUNTER_THRESHOLD) {
            // reset speed sensor error code
            if (ui8_system_state == ERROR_SPEED_SENSOR) {
				ui8_system_state = NO_ERROR;
				ui16_error_speed_sensor_counter = 0;
            }
		}
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E04 ERROR_MOTOR_BLOCKED
// define moved ih CONFIG.H
//#define MOTOR_BLOCKED_COUNTER_THRESHOLD               2  // 2 * 100ms = 0.2 seconds
//#define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10   30 // 30 = 3.0 amps
//#define MOTOR_BLOCKED_ERPS_THRESHOLD                  20 // 20 ERPS
#define MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD         	80  // 80 * 100ms = 8.0 seconds

    static uint8_t ui8_motor_blocked_counter;
    static uint8_t ui8_motor_blocked_reset_counter;

    // if the motor blocked error is enabled start resetting it
    if (ui8_system_state == ERROR_MOTOR_BLOCKED) {
        // increment motor blocked reset counter with 25 milliseconds
        ui8_motor_blocked_reset_counter++;

        // check if the counter has counted to the set threshold for reset
        if (ui8_motor_blocked_reset_counter > MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD) {
            // reset motor blocked error code
            if (ui8_system_state == ERROR_MOTOR_BLOCKED) {
                ui8_system_state = NO_ERROR;
            }

            // reset the counter that clears the motor blocked error
            ui8_motor_blocked_reset_counter = 0;
        }
    }
	else {
        // if battery current is over the current threshold and the motor ERPS is below threshold start setting motor blocked error code
        if ((ui8_battery_current_filtered_x10 > MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10) 
                && (ui16_motor_speed_erps < MOTOR_BLOCKED_ERPS_THRESHOLD)) {
            // increment motor blocked counter with 100 milliseconds
            ++ui8_motor_blocked_counter;

            // check if motor is blocked for more than some safe threshold
            if (ui8_motor_blocked_counter > MOTOR_BLOCKED_COUNTER_THRESHOLD) {
                // set error code
                ui8_system_state = ERROR_MOTOR_BLOCKED;

                // reset motor blocked counter as the error code is set
                ui8_motor_blocked_counter = 0;
            }
        }
		else {
            // current is below the threshold and/or motor ERPS is above the threshold so reset the counter
            ui8_motor_blocked_counter = 0;
        }
    }
	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E05 ERROR_THROTTLE
#define THROTTLE_CHECK_COUNTER_THRESHOLD		 20 // 20 * 100ms = 2.0 seconds
#define ADC_THROTTLE_MIN_VALUE_THRESHOLD		(uint8_t)(ADC_THROTTLE_MIN_VALUE + 5)

    static uint8_t ui8_throttle_check_counter;
	// next line has been copied from motor.c to save time in irq 1
	ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0xFFFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5    	
	if (ui8_throttle_mode_array[m_configuration_variables.ui8_street_mode_enabled]) {
		if (ui8_throttle_check_counter < THROTTLE_CHECK_COUNTER_THRESHOLD) { // 20 // so we have to wait 25 msec * 20 = 500msec
			ui8_throttle_check_counter++;
		
			if ((ui16_adc_throttle >> 2) > ADC_THROTTLE_MIN_VALUE_THRESHOLD) { //47+5
				ui8_system_state = ERROR_THROTTLE;   // throttle may not ve activated at the beginning.
			}
		}
		else {
			if ((ui8_system_state == ERROR_THROTTLE)
			  &&((ui16_adc_throttle >> 2) < ADC_THROTTLE_MIN_VALUE_THRESHOLD)) { //47+5
				// reset throttle error code
				ui8_system_state = NO_ERROR;
			}	
		}
	}
	else {
		ui8_throttle_check_counter = 0;
		// reset throttle error code
		if (ui8_system_state == ERROR_THROTTLE) {
			ui8_system_state = NO_ERROR;
		}
    }
}

static void calc_oem_wheel_speed(void)
{ 
	if (ui8_display_ready_flag) {
		uint32_t ui32_oem_wheel_speed_time;
		uint32_t ui32_oem_wheel_perimeter;
			
		// calc oem wheel speed (wheel turning time)
		if (ui16_wheel_speed_sensor_ticks) {
			ui32_oem_wheel_speed_time = ((uint32_t) ui16_wheel_speed_sensor_ticks * 10) / OEM_WHEEL_SPEED_DIVISOR;
			
			// speed conversion for different perimeter			
			ui32_oem_wheel_perimeter = ((uint32_t) ui8_oem_wheel_diameter * 7975) / 100; // 25.4 * 3.14 * 100 = 7975
			ui32_oem_wheel_speed_time *= ui32_oem_wheel_perimeter;
			ui32_oem_wheel_speed_time /= (uint32_t) m_configuration_variables.ui16_wheel_perimeter;
			
			// oem wheel speed (wheel turning time) ms/2
			ui16_oem_wheel_speed_time = (uint16_t) ui32_oem_wheel_speed_time;
		}
		else {
			ui16_oem_wheel_speed_time = 0;
		}
	}
	
	#if ENABLE_ODOMETER_COMPENSATION
	uint16_t ui16_wheel_speed;
	uint16_t ui16_data_speed;
	uint16_t ui16_speed_difference;

	// calc wheel speed  mm/0.1 sec
	if (ui16_oem_wheel_speed_time) {
		ui16_wheel_speed = (uint16_t)((ui16_display_data_factor / ui16_oem_wheel_speed_time) * ((uint16_t) 100 / 36));
	}
	else {
		ui16_wheel_speed = 0;
	}
	// calc data speed  mm/0.1 sec
	if (ui16_display_data) {
		ui16_data_speed = (uint16_t)((ui16_display_data_factor / ui16_display_data) * ((uint16_t) 100 / 36));
	}
	else {
		ui16_data_speed = 0;
	}
	// calc odometer difference
	if (ui8_display_data_enabled) {	
		if (ui16_data_speed > ui16_wheel_speed) {	
			// calc + speed difference mm/0.1 sec
			ui16_speed_difference = ui16_data_speed - ui16_wheel_speed;
			// add difference to odometer
			ui32_odometer_compensation_mm += (uint32_t) ui16_speed_difference;
		}
		else {
			// calc - speed difference mm/0.1 sec
			ui16_speed_difference = ui16_wheel_speed - ui16_data_speed;
				// subtracts difference from odometer
			ui32_odometer_compensation_mm -= (uint32_t) ui16_speed_difference;
		}
	}
	else {
		// odometer compensation
		if ((ui16_wheel_speed)&&(ui32_odometer_compensation_mm > ZERO_ODOMETER_COMPENSATION)) {
			ui32_odometer_compensation_mm -= (uint32_t) ui16_wheel_speed;
			ui16_oem_wheel_speed_time = 0;
		}
	}
	#endif
} 


static void check_battery_soc(void)
{
	static uint8_t ui8_no_load_counter = 20;
	uint16_t ui16_battery_voltage_x10;
	uint16_t ui16_battery_SOC_used_x10;
	uint16_t ui16_actual_battery_SOC_x10;
	
	// battery voltage x10
	ui16_battery_voltage_x10 = (ui16_battery_voltage_filtered_x1000) / 100;
	
	// filter battery voltage x10
	ui16_battery_voltage_filtered_x10 = filter(ui16_battery_voltage_x10, ui16_battery_voltage_filtered_x10, 2);
	
	// save no load voltage x10 if current is < adc current min for 2 seconds
	if (ui8_adc_battery_current_filtered < 2) {
		if (++ui8_no_load_counter > 20) {
			ui16_battery_no_load_voltage_filtered_x10 = ui16_battery_voltage_x10;
			ui8_no_load_counter--;
		}
	}
	else {
		ui8_no_load_counter = 0;
	}

	// filter battery voltage soc x10
	ui16_battery_voltage_soc_filtered_x10 = filter(ui16_battery_no_load_voltage_filtered_x10, ui16_battery_voltage_soc_filtered_x10, 2);

	#if ENABLE_VLCD6 || ENABLE_XH18
	if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_6_X10) { ui8_battery_state_of_charge = 7; }		// overvoltage
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_5_X10) { ui8_battery_state_of_charge = 6; }	// 4 bars -> SOC reset
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_4_X10) { ui8_battery_state_of_charge = 5; }	// 4 bars -> full
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_3_X10) { ui8_battery_state_of_charge = 4; }	// 3 bars
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_2_X10) { ui8_battery_state_of_charge = 3; }	// 2 bars
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_1_X10) { ui8_battery_state_of_charge = 2; }	// 1 bar
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_0_X10) { ui8_battery_state_of_charge = 1; }	// blink -> empty
	else { ui8_battery_state_of_charge = 0; } // undervoltage
	#else // ENABLE_VLCD5 or 850C
	if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_8_X10) { ui8_battery_state_of_charge = 9; }   		// overvoltage
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_7_X10) { ui8_battery_state_of_charge = 8; }	// 6 bars -> SOC reset
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_6_X10) { ui8_battery_state_of_charge = 7; }	// 6 bars -> full
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_5_X10) { ui8_battery_state_of_charge = 6; }	// 5 bars
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_4_X10) { ui8_battery_state_of_charge = 5; }	// 4 bars
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_3_X10) { ui8_battery_state_of_charge = 4; }	// 3 bars
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_2_X10) { ui8_battery_state_of_charge = 3; }	// 2 bars
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_1_X10) { ui8_battery_state_of_charge = 2; }	// 1 bar
	else if (ui16_battery_voltage_soc_filtered_x10 > BATTERY_SOC_VOLTS_0_X10) { ui8_battery_state_of_charge = 1; }	// blink -> empty
	else { ui8_battery_state_of_charge = 0; } // undervoltage
	#endif
		
	// battery power x 10
	ui16_battery_power_x10 = (uint16_t)(((uint32_t) ui16_battery_voltage_filtered_x10 * ui8_battery_current_filtered_x10) / 10);
	
	// consumed watt-hours
	ui32_wh_sum_x10 += ui16_battery_power_x10;
	// calculate watt-hours X10 since power on
	ui32_wh_since_power_on_x10 = ui32_wh_sum_x10 / 32400; // 36000 -10% for calibration
	// calculate watt-hours X10 since last full charge
	ui32_wh_x10 = ui32_wh_x10_offset + ui32_wh_since_power_on_x10;
	
	// calculate and set remaining percentage x10
	if (m_configuration_variables.ui8_soc_percent_calculation == 2) { // Volts
			ui16_battery_SOC_percentage_x10 = read_battery_soc();
	}
	else { // Auto or Wh
		// calculate percentage battery capacity used x10
		ui16_battery_SOC_used_x10 = (uint16_t)(((uint32_t) ui32_wh_x10 * 100) / ui16_actual_battery_capacity);
		
		// limit used percentage to 100 x10
		if (ui16_battery_SOC_used_x10 > 1000) {
			ui16_battery_SOC_used_x10 = 1000;
		}
		ui16_battery_SOC_percentage_x10 = 1000 - ui16_battery_SOC_used_x10;
	}
	
	// convert remaining percentage x10 to 8 bit
	m_configuration_variables.ui8_battery_SOC_percentage_8b = (uint8_t)(ui16_battery_SOC_percentage_x10 >> 2);
	
	// automatic set battery percentage x10 (full charge)
	if ((ui8_display_ready_flag))
	{	
		if ((!ui8_battery_SOC_init_flag)||
		  ((!ui8_battery_SOC_reset_flag)&&(ui16_battery_voltage_filtered_x10 > BATTERY_VOLTAGE_RESET_SOC_PERCENT_X10)))
		{
			ui16_battery_SOC_percentage_x10 = 1000;
			ui32_wh_x10_offset = 0;
			ui8_battery_SOC_init_flag = 1;
		}
		
		// battery SOC reset flag
		if ((ui8_battery_SOC_init_flag)
		  &&(!ui8_battery_SOC_reset_flag)
		  &&(ui8_startup_counter++ > DELAY_MENU_ON))
		{
			ui16_actual_battery_SOC_x10 = read_battery_soc();
			
			// check soc percentage
			if ((m_configuration_variables.ui8_soc_percent_calculation == 0) // Auto
				&& (ui16_battery_SOC_percentage_x10 > BATTERY_SOC_PERCENT_THRESHOLD_X10)
				&& ((ui16_actual_battery_SOC_x10 < (ui16_battery_SOC_percentage_x10 - BATTERY_SOC_PERCENT_THRESHOLD_X10))
				|| (ui16_actual_battery_SOC_x10 > (ui16_battery_SOC_percentage_x10 + BATTERY_SOC_PERCENT_THRESHOLD_X10)))) {
					ui16_battery_SOC_percentage_x10 = ui16_actual_battery_SOC_x10;
					
					// calculate watt-hours x10
					ui32_wh_x10_offset = ((uint32_t) (1000 - ui16_battery_SOC_percentage_x10) * ui16_actual_battery_capacity) / 100;
					
					// for display soc %
					ui8_display_battery_soc = 1;
					ui8_display_data_enabled = 1;
					ui8_startup_counter = DELAY_MENU_ON >> 1;
					ui8_menu_counter = DELAY_MENU_ON >> 1;
			}
			else {
				ui8_battery_SOC_reset_flag = 1;
			}
		}
	}
}


// read battery percentage x10 (actual charge)
uint16_t read_battery_soc(void)
{
	uint16_t ui16_battery_SOC_calc_x10 = 0;
	
	#if ENABLE_VLCD6 || ENABLE_XH18
	switch (ui8_battery_state_of_charge) {
		case 0:	ui16_battery_SOC_calc_x10 = 10; break;  // undervoltage
		case 1:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(1, 250, LI_ION_CELL_VOLTS_1_X100, LI_ION_CELL_VOLTS_0_X100); break; // blink - empty
		case 2:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(250, 250, LI_ION_CELL_VOLTS_2_X100, LI_ION_CELL_VOLTS_1_X100); break; // 1 bars
		case 3:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(500, 250, LI_ION_CELL_VOLTS_3_X100, LI_ION_CELL_VOLTS_2_X100); break; // 2 bars
		case 4:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(750, 250, LI_ION_CELL_VOLTS_4_X100, LI_ION_CELL_VOLTS_3_X100); break; // 3 bars
		case 5:	ui16_battery_SOC_calc_x10 = 1000; break; // 4 bars - full
		case 6:	ui16_battery_SOC_calc_x10 = 1000; break; // 4 bars - SOC reset
		case 7:	ui16_battery_SOC_calc_x10 = 1000; break; // overvoltage
	}
	#else // ENABLE_VLCD5 or 850C
	switch (ui8_battery_state_of_charge) {
		case 0:	ui16_battery_SOC_calc_x10 = 10; break;  // undervoltage
		case 1:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(1, 167, LI_ION_CELL_VOLTS_1_X100, LI_ION_CELL_VOLTS_0_X100); break; // blink - empty
		case 2:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(167, 167, LI_ION_CELL_VOLTS_2_X100, LI_ION_CELL_VOLTS_1_X100); break; // 1 bars
		case 3:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(334, 167, LI_ION_CELL_VOLTS_3_X100, LI_ION_CELL_VOLTS_2_X100); break; // 2 bars
		case 4:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(500, 167, LI_ION_CELL_VOLTS_4_X100, LI_ION_CELL_VOLTS_3_X100); break; // 3 bars
		case 5:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(667, 167, LI_ION_CELL_VOLTS_5_X100, LI_ION_CELL_VOLTS_4_X100); break; // 4 bars
		case 6:	ui16_battery_SOC_calc_x10 = calc_battery_soc_x10(834, 167, LI_ION_CELL_VOLTS_6_X100, LI_ION_CELL_VOLTS_5_X100); break; // 5 bars
		case 7:	ui16_battery_SOC_calc_x10 = 1000; break; // 6 bars - full
		case 8:	ui16_battery_SOC_calc_x10 = 1000; break; // 6 bars - SOC reset
		case 9:	ui16_battery_SOC_calc_x10 = 1000; break; // overvoltage
	}	
	#endif
	
	return ui16_battery_SOC_calc_x10;
}


// calculate battery soc percentage x10
uint16_t calc_battery_soc_x10(uint16_t ui16_battery_soc_offset_x10, uint16_t ui16_battery_soc_step_x10, uint16_t ui16_cell_volts_max_x100, uint16_t ui16_cell_volts_min_x100)
{
#define CELL_VOLTS_CALIBRATION				8
	
	uint16_t ui16_cell_voltage_soc_x100 = ui16_battery_voltage_soc_filtered_x10 * 10 / BATTERY_CELLS_NUMBER;
	
	uint16_t ui16_battery_soc_calc_temp_x10 = (ui16_battery_soc_offset_x10 + (ui16_battery_soc_step_x10 * (ui16_cell_voltage_soc_x100 - ui16_cell_volts_min_x100) / (ui16_cell_volts_max_x100 - ui16_cell_volts_min_x100 + CELL_VOLTS_CALIBRATION)));
	
	return ui16_battery_soc_calc_temp_x10;
}

static void ebike_control_lights(void)
{
#define DEFAULT_FLASH_ON_COUNTER_MAX      3
#define DEFAULT_FLASH_OFF_COUNTER_MAX     2
#define BRAKING_FLASH_ON_COUNTER_MAX      1
#define BRAKING_FLASH_OFF_COUNTER_MAX     1

    //static uint8_t ui8_default_flash_state;
    static uint8_t ui8_default_flash_state_counter; // increments every function call -> 100 ms
    static uint8_t ui8_braking_flash_state;
    static uint8_t ui8_braking_flash_state_counter; // increments every function call -> 100 ms

    /****************************************************************************/

    // increment flash counters
    ++ui8_default_flash_state_counter;
    ++ui8_braking_flash_state_counter;

    /****************************************************************************/

    // set default flash state
    if ((ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_ON_COUNTER_MAX)) {
        // reset flash state counter
        ui8_default_flash_state_counter = 0;

        // toggle flash state
        ui8_default_flash_state = 0;
    }
	else if ((!ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_OFF_COUNTER_MAX)) {
        // reset flash state counter
        ui8_default_flash_state_counter = 0;

        // toggle flash state
        ui8_default_flash_state = 1;
    }

    /****************************************************************************/

    // set braking flash state
    if ((ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_ON_COUNTER_MAX)) {
        // reset flash state counter
        ui8_braking_flash_state_counter = 0;

        // toggle flash state
        ui8_braking_flash_state = 0;
    }
	else if ((!ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_OFF_COUNTER_MAX)) {
        // reset flash state counter
        ui8_braking_flash_state_counter = 0;

        // toggle flash state
        ui8_braking_flash_state = 1;
    }

    /****************************************************************************/

    // select lights configuration
    switch (m_configuration_variables.ui8_lights_configuration) {
      case 0:
        // set lights
        lights_set_state(ui8_lights_state);
        break;
      case 1:
        // check lights state
        if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 2:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 3:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 4:
        // check light and brake state
        if (ui8_lights_state && ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 5:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 6:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 7:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_brake_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
      case 8:
        // check brake state
        if (ui8_brake_state) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        }
		else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        }
		else {
            // set lights
            lights_set_state(ui8_lights_state);
        }
        break;
	  default:
        // set lights
        lights_set_state(ui8_lights_state);
        break;
    }

    /*------------------------------------------------------------------------------------------------------------------

     NOTE: regarding the various light modes

     (0) lights ON when enabled
     (1) lights FLASHING when enabled

     (2) lights ON when enabled and BRAKE-FLASHING when braking
     (3) lights FLASHING when enabled and ON when braking
     (4) lights FLASHING when enabled and BRAKE-FLASHING when braking

     (5) lights ON when enabled, but ON when braking regardless if lights are enabled
     (6) lights ON when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled

     (7) lights FLASHING when enabled, but ON when braking regardless if lights are enabled
     (8) lights FLASHING when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled

     ------------------------------------------------------------------------------------------------------------------*/
}

