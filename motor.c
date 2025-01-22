
#include "main.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "motor.h"
#include "ebike_app.h"
#include "common.h"

#include "cy_retarget_io.h"
//#include "cy_utils.h"

// **************  to test slow rotation without using the hall sensor and so discover pattern sequence
// just to test rotation at a low speed and low power to verify the the hall sequence is OK
#define SPEED_COUNTER_MAX 19000 /360  // one electrical rotation per sec ; so 1 mecanical rotation takes 4 sec ; so 15 rpm
#define DUTY_CYCLE_TEST 30// 256 = 100% ; 40 gives a current = 1A from ADC on pin 2.8 with a 12V battery
#define ANGLE_INIT 0
// end of those test parameters


// patern for hall sensor is 1,3,2,6,4, 5
// This table should be read with expected pattern and so upload in shadow register for the next expected
// when current pattern is 1 and expected = 3 , the sadow register should be prepare for the next transition with current = 3 and exp=6 
uint8_t expected_pattern_table[8] = {
    3, // 0 should not happen
    3, // after 1 => 3 
    6, // after 2 => 6
    2, // after 3 => 2
    5, // after 4 => 5
    1, // after 5 => 1
    4, // after 6 => 4
    1 // 7 should not happen 
};


// copied from tsdz2
#define SVM_TABLE_LEN   256
// svm table 19 Khz
/*
static const uint8_t ui8_svm_table[SVM_TABLE_LEN] = { 202, 203, 205, 206, 207, 208, 209, 210, 211, 211, 212, 213, 213,
        214, 214, 214, 215, 215, 215, 215, 215, 215, 215, 215, 214, 214, 214, 213, 213, 212, 211, 211, 210, 209, 208,
        208, 207, 206, 205, 204, 202, 201, 199, 195, 191, 187, 183, 178, 174, 170, 165, 161, 157, 152, 148, 143, 139,
        134, 130, 125, 121, 116, 112, 108, 103, 99, 94, 90, 85, 81, 76, 72, 67, 63, 58, 54, 50, 45, 41, 37, 32, 28, 24,
        20, 16, 14, 13, 11, 10, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4,
        4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 13, 12, 10, 9, 8, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 10, 11, 13, 14, 16, 20, 24, 28, 32, 37, 41, 45, 50, 54, 58, 63, 67, 72,
        76, 81, 85, 90, 94, 99, 103, 108, 112, 116, 121, 125, 130, 134, 139, 143, 148, 152, 157, 161, 165, 170, 174,
        178, 183, 187, 191, 195, 199, 201, 202, 204, 205, 206, 207, 208, 208, 209, 210, 211, 211, 212, 213, 213, 214,
        214, 214, 215, 215, 215, 215, 215, 215, 215, 215, 214, 214, 214, 213, 213, 212, 211, 211, 210, 209, 208, 207,
        206, 205, 203, 202, 201 };
*/
// for tsdz8 using a timer counting up to 1680 (instead of 420)
// svm table 19 Khz
static const uint16_t ui16_svm_table[SVM_TABLE_LEN] = {
1566,1576,1586,1595,1604,1612,1620,1627,1634,1640,1646,1651,1656,1661,1665,1668,1671,1674,1676,
1677,1678,1678,1678,1678,1677,1675,1673,1670,1667,1664,1659,1655,1650,1644,1638,1632,1625,1617,
1609,1601,1592,1583,1573,1556,1524,1493,1461,1428,1396,1363,1329,1295,1261,1227,1193,1158,1123,
1088,1053,1018,982,947,911,876,840,804,769,733,698,662,627,592,557,522,487,453,419,385,351,317,
284,252,219,187,156,124,107,97,88,79,71,63,55,48,42,36,30,25,21,16,13,10,7,5,3,2,2,2,2,3,4,6,9,
12,15,19,24,29,34,40,46,53,60,68,76,85,94,104,114,104,94,85,76,68,60,53,46,40,34,29,24,19,15,12,
9,6,4,3,2,2,2,2,3,5,7,10,13,16,21,25,30,36,42,48,55,63,71,79,88,97,107,124,156,187,219,252,284,
317,351,385,419,453,487,522,557,592,627,662,698,733,769,804,840,876,911,947,982,1018,1053,1088,
1123,1158,1193,1227,1261,1295,1329,1363,1396,1428,1461,1493,1524,1556,1573,1583,1592,1601,1609,
1617,1625,1632,1638,1644,1650,1655,1659,1664,1667,1670,1673,1675,1677,1678,1678,1678,1678,1677,
1676,1674,1671,1668,1665,1661,1656,1651,1646,1640,1634,1627,1620,1612,1604,1595,1586,1576};



/* it was first this table but there was a small bug in a formula (255 instead of 256)
1569,1579,1588,1597,1606,1614,1622,1629,1636,1642,1648,1653,1658,1662,1666,1669,1672,1674,1676,
1677,1678,1678,1678,1677,1676,1674,1672,1669,1666,1662,1658,1653,1648,1642,1636,1629,1622,1614,
1606,1597,1588,1579,1569,1543,1511,1479,1447,1414,1381,1348,1314,1280,1246,1211,1177,1142,1107,
1072,1036,1001,965,929,894,858,822,786,751,715,679,644,608,573,538,503,469,434,400,366,332,299,
266,233,201,169,137,111,101,92,83,74,66,58,51,44,38,32,27,22,18,14,11,8,6,4,3,2,2,2,3,4,6,8,11,
14,18,22,27,32,38,44,51,58,66,74,83,92,101,111,106,97,87,78,70,62,55,48,41,35,30,25,20,16,12,9,
7,5,3,2,2,2,2,3,5,7,9,12,16,20,25,30,35,41,48,55,62,70,78,87,97,106,122,153,185,217,249,282,315,
349,383,417,451,486,521,556,591,626,662,697,733,768,804,840,876,912,947,983,1018,1054,1089,1124,
1159,1194,1229,1263,1297,1331,1365,1398,1431,1463,1495,1527,1558,1574,1583,1593,1602,1610,1618,
1625,1632,1639,1645,1650,1655,1660,1664,1668,1671,1673,1675,1677,1678,1678,1678,1678,1677,1675,
1673,1671,1668,1664,1660,1655,1650,1645,1639,1632,1625,1618,1610,1602,1593,1583,1574};
*/

// motor variables
uint8_t ui8_hall_360_ref_valid = 0; // fill with a hall pattern to check sequence is correct
uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
static uint8_t ui8_motor_phase_absolute_angle;
volatile uint16_t ui16_hall_counter_total = 0xffff; // number of tim3 ticks between 2 rotations// inTSDZ2 it was a u16

// power variables
volatile uint8_t ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT; // 194
volatile uint8_t ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT; // 73
volatile uint16_t ui16_adc_voltage_cut_off = 300*100/BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000; // 30Volt default value =  300*100/87 in TSDZ2
volatile uint8_t ui8_adc_battery_current_filtered = 0;
volatile uint8_t ui8_controller_adc_battery_current_target = 0;
volatile uint8_t ui8_g_duty_cycle = 0;
volatile uint8_t ui8_controller_duty_cycle_target = 0;
// Field Weakening Hall offset (added during interpolation)
volatile uint8_t ui8_fw_hall_counter_offset = 0;
volatile uint8_t ui8_fw_hall_counter_offset_max = 0;
volatile uint8_t ui8_field_weakening_enabled = 0;

// Duty cycle ramp up
static uint8_t ui8_counter_duty_cycle_ramp_up = 0;
static uint8_t ui8_counter_duty_cycle_ramp_down = 0;

// FOC angle
static uint8_t ui8_foc_angle_accumulated;
static uint8_t ui8_foc_flag;
static uint8_t ui8_g_foc_angle = 0;
static uint8_t ui8_foc_angle_multiplier = FOC_ANGLE_MULTIPLIER; //39 for 48V motor
static uint8_t ui8_adc_foc_angle_current = 0;

// battery current variables
static uint8_t ui8_adc_battery_current_acc = 0;
static uint16_t ui16_adc_motor_phase_current = 0; // mstrens: it was uint8 in original code

// ADC Values
volatile uint16_t ui16_adc_voltage;
volatile uint16_t ui16_adc_torque;
volatile uint16_t ui16_adc_throttle;

// brakes
volatile uint8_t ui8_brake_state = 0;

// cadence sensor
#define NO_PAS_REF 5
volatile uint16_t ui16_cadence_sensor_ticks = 0;
static uint16_t ui16_cadence_sensor_ticks_counter_min = CADENCE_SENSOR_CALC_COUNTER_MIN;
static uint8_t ui8_pas_state_old = 4;
static uint16_t ui16_cadence_calc_counter, ui16_cadence_stop_counter;
static uint8_t ui8_cadence_calc_ref_state = NO_PAS_REF;
const static uint8_t ui8_pas_old_valid_state[4] = { 0x01, 0x03, 0x00, 0x02 };

// wheel speed sensor
volatile uint16_t ui16_wheel_speed_sensor_ticks = 0;
volatile uint16_t ui16_wheel_speed_sensor_ticks_counter_min = 0;

// battery soc
volatile uint8_t ui8_battery_SOC_saved_flag = 0;
volatile uint8_t ui8_battery_SOC_reset_flag = 0;

// Last rotor complete revolution Hall ticks
static uint16_t ui16_hall_360_ticks;

// Last Hall sensor state
static uint8_t  previous_hall_pattern = 7; // Invalid value, force execution of Hall code at the first run

// Hall counter value of last Hall transition 
uint16_t previous_360_ref_ticks ; 

// ----------   end of copy from tsdz2 -------------------------

uint16_t speed_counter = SPEED_COUNTER_MAX ;  
uint32_t angle_deg = ANGLE_INIT;      // for debug with slow rotation for hall pattern discovery
//uint8_t angle_256 = 0; // angle in range 0/255 (represent 0/360°)

uint8_t ui8_temp;
uint16_t ui16_temp;

volatile uint16_t ui16_a = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_b = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
volatile uint16_t ui16_c = PWM_COUNTER_MAX / 2 ;//   840 in tsdz8   // 4*210 from tsdz2
uint8_t ui8_g_duty_cycle_test = DUTY_CYCLE_TEST;
volatile uint16_t svm_up_counter = 0; // to debug
volatile uint16_t svm_down_counter = 0; // to debug

uint16_t ui16_a_min = 0XFFFF; // to debug
uint16_t ui16_a_max = 0X0000; // to debug
// for printing (debug)
uint8_t ui8_svm_table_index_print  ; 
uint8_t ui16_temp_print  ;
uint16_t ui16_a_print ;
uint16_t ui16_new_angle_print; 
volatile uint32_t hall_print_pos = 0; //save the angle to let it be printed with hall position
volatile uint32_t hall_print_angle = 0; // save the angle to let it be printed with hall position
volatile uint32_t posif_print_current_pattern = 0; 
volatile uint32_t hall_print_interval = 0; 
volatile uint32_t posif_SR0 = 0; // count hall valid transition 

volatile uint32_t posif_SR1 = 0; // count hall transition (before check if valid or not)
volatile bool new_hall = false;

volatile uint32_t hall_pin_pos = 0;
uint16_t debug_time_ccu8_irq0 = 0;
//************ for calibration ***********************
#define CALIBRATE_OFFSET_STEP 1      // step used when increasing the calibrated_offset_angle (normally 1; could be set to 0 for some kind of test)
#if (CALIBRATE_HALL_SENSORS == (1))
uint8_t calibrated_offset_angle = 17 ; // This is the first value used for calibration; it increases every 4 sec
uint8_t calibrated_offset_angle_prev ; // used to detect an increase and so calculate current average and offset providing the min current
#else // not ALIBRATE_HALL_SENSORS
uint8_t calibrated_offset_angle = 28 ; // Put here the value found with the hall sensor calibration process 
#endif


extern uint32_t debug_values[20] ; // for debug with print at interval
extern uint32_t first_debug_values[100][20];
extern uint32_t first_debug_index ; 
uint16_t last_hall_pattern_change_ticks_prev = 0;

// for calibration process of the hall sensor offsets
uint32_t current_accumulated = 0;
uint16_t current_accumulated_counter = 0;
uint16_t current_average = 0;
uint16_t current_min = 0xFFFF;
uint8_t calibrated_offset_angle_min = 0; // save the offset angle providing the lowest average current (during calibration)

// variables captured during the hall sensor irq
volatile uint32_t hall_pattern_irq;                   // current hall pattern
volatile uint16_t hall_pattern_change_ticks_irq; // ticks from ccu4 slice 3 for last pattern change

// When a hall pattern transition occurs (good or wrong) ; this occurs after the delay for sampling
void CCU40_1_IRQHandler(){ // when a transition occurs, CCU4 performs a Serice request 1   // __RAM_FUNC 
    // get the timer from CCU4 slice 3 running
    hall_pattern_change_ticks_irq = XMC_CCU4_SLICE_GetTimerValue(RUNNING_250KH_TIMER_HW) ;
    // read the hall pattern
    hall_pattern_irq = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
    hall_pattern_irq |=  XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN) << 1;
    hall_pattern_irq |=  XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN) << 2;
}


/*
// When a correct transition occurs
void CCU40_0_IRQHandler(){ // when a correct transition occurs, CCU4 performs a capture + clear and an Serice request 0
    // we have to save the capture register and to upload the posif shadow register for next transition.
    uint32_t capture = XMC_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW, (0U) );
    hall_print_interval = capture & 0xFFFF ;  // bits 0/15 = the value
    uint32_t prescaler = (capture>>16) & 0X0F ; // bits 16/19 = prescaler
    uint32_t FFL = (capture>>20) & 0X01 ; // bit 20 : 1 = new value
    uint32_t current_pins = getHallPosition();
    uint8_t current =  XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    uint8_t expected = XMC_POSIF_HSC_GetExpectedPattern(HALL_POSIF_HW);
    printf("Entering CCU4 SR0= %ld pins=%ld Cur=%ld Exp=%ld Interval=%ld pre=%ld ffl=%ld\r\n",
                        posif_SR0, current_pins, (uint32_t) current, (uint32_t) expected, hall_print_interval, prescaler, FFL);
    
        
    update_shadow_pattern(expected);
    posif_print_current_pattern = XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    posif_SR0++;
    current_pins = getHallPosition();
    current =  XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    expected = XMC_POSIF_HSC_GetExpectedPattern(HALL_POSIF_HW);
    printf("End CCU4 SR0= %ld pins=%ld Cur=%ld Exp=%ld\r\n", posif_SR0, current_pins, (uint32_t) current, (uint32_t) expected);
    
}
*/
/* // not used currently - Service request 1 is used to capture hall pattern and timestamp when a transition occurs
// could be used to detect that motor is not running because timer reached the period 
void CCU40_1_IRQHandler(){
    posif_SR1++;
    uint8_t current =  XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    uint8_t expected = XMC_POSIF_HSC_GetExpectedPattern(HALL_POSIF_HW);
    uint32_t current_pins = getHallPosition();
    printf("CCU4 SR1= %ld pins=%ld Cur=%ld Exp=%ld\r\n", posif_SR1, current_pins, (uint32_t) current, (uint32_t) expected);
}
*/

/*
void POSIF0_1_IRQHandler(){ // to debug; 
    posif_SR1++;
    uint8_t current =  XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    uint8_t expected = XMC_POSIF_HSC_GetExpectedPattern(HALL_POSIF_HW);
    uint32_t current_pins = getHallPosition();

    printf("SR1= %ld pins=%ld Cur=%ld Exp=%ld\r\n", posif_SR1, current_pins, (uint32_t) current, (uint32_t) expected);
    //uint32_t current_pos = getHallPosition();
    //update_shadow_pattern(current_pos);
    //XMC_POSIF_HSC_UpdateHallPattern(HALL_POSIF_HW); // 
    //posif_print_current_pattern = XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
}
*/
uint8_t current_hall_pattern = 1;
uint8_t current_hall_pattern_prev = 1;
uint32_t capture_accumulated = 0 ;

// added by mstrens to take care of error in the position of the hall sensor
// see the calibration process in doc.txt
#define CALIBRATED_HALL_ANGLE_PATTERN_1 0  // 90°
#define CALIBRATED_HALL_ANGLE_PATTERN_2 -3 // 210
#define CALIBRATED_HALL_ANGLE_PATTERN_3 -1 // 150
#define CALIBRATED_HALL_ANGLE_PATTERN_4 0  // 330
#define CALIBRATED_HALL_ANGLE_PATTERN_5 -2 // 30
#define CALIBRATED_HALL_ANGLE_PATTERN_6 -1 // 270

#if (CALIBRATE_HALL_SENSORS == 1)
volatile uint8_t ui8_hall_ref_angles[8] = { // for each angle, we substract 64 wich is in fact the value for 90°
        0,                     // error ; index must be between 1 and 6
        PHASE_ROTOR_ANGLE_90 , // 64    hall pattern 1
        PHASE_ROTOR_ANGLE_210, // 149   hall pattern 2
        PHASE_ROTOR_ANGLE_150, // 107   hall pattern 3
        PHASE_ROTOR_ANGLE_330, // 235   hall patern 4
        PHASE_ROTOR_ANGLE_30 , // 21    hall pattern 5
        PHASE_ROTOR_ANGLE_270, // 192   hall pattern 6
        0                     // error ; index must be between 1 and 6
};
#else // not ALIBRATE_HALL_SENSORS
volatile uint8_t ui8_hall_ref_angles[8] = { // for each angle, we substract 64 wich is in fact the value for 90°
        0,                     // error ; index must be between 1 and 6
        PHASE_ROTOR_ANGLE_90 + CALIBRATED_HALL_ANGLE_PATTERN_1, // 151-64 = 87 : hall pattern 1
        PHASE_ROTOR_ANGLE_210 + CALIBRATED_HALL_ANGLE_PATTERN_2, // 2              hall pattern 2
        PHASE_ROTOR_ANGLE_150 + CALIBRATED_HALL_ANGLE_PATTERN_3, // 109-64 = 45   hall pattern 3
        PHASE_ROTOR_ANGLE_330 + CALIBRATED_HALL_ANGLE_PATTERN_4, // 237 - 64 = 173 hall patern 4
        PHASE_ROTOR_ANGLE_30 + CALIBRATED_HALL_ANGLE_PATTERN_5, // 194 -64 = 130 hall pattern 5
        PHASE_ROTOR_ANGLE_270 + CALIBRATED_HALL_ANGLE_PATTERN_6, // -40            hall pattern 6
        0                     // error ; index must be between 1 and 6
};
#endif
/* // this is the first table that was adapted from tsdz2 but that generates angles in the wrong direction
volatile uint8_t ui8_hall_ref_angles[8] = { // for each angle, we substract 90 wich is in fact the value for 90°
        0,                     // error ; index must be between 1 and 6
        PHASE_ROTOR_ANGLE_210, // 151-64 = 87 : hall pattern 1
        PHASE_ROTOR_ANGLE_90, // 2              hall pattern 2
        PHASE_ROTOR_ANGLE_150, // 109-64 = 45   hall pattern 3
        PHASE_ROTOR_ANGLE_330, // 237 - 64 = 173 hall patern 4
        PHASE_ROTOR_ANGLE_270, // 194 -64 = 130 hall pattern 5
        PHASE_ROTOR_ANGLE_30, // -40            hall pattern 6
        0                     // error ; index must be between 1 and 6
};
*/
/*    for TSDZ2
volatile uint8_t ui8_hall_counter_offsets[8] = {
    0,
    HALL_COUNTER_OFFSET_DOWN, // 23  //210°
    HALL_COUNTER_OFFSET_DOWN, // 23  // 90_
    HALL_COUNTER_OFFSET_UP, //44     // 150°
    HALL_COUNTER_OFFSET_DOWN, // 23  // 330°
    HALL_COUNTER_OFFSET_UP, //44     // 270°
    HALL_COUNTER_OFFSET_UP, //44     // 30°
    0
};
*/

// this is in theory the best table
volatile uint8_t ui8_hall_counter_offsets[8] = {
    0,
    HALL_COUNTER_OFFSET_DOWN , // 23  //210° when Hall pattern = 1
    HALL_COUNTER_OFFSET_DOWN , // 23  // 90_    when Hall pattern = 2
    HALL_COUNTER_OFFSET_UP , //44     // 150° when Hall pattern = 3
    HALL_COUNTER_OFFSET_DOWN , // 23  // 330°   when Hall pattern = 4
    HALL_COUNTER_OFFSET_UP, //44     // 270°  when Hall pattern = 5
    HALL_COUNTER_OFFSET_UP , //44     // 30°  when Hall pattern = 6
    0
};

/*
volatile uint8_t ui8_hall_counter_offsets[8] = {
    0,
    33 , // 23  //210° when Hall pattern = 1
    33 , // 23  // 90_    when Hall pattern = 2
    33 , //44     // 150° when Hall pattern = 3
    33 , // 23  // 330°   when Hall pattern = 4
    33 , //44     // 270°  when Hall pattern = 5
    33 , //44     // 30°  when Hall pattern = 6
    0
};
*/

/* orignal values from tsdz2 where there is first a conversion of hall patern to an index
// phase angle for rotor positions 30, 90, 150, 210, 270, 330 degrees
volatile uint8_t ui8_hall_ref_angles[6] = { // for each angle, we substract 90 wich is in fact the value for 90°
		PHASE_ROTOR_ANGLE_30, // -40             // up
		PHASE_ROTOR_ANGLE_90, // 2               // down
		PHASE_ROTOR_ANGLE_150, // 109-64 = 45    // up
		PHASE_ROTOR_ANGLE_210, // 151-64 = 87    // down
		PHASE_ROTOR_ANGLE_270, // 194 -64 = 130  // up
		PHASE_ROTOR_ANGLE_330}; // 237 - 64 = 173// down

// Hall counter offset for states 6,2,3,1,5,4 (value configured from Android App)
volatile uint8_t ui8_hall_counter_offsets[6] = {
        HALL_COUNTER_OFFSET_UP, //44
        HALL_COUNTER_OFFSET_DOWN, // 23
        HALL_COUNTER_OFFSET_UP, // 44
        HALL_COUNTER_OFFSET_DOWN, // 23
        HALL_COUNTER_OFFSET_UP, //44
        HALL_COUNTER_OFFSET_DOWN}; //23
*/

// Hall offset for current Hall state
static uint8_t ui8_hall_counter_offset;


// for calibration
uint32_t hall_pattern_error_counter = 0;
uint32_t hall_pattern_valid_counter = 0;
uint32_t calibrated_offset_increase_counter = 0;
uint16_t last_hall_pattern_change_ticks_prev ;
uint16_t hall_pattern_intervals[8] = {0};

#if (AUTOMATIC_ROTATION == 1) 
// this code is just to generate a rotation of the magnetic field at a fixed speed (not taking into account the hall sensors)
void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
// here we just calculate the new compare values used for the 3 slices (0,1,2) that generates the 3 PWM
// timer updates occurs in the counting down interrupt

// timer runs at 2*1680/64000000 *840 => 52,5usec
// if counter = 2, we increase the angle once evey 105 usec
// to get 1 mecanical rotation, we need 105 * 256° * 4poles = 107520 usec 
// In 1 sec we have 1/107520 * 1000000 = 9,300595238 rotation per sec = 558 rotations/ min
// if counter = 10 => rpm = 111
// if counter = 20 => rpm = 56
    // get hall position to put the info in a uart message
    hall_pin_pos = hall_pattern_irq; // filled in irq generated by Service request 1 from CCU4 slice 0 generated when POSIF detects a change (good or wrong)
    svm_up_counter++; //just for debugging
    speed_counter--;
    if (speed_counter == 0){ // once evey x cycles (x * 52,5 usec)
        speed_counter = SPEED_COUNTER_MAX;
        if (hall_pin_pos != hall_print_pos){
            new_hall = true; // trigger that position must be printed in main loop
        }
        hall_print_pos = hall_pin_pos; // save the pattern based on pins
        hall_print_angle = angle_deg; // save the angle to let it be printed with hall position
        
        // update the PWM values
        angle_deg = angle_deg + 1;  // increase by 1 degree
        if (angle_deg >= 360) angle_deg = ANGLE_INIT;
        ui16_new_angle_print = angle_deg ;
        uint8_t ui8_svm_table_index = (uint8_t) ((((uint16_t)angle_deg) << 8) / 360); // convert degree 360 to index up to 256
        ui8_svm_table_index_print = ui8_svm_table_index ; 
        // Phase A is advanced 240 degrees over phase B
        ui16_temp = ui16_svm_table[(uint8_t) (ui8_svm_table_index + 171)]; // 171 = 240 deg when 360° is coded as 256
        ui16_temp_print = ui16_temp ;
        //next ui16_temp varies between 0 and 428
        if (ui16_temp > MIDDLE_SVM_TABLE) { // 214 at 19 khz
            ui16_a = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle_test)>>8);
            ui16_a_print = ui16_a ;
            //e.g.  temp = 157; duty cycle = 50% = 128 (duty_cycle 256 = 100%)
            //(157-107) * 128 = 50 *128
            //(107 + (50*128/256)) *2 = (107 + 25)*2 = 132*2 // Why *2?????
            // Compare values in timer can be between 0 and 420
            // max in table is 215; so compare value is nearly max 430. This is more than 420
            // I expect that duty_cycle is never 100%

        } else {
            ui16_a = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle_test)>>8);
        }
        // phase B as reference phase
        ui16_temp = ui16_svm_table[ui8_svm_table_index];
        if (ui16_temp > MIDDLE_SVM_TABLE) {
            ui16_b = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle_test)>>8);
        } else {
            ui16_b = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle_test)>>8);
        }

        // phase C is advanced 120 degrees over phase B
        ui16_temp = ui16_svm_table[(uint8_t) (ui8_svm_table_index + 85 )] ; // 85 = 120 deg
        if (ui16_temp > MIDDLE_SVM_TABLE) {
            ui16_c = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle_test)>>8);
        } else {
            ui16_c = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle_test)>>8);
        }
        // ui16_a, b, c = the values to be filled in pwm timers
    }
}
#else // here the code for production   __RAM_FUNC    
void CCU80_0_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting UP (= 1/4 of 19mhz cycles with 1680 ticks at 64mHz and centered aligned)
// here we just calculate the new compare values used for the 3 slices (0,1,2) that generates the 3 PWM
    //XMC_GPIO_SetOutputHigh(OUT_LIGHT_PORT,OUT_LIGHT_PIN); // to check the time required by this interrupt
    // get and save values from the interrupt (when hall pattern changed)
    uint32_t start_ticks = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    uint8_t current_hall_pattern =  (uint8_t) hall_pattern_irq & 0x07 ;  // hall pattern when last hall change occured
    uint16_t last_hall_pattern_change_ticks = hall_pattern_change_ticks_irq ;  // ticks at this change
    uint16_t current_ticks = XMC_CCU4_SLICE_GetTimerValue(RUNNING_250KH_TIMER_HW) ; // ticks now
    XMC_ExitCriticalSection(critical_section_value);
    uint16_t enlapsed_time =  current_ticks - last_hall_pattern_change_ticks ; // ticks between now and last change
    
    // when pattern change
    if ( current_hall_pattern != previous_hall_pattern) {
        if (current_hall_pattern != expected_pattern_table[previous_hall_pattern]){ // new pattern is not the expected one
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0x00
            ui8_hall_360_ref_valid = 0;  // reset the indicator saying no error for a 360° electric rotation 
            #if (CALIBRATE_HALL_SENSORS == (1))
            hall_pattern_error_counter++;
            #endif 
        } else {   // valid transition
            if (current_hall_pattern ==  0x01) {  // rotor at 210°
                if (ui8_hall_360_ref_valid) { // check that we have a full rotation without pattern sequence error
                        ui16_hall_counter_total = last_hall_pattern_change_ticks - previous_360_ref_ticks; // save the total number of tick for one electric rotation
                        ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES; // 0x80 ; it says that we can interpolate because speed is known
                }
                ui8_hall_360_ref_valid = 0x01;
                previous_360_ref_ticks = last_hall_pattern_change_ticks ;    
            } else if (current_hall_pattern ==  0x03) {  // rotor at 150°){
                // update ui8_g_foc_angle once every ERPS (used for g_foc_angle calculation) ;
                            ui8_foc_flag = 1;
            }
            #if (CALIBRATE_HALL_SENSORS == (1))
            hall_pattern_valid_counter++;     // count the valid frame
            hall_pattern_intervals[current_hall_pattern]  = last_hall_pattern_change_ticks - last_hall_pattern_change_ticks_prev; // save the interval between 2 changes
            last_hall_pattern_change_ticks_prev = last_hall_pattern_change_ticks; 
            #endif
        }
        previous_hall_pattern = current_hall_pattern; // saved for future check here
        // set rotor angle based on hall patern
        ui8_motor_phase_absolute_angle = ui8_hall_ref_angles[current_hall_pattern]; 
        // set hall counter offset for rotor interpolation based on current hall state
        ui8_hall_counter_offset = ui8_hall_counter_offsets[current_hall_pattern];
        #if (DEBUG_HALL_SENSOR == 1)
        if ( first_debug_index < 100) { // for debug we save the first 100 changes
            first_debug_values[first_debug_index][0] = current_ticks ;
            first_debug_values[first_debug_index][1] = (uint32_t) current_ticks - (uint32_t) last_hall_pattern_change_ticks_prev ;
            first_debug_values[first_debug_index][2] = current_hall_pattern ;
            first_debug_values[first_debug_index][3] = ui8_motor_phase_absolute_angle ;
            first_debug_values[first_debug_index][4] = enlapsed_time ;
            first_debug_values[first_debug_index][5] = ui8_controller_duty_cycle_target;
            first_debug_values[first_debug_index][6] =  ui8_g_duty_cycle;
            first_debug_values[first_debug_index][7] = ui8_controller_adc_battery_current_target ; 
            first_debug_values[first_debug_index][8] = ui8_adc_battery_current_filtered; 
            first_debug_values[first_debug_index][9] = hall_pattern_error_counter;
            first_debug_values[first_debug_index][10] = hall_pattern_valid_counter;
            first_debug_values[first_debug_index][11] = ui8_motor_commutation_type;
            first_debug_values[first_debug_index][12] = ui16_hall_counter_total;
            first_debug_values[first_debug_index][13] = calibrated_offset_angle;
            first_debug_values[first_debug_index][14] = ui8_hall_360_ref_valid;
            first_debug_index++;
        }
        #endif
        #if (CALIBRATE_HALL_SENSORS == (1))
        last_hall_pattern_change_ticks_prev = last_hall_pattern_change_ticks;
        #endif 
    } else { // no hall patern change
        // Verify if rotor stopped (< 10 ERPS)
        if (enlapsed_time > (HALL_COUNTER_FREQ/MOTOR_ROTOR_INTERPOLATION_MIN_ERPS/6)) { // 250000/10 /6 = 4166
            ui8_motor_commutation_type = BLOCK_COMMUTATION; // 0
            ui8_g_foc_angle = 0;
            ui8_hall_360_ref_valid = 0;
            ui16_hall_counter_total = 0xffff;
        }

    }
    // to calibrate hall sensor offsets:  increase an angle offset to see when motor runs with the lowest current
    #if (CALIBRATE_HALL_SENSORS == 1)
    // measure the current and search the min; save the offset that provides the  min current
    if ( calibrated_offset_angle != calibrated_offset_angle_prev){
        calibrated_offset_angle_prev = calibrated_offset_angle;
        current_average = current_accumulated / current_accumulated_counter;
        current_accumulated = 0;
        current_accumulated_counter = 0;
        if (current_average < current_min){
            current_min= current_average;
            calibrated_offset_angle_min = calibrated_offset_angle;
        }

    } else {
        // sum the 2 currents
        current_accumulated += (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 8 ) & 0xFFFF) +
                                 (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 12 ) & 0xFFFF);
        current_accumulated_counter += 2;
    }
    if ((system_ticks - calibrated_offset_increase_counter)> 4000){ // increases every 4000 msec
            calibrated_offset_increase_counter = system_ticks;
            calibrated_offset_angle += CALIBRATE_OFFSET_STEP; 
    }    
    #endif
    /****************************************************************************/
    // - calculate interpolation angle and sine wave table index when spped is known
    uint8_t ui8_interpolation_angle = 0; // interpolation angle
    uint32_t compensated_enlapsed_time; 
    if (ui8_motor_commutation_type != BLOCK_COMMUTATION) {  // as long as hall patern are OK and motor is running 
        // ---------
        // uint8_t ui16_temp = ((uint32_t)ui16_a << 8) / ui16_hall_counter_total; // ui16_hall_counter_total is the number of ticks for a full cycle
        // temp is here related to 256 (because 256 represent 360°); So 36° is here 25
        compensated_enlapsed_time = enlapsed_time + ui8_fw_hall_counter_offset + ui8_hall_counter_offset;
        // convert time tick to angle (256 = 360°)
        // to do : use perhaps the match coprocessor to perform the division in less than 1 usec
        ui8_interpolation_angle = (((uint32_t) compensated_enlapsed_time) << 8) /  ui16_hall_counter_total; // <<8 = 256 = 360 electric angle
        if (ui8_interpolation_angle > 42){  // added by mstrens because interpolation should not exceed 60°
            ui8_interpolation_angle = 0;
        }
        /*
        // ---------
        // Avoid to use the slow _divulong library function.
        // Faster implementation of the above operation based on the following assumptions:
        // 1) ui16_a < 8192 (only 13 of 16 significants bits)
        // 2) LSB of (ui16_a << 8) is obviously 0x00
        // 3) The result should be less than 60 degrees. Use 180 deg (value of 128) to be safe.
        uint8_t ui8_cnt = 7; //max 6 loops: result < 128
        // Add Field Weakening counter offset (fw angle increases with rotor speed)
        // note: ui16_a - ui16_b = Hall counter ticks from the last Hall sensor transition;
        ui16_a = ((uint8_t)(ui8_fw_hall_counter_offset + ui8_hall_counter_offset) + (ui16_a - ui16_b)) << 1;
        // here ui16_a is the hall counter ticks since last hall sensor change but with some offset corrections (*2?)
        do {
            ui16_a <<= 1;
            ui8_temp <<= 1;
            if (ui16_hall_counter_total <= ui16_a) {
                ui16_a -= ui16_hall_counter_total;
                ui8_temp |= (uint8_t)0x01;
            }
        } while (--ui8_cnt);
        */
    }
    uint8_t ui8_svm_table_index = ui8_interpolation_angle + ui8_motor_phase_absolute_angle + ui8_g_foc_angle + calibrated_offset_angle;
    
    // Phase A is advanced 240 degrees over phase B
    ui16_temp = ui16_svm_table[(uint8_t) (ui8_svm_table_index + 171)]; // 171 = 240 deg when 360° is coded as 256
    if (ui16_temp > MIDDLE_SVM_TABLE) { // 214 at 19 khz
        ui16_a = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8); // >>8 because duty_cycle 100% is 256
        //ui16_a_print = ui16_a ;
    } else {
        ui16_a = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }
    
    // phase B as reference phase
    ui16_temp = ui16_svm_table[ui8_svm_table_index] ;
    if (ui16_temp > MIDDLE_SVM_TABLE) {
        ui16_b = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8);
    } else {
        ui16_b = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }

    // phase C is advanced 120 degrees over phase B
    ui16_temp = ui16_svm_table[(uint8_t) (ui8_svm_table_index + 85 )] ; // 85 = 120 deg
    if (ui16_temp > MIDDLE_SVM_TABLE) {
        ui16_c = MIDDLE_SVM_TABLE + (((ui16_temp - MIDDLE_SVM_TABLE) * (uint16_t) ui8_g_duty_cycle)>>8);
    } else {
        ui16_c = MIDDLE_SVM_TABLE - (((MIDDLE_SVM_TABLE - ui16_temp) * (uint16_t) ui8_g_duty_cycle)>>8);
    }
    #if (CALIBRATE_HALL_SENSORS == (1))
    debug_values[0] =  ui8_svm_table_index;
    debug_values[1] =  ui8_g_duty_cycle;
    debug_values[2] =  ui8_controller_adc_battery_current_target;
    debug_values[3] =  ui8_adc_battery_current_filtered;
    debug_values[4] =  ui8_motor_commutation_type;
    debug_values[5] = ui8_interpolation_angle ;
    debug_values[6] = enlapsed_time;
    debug_values[7] = compensated_enlapsed_time;
    debug_values[8] = ui16_hall_counter_total;
    debug_values[9] = hall_pattern_error_counter;
    debug_values[10] = hall_pattern_valid_counter;
    debug_values[11] = ui8_controller_duty_cycle_target;
    debug_values[12] = ui8_controller_adc_battery_current_target ;
    debug_values[13] = calibrated_offset_angle;
    #endif // end of CALIBRATE_HALL_SENSORS
    //XMC_GPIO_SetOutputLow(OUT_LIGHT_PORT,OUT_LIGHT_PIN); // to check the time required by this interrupt
    
    uint32_t temp  = (uint32_t) XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW) ;
    if (temp > start_ticks)
     {temp = temp - start_ticks;} else {temp = 0;}
    if (temp > debug_time_ccu8_irq0) debug_time_ccu8_irq0 = (uint16_t) temp; // store the max enlapsed time in the irq
} // end of CCU80_0_IRQHandler

#endif // end of code for production

void CCU80_1_IRQHandler(){ // called when ccu8 Slice 3 reaches 840  counting DOWN (= 1/4 of 19mhz cycles)   __RAM_FUNC 
    //svm_down_counter++; // just for debugging
    //if (ui16_a < ui16_a_min ) ui16_a_min  = ui16_a ;
    //if (ui16_a > ui16_a_max ) ui16_a_max  = ui16_a ;
    
    uint32_t critical_section_value = XMC_EnterCriticalSection();
    // fill the PWM parameters with the values calculated in the other CCU8 interrupt
    XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_U_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_a);
    XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_V_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_b);
    XMC_CCU8_SLICE_SetTimerCompareMatch(PHASE_W_TIMER_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1 , ui16_c);
    /* Enable shadow transfer for slice 0,1,2 for CCU80 Kernel */
	XMC_CCU8_EnableShadowTransfer(ccu8_0_HW, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
	                                            (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 ));
    XMC_ExitCriticalSection(critical_section_value);

    /****************************************************************************/
        // Read all ADC values (right aligned values).
        // No overrun errors can occurs here because the conversion is started at the beginning
        // of the PWM up interrupt and in this position is already ended.
        /*
        vadc22 = XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 2 ); // torque gr0 ch7 result 2 in bg p2.2
        vadc23 = XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 3 ); // unknown gr1 ch5 result 3      p2.3
        vadc24 = XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ); // battery gr1 ch6 result 4   in bg  p2.4
        vadc25 = XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ); // throttle gr1 ch7 result 5  in bg  p2.5
        vadc26 = XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 6 ); // VCC      gr0 ch0 result 6  in bg  p2.§
        vadc27 = XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 7 ); // unknown  gr1 ch1 result 7         p2.7
        vadc28 = XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 8 ); // current  gr0 ch1 result 8 in queue 0 p2.8
        vadc29 = XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 9 ); // cur U    gr0 ch2 result 9  in bg     p2.9
        vadc210 = XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 10 );// curV    gr0 ch3 result 10 in bg     p2.10
        vadc211 = XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 11 );// curW    gr0 ch4 result 11 in bg     p2.11
        
        vadc28bis = XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 12 ); // cur bis gr1 ch0 result 12 in queue 1 p2.8
        vadc29bis = XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 13 ); // cur U bis gr1 ch4 result 13          p2.9
        vadc210bis = XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 14 );// cur V bis gr1 ch2 result 14          p2.10
        vadc211bis = XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 15 );// cur W bis gr1 ch3 result 15          p2.11
        */
       // adc values are reduced to 10 bits instead of 12 bits to use the same resolution as tsdz2
       // note: per vadc group, the result number is the same as the pin number (except for group 1 current sensor)
        ui16_adc_voltage  = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 4 ) & 0xFFFF) >> 2; // battery gr1 ch6 result 4   in bg
        ui16_adc_torque   = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 2 ) & 0xFFFF) >> 2; // torque gr0 ch7 result 2 in bg p2.2
        ui16_adc_throttle = (XMC_VADC_GROUP_GetResult(vadc_0_group_1_HW , 5 ) & 0xFFFF) >> 2; // throttle gr1 ch7 result 5  in bg  p2.5
        
        // current is in 8 bits instead of 12 bits
        ui8_temp = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 8 ) & 0xFFFF) >> 4 ; // current  gr0 ch1 result 8 in queue 0 p2.8 ; from 12 bits to 8 bits
        // to test with other measurement which one is best
        //ui8_temp = (XMC_VADC_GROUP_GetResult(vadc_0_group_0_HW , 8 ) & 0FFFF ) >> 4 ; // current  gr0 ch1 result 8 in queue 0 p2.8
        ui8_adc_battery_current_acc = (uint8_t)(ui8_temp >> 1) + (ui8_adc_battery_current_acc>>1);
        ui8_adc_battery_current_filtered = (uint8_t)(ui8_adc_battery_current_acc >> 1) + (ui8_adc_battery_current_filtered >> 1);

        
        // update foc_angle once per electric rotation (based on fog_flag
        // foc_angle is added to the position given by hall sensor + interpolation )
        if (ui8_g_duty_cycle > 0) {
            // calculate phase current.
            ui16_adc_motor_phase_current = (uint16_t)((uint16_t)((uint16_t)ui8_adc_battery_current_filtered << 8)) / ui8_g_duty_cycle;
            if (ui8_foc_flag) { // is set on 1 when rotor is at 150°
				ui8_adc_foc_angle_current = (ui8_adc_battery_current_filtered >> 1) + (ui16_adc_motor_phase_current >> 1);
                ui8_foc_flag = (uint16_t)(ui8_adc_foc_angle_current * ui8_foc_angle_multiplier) / 256; // multiplier = 39 for 48V tsdz2
                if (ui8_foc_flag > 13)
                    ui8_foc_flag = 13;
                ui8_foc_angle_accumulated = ui8_foc_angle_accumulated - (ui8_foc_angle_accumulated >> 4) + ui8_foc_flag;
                ui8_g_foc_angle = ui8_foc_angle_accumulated >> 4;
                ui8_foc_flag = 0;
            }
        } else {
            ui16_adc_motor_phase_current = 0;
            if (ui8_foc_flag) {
                ui8_foc_angle_accumulated = ui8_foc_angle_accumulated - (ui8_foc_angle_accumulated >> 4);
                ui8_g_foc_angle = ui8_foc_angle_accumulated >> 4;
                ui8_foc_flag = 0;
            }
        }

        // get brake state-
        ui8_brake_state = XMC_GPIO_GetInput(IN_BRAKE_PORT, IN_BRAKE_PIN) == 0; // 0 means that brake is on

                /****************************************************************************/
        // PWM duty_cycle controller:
        // - limit battery undervolt
        // - limit battery max current
        // - limit motor max phase current
        // - limit motor max ERPS
        // - ramp up/down PWM duty_cycle and/or field weakening angle value

        // check if to decrease, increase or maintain duty cycle
        //note:
        // ui8_adc_battery_current_filtered is calculated just here above
        // ui8_adc_motor_phase_current_max = 135 per default for TSDZ2 (13A *100/16) *187/112 = battery_current convert to ADC10bits *and ratio between adc max for phase and for battery
        //        is initiaded in void ebike_app_init(void) in ebyke_app.c
        
        
        // every 25ms ebike_app_controller fills
        //  - ui8_controller_adc_battery_current_target
        //  - ui8_controller_duty_cycle_target // is usually filled with 255 (= 100%)
        //  - ui8_controller_duty_cycle_ramp_up_inverse_step
        //  - ui8_controller_duty_cycle_ramp_down_inverse_step
        // Furthermore,  hen ebyke_app_controller start pwm, g_duty_cycle is first set on 30 (= 12%)

        if ((ui8_controller_duty_cycle_target < ui8_g_duty_cycle)                     // requested duty cycle is lower than actual
          || (ui8_controller_adc_battery_current_target < ui8_adc_battery_current_filtered)  // requested current is lower than actual
		  || (ui16_adc_motor_phase_current > ui8_adc_motor_phase_current_max)               // motor phase is to high
          || (ui16_hall_counter_total < (HALL_COUNTER_FREQ / MOTOR_OVER_SPEED_ERPS))        // Erps is to high
          || (ui16_adc_voltage < ui16_adc_voltage_cut_off)                                  // voltage is to low
          || (ui8_brake_state)) {                                                           // brake is ON
			
            // reset duty cycle ramp up counter (filter)
            ui8_counter_duty_cycle_ramp_up = 0;
			
            // ramp down duty cycle ;  after N iterations at 19 khz 
            if (++ui8_counter_duty_cycle_ramp_down > ui8_controller_duty_cycle_ramp_down_inverse_step) {
                ui8_counter_duty_cycle_ramp_down = 0;
                //  first decrement field weakening angle if set or duty cycle if not
                if (ui8_fw_hall_counter_offset > 0) {
                    ui8_fw_hall_counter_offset--;
                }
				else if (ui8_g_duty_cycle > 0) {
                    ui8_g_duty_cycle--;
				}
            }
        }
		else if ((ui8_controller_duty_cycle_target > ui8_g_duty_cycle)                     // requested duty cycle is higher than actual
          && (ui8_controller_adc_battery_current_target > ui8_adc_battery_current_filtered)) { //Requested current is higher than actual
			// reset duty cycle ramp down counter (filter)
            ui8_counter_duty_cycle_ramp_down = 0;

            // ramp up duty cycle
            if (++ui8_counter_duty_cycle_ramp_up > ui8_controller_duty_cycle_ramp_up_inverse_step) {
                ui8_counter_duty_cycle_ramp_up = 0;

                // increment duty cycle
                if (ui8_g_duty_cycle < PWM_DUTY_CYCLE_MAX) {
                    ui8_g_duty_cycle++;
                }
            }
        }
		else if ((ui8_field_weakening_enabled)
				&& (ui8_g_duty_cycle == PWM_DUTY_CYCLE_MAX)) {
            // reset duty cycle ramp down counter (filter)
            ui8_counter_duty_cycle_ramp_down = 0;

            if (++ui8_counter_duty_cycle_ramp_up > ui8_controller_duty_cycle_ramp_up_inverse_step) {
               ui8_counter_duty_cycle_ramp_up = 0;

               // increment field weakening angle
               if (ui8_fw_hall_counter_offset < ui8_fw_hall_counter_offset_max) {
                   ui8_fw_hall_counter_offset++;
			   }
            }
        }
		else {
            // duty cycle is where it needs to be so reset ramp counters (filter)
            ui8_counter_duty_cycle_ramp_up = 0;
            ui8_counter_duty_cycle_ramp_down = 0;
        }

        /****************************************************************************/
        // Wheel speed sensor detection
        // To be added

        /****************************************************************************/
        /*
         * - New pedal start/stop detection Algorithm (by MSpider65) -
         *
         * Pedal start/stop detection uses both transitions of both PAS sensors
         * ui8_temp stores the PAS1 and PAS2 state: bit0=PAS1,  bit1=PAS2
         * Pedal forward ui8_temp sequence is: 0x01 -> 0x00 -> 0x02 -> 0x03 -> 0x01
         * After a stop, the first forward transition is taken as reference transition
         * Following forward transition sets the cadence to 7RPM for immediate startup
         * Then, starting form the second reference transition, the cadence is calculated based on counter value
         * All transitions are a reference for the stop detection counter (4 time faster stop detection):
         */
        // to be added

        // original perform also a save of some parameters (battery consumption)
            // here ui16_a, b, c = the values to be filled in pwm timers
        // debug_values[5] = interpolation filled in first irq
}  // end of CCU8_1_IRQ

/*
 * This function will return the current state of the POSIF input pins to
 * which hall sensors are connected. This information is required before
 * starting the motor to know the start position of the motor.
 */
uint32_t getHallPosition(void)
{
  uint32_t hallposition;
  uint32_t hall[3] = { 0U };

  /*Read the input pins.*/
  hall[0] = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
  hall[1] = XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN);
  hallposition = (uint32_t)(hall[0] | ((uint32_t) hall[1] << 1U));

  hall[2] = XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN);
  hallposition |= ((uint32_t)(hall[2] << 2));


  return ((uint32_t)(hallposition & 0X07));
}

// is probably not required anymore because we use irq to capture the hall pattern changes just like tsdz2 (and not posif logic)
void posif_init_position(){
    uint8_t current_pos = getHallPosition(); // get current position
    while ((current_pos == 0 )|| (current_pos >6)){
        current_pos = getHallPosition(); // wait for a valid initial value; test shows that it starts wit 7
    }
    uint8_t next_pos = expected_pattern_table[current_pos];

    uint32_t current_pins;
    uint8_t current ;
    uint8_t expected ;
    current_pins = getHallPosition();
    current =  XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    expected = XMC_POSIF_HSC_GetExpectedPattern(HALL_POSIF_HW);
    printf("before update shadow: sR0= %ld pins=%ld Cur=%ld Exp=%ld\r\n", posif_SR0, current_pins , (uint32_t) current, (uint32_t) expected);

    update_shadow_pattern(current_pos); // update shadow register // shadow should be exp = 3 ,  current = 1

    current =  XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    expected = XMC_POSIF_HSC_GetExpectedPattern(HALL_POSIF_HW);
    printf("after update shadow: sR0= %ld pins=%ld Cur=%ld Exp=%ld\r\n", posif_SR0, current_pins , (uint32_t) current, (uint32_t) expected);

    XMC_POSIF_HSC_UpdateHallPattern(HALL_POSIF_HW); // upload shadow register in real register; Then shadow reg becomes 0 0
    current =  XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    expected = XMC_POSIF_HSC_GetExpectedPattern(HALL_POSIF_HW);
    printf("after update hallPattern: sR0= %ld pins=%ld Cur=%ld Exp=%ld\r\n", posif_SR0, current_pins , (uint32_t) current, (uint32_t) expected);

    update_shadow_pattern(next_pos); // prepare already shadow register for next change // shadow should be exp 2 , current 2
    current =  XMC_POSIF_HSC_GetCurrentPattern(HALL_POSIF_HW);
    expected = XMC_POSIF_HSC_GetExpectedPattern(HALL_POSIF_HW);
    printf("after second update shadow: sR0= %ld pins=%ld Cur=%ld Exp=%ld\r\n", posif_SR0, current_pins , (uint32_t) current, (uint32_t) expected);
}


// after a transition, shadow register is automatically copy in HALP (with current and expected)
// CCU4 slice1 generates a SR0 (at end of period) that is internally routed to MSET of posif
// When a correct transition happens, new values (next and current) must be uploaded in posif shadow register by the firmware
// When a correct transition happens, posif generates a signal on OUT1 that is mapped to event 0 on CCU4 slice 1
// CCU4 (event 0) then performs:
// - a capture of current timer
// - a start of timer (with clear and start)
// - a SR0 that firmware must use to know that a transition occured, a capture has been done, a new value must be filled in shadow register
// - it is not mandatory to use an interrupt nor to read the SR0 because it is possible to check in the 19 khz interrupt

void update_shadow_pattern(uint8_t current_pattern){
    if (current_pattern == 0 || current_pattern > 6) current_pattern = 1;
    XMC_POSIF_HSC_SetCurrentPattern(HALL_POSIF_HW, current_pattern);
    XMC_POSIF_HSC_SetExpectedPattern(HALL_POSIF_HW, expected_pattern_table[current_pattern]);
}



void motor_enable_pwm(void) { //set posif with current position & restart the timers
    
    uint32_t current_hall_pattern = 0;
    uint32_t expected_hall_pattern ; 
      /*Read the input pins.*/
    current_hall_pattern = XMC_GPIO_GetInput(IN_HALL0_PORT, IN_HALL0_PIN);// hall 0
    current_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL1_PORT, IN_HALL1_PIN) << 1;
    current_hall_pattern |=  XMC_GPIO_GetInput(IN_HALL2_PORT, IN_HALL2_PIN) << 2;
    // find expected
    expected_hall_pattern = expected_pattern_table[current_hall_pattern];
    // update shadow pattern
    XMC_POSIF_HSC_SetCurrentPattern(HALL_POSIF_HW, current_hall_pattern);
    XMC_POSIF_HSC_SetExpectedPattern(HALL_POSIF_HW, expected_hall_pattern);
    // transfert shadow register to active registers
    XMC_POSIF_HSC_UpdateHallPattern(HALL_POSIF_HW); // upload shadow register in real register; Then shadow reg becomes 0 0
    // load shadow registers with next values
    XMC_POSIF_HSC_SetCurrentPattern(HALL_POSIF_HW, expected_hall_pattern); // current becomes previous expected
    expected_hall_pattern = expected_pattern_table[expected_hall_pattern]; // get next expected
    XMC_POSIF_HSC_SetExpectedPattern(HALL_POSIF_HW, expected_hall_pattern);

    // one solution to activate is to generate an event that starts all timers in a synchronized way
    /* Enable Global Start Control CCU80  in a synchronized way*/
    XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
    // another way could be to set a flag to be used in the IRQ to force a PWM of 100% or 0% (to see wich one connect all LOW side fet)
}

void motor_disable_pwm(void) {
    // we stop and clear the 3 timers that control motor PWM
    XMC_CCU8_SLICE_StopClearTimer(PHASE_U_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_V_TIMER_HW);
    XMC_CCU8_SLICE_StopClearTimer(PHASE_W_TIMER_HW);
    // CCU8_3 is not stopped becauses it is required to manage some tasks (speed, torque,...) 
}





// calculate an average speed based on the capture values
// for each capture value for a given pattern:
// substract from a total the current value in an array with index = current pattern
// add the new value to this total
// store the new value in this array at the same index.
// multiply the total by the prescaler
// 

// there must be a way to check if the motor is locked
// in tsdz2 there is a test if speed is lees than X and current is more than X


// there must be also a protection for over current
// there could be 2 checks
// if current exceed a value (shortcut), we stop the motor immédiately
// if current exceed a value for more than X time, the motor stop