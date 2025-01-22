#include "cybsp.h"
#include "cy_utils.h"

#include "cy_retarget_io.h"

// function to get data from rxfifo and set them in rxBuffer; set a flag when a valid frame is received
// is called only when at least one byte is present in fifo
// UART
#define UART_RX_BUFFER_LEN   						7
#define RX_CHECK_CODE					(UART_RX_BUFFER_LEN - 1)															
#define UART_TX_BUFFER_LEN							9
#define TX_CHECK_CODE					(UART_TX_BUFFER_LEN - 1)
#define TX_STX										0x43
#define RX_STX										0x59
uint8_t ui8_state_machine = 0; // 0= not yet a start byte , 1 = accumulate 
uint32_t rxIdx = 0;
uint32_t ui8_rx_buffer[UART_RX_BUFFER_LEN];
uint8_t ui8_rx_counter = 0;
uint8_t ui8_received_package_flag = 0; // become 1 when buffer is filled with a frame
uint8_t ui8_tx_buffer[UART_TX_BUFFER_LEN];


void fillRxBuffer() {
	// Read data : Reading data clear UART2_FLAG_RXNE flag
	uint8_t	ui8_byte_received = (uint8_t) XMC_USIC_CH_RXFIFO_GetData(CYBSP_DEBUG_UART_HW);

    switch (ui8_state_machine) {
        case 0:
            // see if we get start package byte
            if (ui8_byte_received == RX_STX)  {
                ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
                ui8_rx_counter++;
                ui8_state_machine = 1;
            }
            else {
                ui8_rx_counter = 0;
                ui8_state_machine = 0;
            }
        break;

        case 1:
            ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
            ui8_rx_counter++;

            // see if is the last byte of the package
            if (ui8_rx_counter >= UART_RX_BUFFER_LEN) {
                ui8_rx_counter = 0;
                ui8_state_machine = 0;
                ui8_received_package_flag = 1; // signal that we have a full package to be processed
            }
        break;

        default:
        break;

    }
}



// assist level 
#define OFF											0
#define ECO											1
#define TOUR										2
#define SPORT										3
#define TURBO										4

// assist pedal level mask
#define ASSIST_PEDAL_LEVEL0							0x10
#define ASSIST_PEDAL_LEVEL01						0x80
#define ASSIST_PEDAL_LEVEL1							0x40
#define ASSIST_PEDAL_LEVEL2							0x02
#define ASSIST_PEDAL_LEVEL3							0x04
#define ASSIST_PEDAL_LEVEL4							0x08

// oem display fault & function code
#define CLEAR_DISPLAY								0
#define NO_FUNCTION									0
#define NO_FAULT									0
#define NO_ERROR                                  	0 

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


void processRxBuffer() {
	uint8_t ui8_i;
	uint8_t ui8_rx_check_code;
	uint8_t ui8_assist_level_mask;
	static uint8_t no_rx_counter = 0;
	static uint8_t ui8_lights_counter = 0;
	//static uint8_t ui8_walk_assist_debounce_flag = 0;
	static uint8_t ui8_walk_assist_debounce_counter = 0;
	static uint8_t ui8_walk_assist_button_pressed = 0;
	
    // normally from external
    static uint8_t ui8_lights_button_flag = 0;
    static uint8_t ui8_assist_level_01_flag = 0;
    static uint8_t ui8_assist_level = 0;

	// increment the comms safety counter
    no_rx_counter++;
	// increment walk assist counter
	ui8_walk_assist_debounce_counter++;
	// increment lights_counter
	ui8_lights_counter++;
	// increment display menu counter
//	ui8_menu_counter++;

	if (ui8_received_package_flag) {
		// verify check code of the package
		ui8_rx_check_code = 0x00;
							 
		for(ui8_i = 0; ui8_i < RX_CHECK_CODE; ui8_i++)
		{
			ui8_rx_check_code += ui8_rx_buffer[ui8_i];
		}

		// see if check code is ok...
		if (ui8_rx_check_code == ui8_rx_buffer[RX_CHECK_CODE]) {
			// Reset the safety counter when a valid message from the LCD is received
            no_rx_counter = 0;
			
			// mask lights button from display
			ui8_lights_button_flag = ui8_rx_buffer[1] & 0x01;
			
			// mask walk assist button from display
			ui8_walk_assist_button_pressed = ui8_rx_buffer[1] & 0x20;
			
			// mask assist level from display
			ui8_assist_level_mask = ui8_rx_buffer[1] & 0xDE; // mask: 11011110
			ui8_assist_level_01_flag = 0;
			
			// set assist level
			switch (ui8_assist_level_mask) {
				case ASSIST_PEDAL_LEVEL0: ui8_assist_level = OFF; break;
				case ASSIST_PEDAL_LEVEL01:   //  this bit is not used by all display; it seems to be level lower than 1
					ui8_assist_level = ECO;
					ui8_assist_level_01_flag = 1;
					break;
				case ASSIST_PEDAL_LEVEL1: ui8_assist_level = ECO; break;
				case ASSIST_PEDAL_LEVEL2: ui8_assist_level = TOUR; break;
				case ASSIST_PEDAL_LEVEL3: ui8_assist_level = SPORT; break;
				case ASSIST_PEDAL_LEVEL4: ui8_assist_level = TURBO; break;
			}
        }
    }    
}


// uart send
volatile uint8_t ui8_working_status = 0;
volatile uint8_t ui8_display_fault_code = 0;

uint8_t ui8_battery_state_of_charge = 0; // from 0 up to 9

static uint16_t ui16_adc_pedal_torque_offset_init = 0; // 150
static uint16_t ui16_adc_pedal_torque = 0;

uint16_t ui16_oem_wheel_speed_time = 100;

void fillTxBuffer()
{
	uint8_t ui8_i;
	uint8_t ui8_tx_check_code;
	
	
	// send the data to the LCD
	// start up byte
	ui8_tx_buffer[0] = TX_STX;

	// clear fault code
	ui8_display_fault_code = NO_FAULT;

	// initialize working status
	ui8_working_status &= 0xFE; // bit0 = 0 (battery normal)

	switch (ui8_battery_state_of_charge) {
		case 0:
			ui8_working_status |= 0x01; // bit0 = 1 (battery undervoltage)
			ui8_tx_buffer[1] = 0x00;
			break;
		case 1:
			ui8_tx_buffer[1] = 0x00; // Battery 0/6 (empty and blinking)
			break;
		case 2:
			ui8_tx_buffer[1] = 0x02; // Battery 1/6
			break;
		case 3:
			ui8_tx_buffer[1] = 0x04; // Battery 2/6
			break;
		case 4:
			ui8_tx_buffer[1] = 0x06; // Battery 3/6
			break;
		case 5:
			ui8_tx_buffer[1] = 0x08; // Battery 4/6
			break;
		case 6:
			ui8_tx_buffer[1] = 0x0A; // Battery 5/6
			break;
		case 7:
			ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (full)
			break;
		case 8:
			ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (soc reset)
			break;
		case 9:
			ui8_tx_buffer[1] = 0x0C; // Battery 6/6 (full)
			// E01 ERROR_OVERVOLTAGE
			ui8_display_fault_code = ERROR_OVERVOLTAGE; // Fault overvoltage
			break;
	}
	
	// reserved for VLCD5, torque sensor value TE and TE1
	ui8_tx_buffer[3] = ui16_adc_pedal_torque_offset_init;
	if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_offset_init) {
		ui8_tx_buffer[4] = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset_init;
	}
	else {
		ui8_tx_buffer[4] = 0;
	}
	

	ui8_display_fault_code = 1;
	ui8_tx_buffer[5] = ui8_display_fault_code;
	
	
	if (ui16_oem_wheel_speed_time) {
		#if ALTERNATIVE_MILES
		// in VLCD6 display the km/miles conversion is not present.
		// alternative mph for VLCD6 converts the sent speed time
		// applicable to other displays type, setting km/h on diplay
		// odometer history would remain in km, only those added would be in miles.
		ui16_data_value = (ui16_oem_wheel_speed_time * 16) / 10;
		ui8_tx_buffer[6] = (uint8_t) (ui16_data_value & 0xFF);
		ui8_tx_buffer[7] = (uint8_t) (ui16_data_value >> 8);
		#else
		// km/h or mph
		ui8_tx_buffer[6] = (uint8_t) (ui16_oem_wheel_speed_time & 0xFF);
		ui8_tx_buffer[7] = (uint8_t) (ui16_oem_wheel_speed_time >> 8);
		#endif
	}
	else {
		ui8_tx_buffer[6] = 0x07;
		ui8_tx_buffer[7] = 0x07;
	}
			
	// prepare check code of the package
	ui8_tx_check_code = 0x00;
	for(ui8_i = 0; ui8_i < TX_CHECK_CODE; ui8_i++)
	{
		ui8_tx_check_code += ui8_tx_buffer[ui8_i];
	}
	ui8_tx_buffer[TX_CHECK_CODE] = ui8_tx_check_code;

	
}
