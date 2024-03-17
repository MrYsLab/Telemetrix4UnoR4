/*
  Copyright (c) 2023 Alan Yorinks All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
  Version 3 as published by the Free Software Foundation; either
  or (at your option) any later version.
  This library is distributed in the hope that it will be useful,f
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
  along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


// This file is rather large, so it has been rearranged in logical sections.
// Here is the list of sections to help make it easier to locate items of interest,
// and aid when adding new features.

// 1. Feature Enabling Defines
// 2. Arduino ID
// 3. Client Command Related Defines and Support
// 4. Server Report Related Defines
// 5. i2c Related Defines
// 6. Pin Related Defines And Data Structures
// 7. Feature Related Defines, Data Structures and Storage Allocation
// 8. Command Functions
// 9. Scanning Inputs, Generating Reports And Running Steppers
// 10. Setup and Loop


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                    FEATURE ENABLING DEFINES                      */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// comment out this line to suppress the starting banner
#define ENABLE_STARTING_BANNER 1

// To disable a feature, comment out the desired enabling define or defines

// This will allow SPI support to be compiled into the sketch.
// Comment this out to save sketch space for the UNO
#define SPI_ENABLED 1

// This will allow OneWire support to be compiled into the sketch.
// Comment this out to save sketch space for the UNO
// #define ONE_WIRE_ENABLED 1

// This will allow DHT support to be compiled into the sketch.
// Comment this out to save sketch space for the UNO
#define DHT_ENABLED 1

// This will allow sonar support to be compiled into the sketch.
// Comment this out to save sketch space for the UNO
#define SONAR_ENABLED 1

// This will allow servo support to be compiled into the sketch.
// Comment this out to save sketch space for the UNO
#define SERVO_ENABLED 1

// This will allow stepper support to be compiled into the sketch.
// Comment this out to save sketch space for the UNO
// Accelstepper is currently not compatible with the UNO R4 WIFI
//#define STEPPERS_ENABLED 1

// This will allow I2C support to be compiled into the sketch.
// Comment this out to save sketch space for the UNO
#define I2C_ENABLED 1

// This allows LED matrix support to be compiled in the sketch.
// Comment this out to save sketch space for the UNO
#define LED_MATRIX_SUPPORTED 1

#include <Arduino.h>
#include "Telemetrix4UnoR4.h"

#ifdef SERVO_ENABLED

#include <Servo.h>

#endif

#ifdef SONAR_ENABLED

#include <NewPing.h>

#endif

#ifdef I2C_ENABLED

#include <Wire.h>

#endif

#ifdef DHT_ENABLED

#include <DHTStable.h>

#endif

#ifdef SPI_ENABLED

#include <SPI.h>

#endif

#ifdef ONE_WIRE_ENABLED

#include <OneWire.h>

#endif

#ifdef STEPPERS_ENABLED
#include <AccelStepper.h>
#endif

#ifdef LED_MATRIX_SUPPORTED

#include "Arduino_LED_Matrix.h"

#endif

// LED Matrix Support is based on
// https://github.com/ClemensAtElektor/Arduino-UNO-R4/blob/main/R4WiFi_led_matrix/R4WiFi_led_matrix.ino


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                    Arduino ID                      */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// This value must be the same as specified when instantiating the
// telemetrix client. The client defaults to a value of 1.
// This value is used for the client to auto-discover and to
// connect to a specific board regardless of the current com port
// it is currently connected to.

#define ARDUINO_ID 1


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*         Client Command Related Defines and Support               */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Commands Sent By The Client


// Add commands retaining the sequential numbering.
// The order of commands here must be maintained in the command_table.
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define ANALOG_WRITE 3
#define MODIFY_REPORTING 4  // mode(all, analog, or digital), pin, enable or disable
#define GET_FIRMWARE_VERSION 5
#define ARE_U_THERE 6
#define SERVO_ATTACH 7
#define SERVO_WRITE 8
#define SERVO_DETACH 9
#define I2C_BEGIN 10
#define I2C_READ 11
#define I2C_WRITE 12
#define SONAR_NEW 13
#define DHT_NEW 14
#define STOP_ALL_REPORTS 15
#define SET_ANALOG_SCANNING_INTERVAL 16
#define ENABLE_ALL_REPORTS 17
#define RESET 18
#define SPI_INIT 19
#define SPI_WRITE_BLOCKING 20
#define SPI_READ_BLOCKING 21
#define SPI_SET_FORMAT 22
#define SPI_CS_CONTROL 23
#define ONE_WIRE_INIT 24
#define ONE_WIRE_RESET 25
#define ONE_WIRE_SELECT 26
#define ONE_WIRE_SKIP 27
#define ONE_WIRE_WRITE 28
#define ONE_WIRE_READ 29
#define ONE_WIRE_RESET_SEARCH 30
#define ONE_WIRE_SEARCH 31
#define ONE_WIRE_CRC8 32
#define SET_PIN_MODE_STEPPER 33
#define STEPPER_MOVE_TO 34
#define STEPPER_MOVE 35
#define STEPPER_RUN 36
#define STEPPER_RUN_SPEED 37
#define STEPPER_SET_MAX_SPEED 38
#define STEPPER_SET_ACCELERATION 39
#define STEPPER_SET_SPEED 40
#define STEPPER_SET_CURRENT_POSITION 41
#define STEPPER_RUN_SPEED_TO_POSITION 42
#define STEPPER_STOP 43
#define STEPPER_DISABLE_OUTPUTS 44
#define STEPPER_ENABLE_OUTPUTS 45
#define STEPPER_SET_MINIMUM_PULSE_WIDTH 46
#define STEPPER_SET_ENABLE_PIN 47
#define STEPPER_SET_3_PINS_INVERTED 48
#define STEPPER_SET_4_PINS_INVERTED 49
#define STEPPER_IS_RUNNING 50
#define STEPPER_GET_CURRENT_POSITION 51
#define STEPPER_GET_DISTANCE_TO_GO 52
#define STEPPER_GET_TARGET_POSITION 53
#define GET_FEATURES 54
#define SONAR_SCAN_OFF 55
#define SONAR_SCAN_ON 56
#define BOARD_HARD_RESET 57
#define MATRIX_BANNER_ON 58
#define MATRIX_BANNER_OFF 59


/* Command Forward References*/

// If you add a new command, you must add the command handler
// here as well.

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void analog_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void are_you_there();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void i2c_begin();

extern void i2c_read();

extern void i2c_write();

extern void sonar_new();

extern void dht_new();

extern void stop_all_reports();

extern void set_analog_scanning_interval();

extern void enable_all_reports();

extern void reset_data();

extern void init_pin_structures();

extern void init_spi();

extern void write_blocking_spi();

extern void read_blocking_spi();

extern void set_format_spi();

extern void spi_cs_control();

extern void onewire_init();

extern void onewire_reset();

extern void onewire_select();

extern void onewire_skip();

extern void onewire_write();

extern void onewire_read();

extern void onewire_reset_search();

extern void onewire_search();

extern void onewire_crc8();

extern void set_pin_mode_stepper();

extern void stepper_move_to();

extern void stepper_move();

extern void stepper_run();

extern void stepper_run_speed();

extern void stepper_set_max_speed();

extern void stepper_set_acceleration();

extern void stepper_set_speed();

extern void stepper_get_distance_to_go();

extern void stepper_get_target_position();

extern void stepper_get_current_position();

extern void stepper_set_current_position();

extern void stepper_run_speed_to_position();

extern void stepper_stop();

extern void stepper_disable_outputs();

extern void stepper_enable_outputs();

extern void stepper_set_minimum_pulse_width();

extern void stepper_set_3_pins_inverted();

extern void stepper_set_4_pins_inverted();

extern void stepper_set_enable_pin();

extern void stepper_is_running();

extern void get_features();

extern void sonar_disable();

extern void sonar_enable();

extern void board_hard_reset();

extern void matrix_banner_on();

extern void matrix_banner_off();

// When adding a new command update the command_table.
// The command length is the number of bytes that follow
// the command byte itself, and does not include the command
// byte in its length.

// The command_func is a pointer the command's function.
struct command_descriptor {
    // a pointer to the command processing function
    void (*command_func)(void);
};


// An array of pointers to the command functions.
// The list must be in the same order as the command defines.

command_descriptor command_table[] = {
        { &serial_loopback },
        { &set_pin_mode },
        { &digital_write },
        { &analog_write },
        { &modify_reporting },
        { &get_firmware_version },
        { &are_you_there },
        { &servo_attach },
        { &servo_write },
        { &servo_detach },
        { &i2c_begin },
        { &i2c_read },
        { &i2c_write },
        { &sonar_new },
        { &dht_new },
        { &stop_all_reports },
        { &set_analog_scanning_interval },
        { &enable_all_reports },
        { &reset_data },
        { &init_spi },
        { &write_blocking_spi },
        { &read_blocking_spi },
        { &set_format_spi },
        { &spi_cs_control },
        { &onewire_init },
        { &onewire_reset },
        { &onewire_select },
        { &onewire_skip },
        { &onewire_write },
        { &onewire_read },
        { &onewire_reset_search },
        { &onewire_search },
        { &onewire_crc8 },
        { &set_pin_mode_stepper },
        { &stepper_move_to },
        { &stepper_move },
        { &stepper_run },
        { &stepper_run_speed },
        { &stepper_set_max_speed },
        { &stepper_set_acceleration },
        { &stepper_set_speed },
        (&stepper_set_current_position),
        (&stepper_run_speed_to_position),
        (&stepper_stop),
        (&stepper_disable_outputs),
        (&stepper_enable_outputs),
        (&stepper_set_minimum_pulse_width),
        (&stepper_set_enable_pin),
        (&stepper_set_3_pins_inverted),
        (&stepper_set_4_pins_inverted),
        (&stepper_is_running),
        (&stepper_get_current_position),
        { &stepper_get_distance_to_go },
        (&stepper_get_target_position),
        (&get_features),
        (&sonar_disable),
        (&sonar_enable),
        (&board_hard_reset),
        (&matrix_banner_on),
        (&matrix_banner_off),
};



// maximum length of a command in bytes
#define MAX_COMMAND_LENGTH 30

// buffer to hold incoming command data
byte command_buffer[MAX_COMMAND_LENGTH];


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                 Reporting Defines and Support                    */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Reports sent to the client

#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT ANALOG_WRITE
#define FIRMWARE_REPORT 5
#define I_AM_HERE 6
#define SERVO_UNAVAILABLE 7
#define I2C_TOO_FEW_BYTES_RCVD 8
#define I2C_TOO_MANY_BYTES_RCVD 9
#define I2C_READ_REPORT 10
#define SONAR_DISTANCE 11
#define DHT_REPORT 12
#define SPI_REPORT 13
#define ONE_WIRE_REPORT 14
#define STEPPER_DISTANCE_TO_GO 15
#define STEPPER_TARGET_POSITION 16
#define STEPPER_CURRENT_POSITION 17
#define STEPPER_RUNNING_REPORT 18
#define STEPPER_RUN_COMPLETE_REPORT 19
#define FEATURES 20
#define DEBUG_PRINT 99

#ifdef I2C_ENABLED
// A buffer to hold i2c report data
byte i2c_report_message[64];
#endif

// A buffer to hold spi report data

#ifdef SPI_ENABLED
byte spi_report_message[64];

// SPI settings
uint32_t spi_clock_freq = F_CPU / 4;
uint8_t spi_clock_divider = 4;
#if defined(__AVR__)
uint8_t spi_bit_order = MSBFIRST;
#else
BitOrder spi_bit_order = MSBFIRST;
#endif
uint8_t spi_mode = SPI_MODE0;

#endif

bool stop_reports = false;  // a flag to stop sending all report messages

bool sonar_reporting_enabled = true;  // flag to start and stop sonar reporting

// Input pin reporting control sub commands (modify_reporting)
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4


// DHT Report subtypes
#define DHT_DATA 0
#define DHT_READ_ERROR 1

// firmware version - update this when bumping the version
#define FIRMWARE_MAJOR 1
#define FIRMWARE_MINOR 0
#define FIRMWARE_PATCH 4



// Feature Masks And Storage

#define ONEWIRE_FEATURE 0x01
#define DHT_FEATURE 0x02
#define STEPPERS_FEATURE 0x04
#define SPI_FEATURE 0x08
#define SERVO_FEATURE 0x10
#define SONAR_FEATURE 0x20
#define I2C_FEATURE 0x40
#define LED_MATRIX_FEATURE 0x80

// a byte to hold the enabled features
// the masks are OR'ed into the features byte
uint8_t features = 0;



/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                     i2c Related Defines*/
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**********************************/
/* i2c defines */

#ifdef I2C_ENABLED
// uncomment out the next line to create a 2nd i2c port
// #define SECOND_I2C_PORT

#ifdef SECOND_I2C_PORT
// Change the pins to match SDA and SCL for your board
#define SECOND_I2C_PORT_SDA PB3
#define SECOND_I2C_PORT_SCL PB10

TwoWire Wire2(SECOND_I2C_PORT_SDA, SECOND_I2C_PORT_SCL);
#endif

// a pointer to an active TwoWire object
TwoWire *current_i2c_port;
#endif

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*           LED MATRIX Defines And Data Structures                */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#ifdef LED_MATRIX_SUPPORTED
#define MAX_SCROLL_MESSAGE 25
uint8_t run_banner = 0;
uint8_t banner_text[50] = "USBSerial";
uint8_t no_banner[25] = { 32 };  // turn off all pixels
uint8_t scroll_speed = 50;
const uint16_t ontime = 521;  // microseconds. 521 (us) * 96 (pixels) = 50 ms frame rate if all the pixels are on.
static uint8_t scroll = 0;    // scroll position.


const uint8_t font_5x8[] = {
        3, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,  // space
        1, 0b01011111, 0b00000000, 0b00000000, 0b00000000, 0b00000000,  // !
        3, 0b00000011, 0b00000000, 0b00000011, 0b00000000, 0b00000000,  // "
        5, 0b00010100, 0b00111110, 0b00010100, 0b00111110, 0b00010100,  // #
        4, 0b00100100, 0b01101010, 0b00101011, 0b00010010, 0b00000000,  // $
        5, 0b01100011, 0b00010011, 0b00001000, 0b01100100, 0b01100011,  // %
        5, 0b00110110, 0b01001001, 0b01010110, 0b00100000, 0b01010000,  // &
        1, 0b00000011, 0b00000000, 0b00000000, 0b00000000, 0b00000000,  // '
        3, 0b00011100, 0b00100010, 0b01000001, 0b00000000, 0b00000000,  // (
        3, 0b01000001, 0b00100010, 0b00011100, 0b00000000, 0b00000000,  // )
        5, 0b00101000, 0b00011000, 0b00001110, 0b00011000, 0b00101000,  // *
        5, 0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000,  // +
        2, 0b10110000, 0b01110000, 0b00000000, 0b00000000, 0b00000000,  // ,
        4, 0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00000000,  // -
        2, 0b01100000, 0b01100000, 0b00000000, 0b00000000, 0b00000000,  // .
        4, 0b01100000, 0b00011000, 0b00000110, 0b00000001, 0b00000000,  // /
        4, 0b00111110, 0b01000001, 0b01000001, 0b00111110, 0b00000000,  // 0
        3, 0b01000010, 0b01111111, 0b01000000, 0b00000000, 0b00000000,  // 1
        4, 0b01100010, 0b01010001, 0b01001001, 0b01000110, 0b00000000,  // 2
        4, 0b00100010, 0b01000001, 0b01001001, 0b00110110, 0b00000000,  // 3
        4, 0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00000000,  // 4
        4, 0b00100111, 0b01000101, 0b01000101, 0b00111001, 0b00000000,  // 5
        4, 0b00111110, 0b01001001, 0b01001001, 0b00110000, 0b00000000,  // 6
        4, 0b01100001, 0b00010001, 0b00001001, 0b00000111, 0b00000000,  // 7
        4, 0b00110110, 0b01001001, 0b01001001, 0b00110110, 0b00000000,  // 8
        4, 0b00000110, 0b01001001, 0b01001001, 0b00111110, 0b00000000,  // 9
        2, 0b01010000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,  // :
        2, 0b10000000, 0b01010000, 0b00000000, 0b00000000, 0b00000000,  // ;
        3, 0b00010000, 0b00101000, 0b01000100, 0b00000000, 0b00000000,  // <
        3, 0b00010100, 0b00010100, 0b00010100, 0b00000000, 0b00000000,  // =
        3, 0b01000100, 0b00101000, 0b00010000, 0b00000000, 0b00000000,  // >
        4, 0b00000010, 0b01011001, 0b00001001, 0b00000110, 0b00000000,  // ?
        5, 0b00111110, 0b01001001, 0b01010101, 0b01011101, 0b00001110,  // @
        4, 0b01111110, 0b00010001, 0b00010001, 0b01111110, 0b00000000,  // A
        4, 0b01111111, 0b01001001, 0b01001001, 0b00110110, 0b00000000,  // B
        4, 0b00111110, 0b01000001, 0b01000001, 0b00100010, 0b00000000,  // C
        4, 0b01111111, 0b01000001, 0b01000001, 0b00111110, 0b00000000,  // D
        4, 0b01111111, 0b01001001, 0b01001001, 0b01000001, 0b00000000,  // E
        4, 0b01111111, 0b00001001, 0b00001001, 0b00000001, 0b00000000,  // F
        4, 0b00111110, 0b01000001, 0b01001001, 0b01111010, 0b00000000,  // G
        4, 0b01111111, 0b00001000, 0b00001000, 0b01111111, 0b00000000,  // H
        3, 0b01000001, 0b01111111, 0b01000001, 0b00000000, 0b00000000,  // I
        4, 0b00110000, 0b01000000, 0b01000001, 0b00111111, 0b00000000,  // J
        4, 0b01111111, 0b00001000, 0b00010100, 0b01100011, 0b00000000,  // K
        4, 0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b00000000,  // L
        5, 0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111,  // M
        5, 0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111,  // N
        4, 0b00111110, 0b01000001, 0b01000001, 0b00111110, 0b00000000,  // O
        4, 0b01111111, 0b00001001, 0b00001001, 0b00000110, 0b00000000,  // P
        4, 0b00111110, 0b01000001, 0b01000001, 0b10111110, 0b00000000,  // Q
        4, 0b01111111, 0b00001001, 0b00001001, 0b01110110, 0b00000000,  // R
        4, 0b01000110, 0b01001001, 0b01001001, 0b00110010, 0b00000000,  // S
        5, 0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001,  // T
        4, 0b00111111, 0b01000000, 0b01000000, 0b00111111, 0b00000000,  // U
        5, 0b00001111, 0b00110000, 0b01000000, 0b00110000, 0b00001111,  // V
        5, 0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111,  // W
        5, 0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011,  // X
        5, 0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111,  // Y
        4, 0b01100001, 0b01010001, 0b01001001, 0b01000111, 0b00000000,  // Z
        2, 0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000,  // [
        4, 0b00000001, 0b00000110, 0b00011000, 0b01100000, 0b00000000,  // '\'
        2, 0b01000001, 0b01111111, 0b00000000, 0b00000000, 0b00000000,  // ]
        3, 0b00000010, 0b00000001, 0b00000010, 0b00000000, 0b00000000,  // hat
        4, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b00000000,  // _
        2, 0b00000001, 0b00000010, 0b00000000, 0b00000000, 0b00000000,  // `
        4, 0b00100000, 0b01010100, 0b01010100, 0b01111000, 0b00000000,  // a
        4, 0b01111111, 0b01000100, 0b01000100, 0b00111000, 0b00000000,  // b
        4, 0b00111000, 0b01000100, 0b01000100, 0b00000000, 0b00000000,  // c
        4, 0b00111000, 0b01000100, 0b01000100, 0b01111111, 0b00000000,  // d
        4, 0b00111000, 0b01010100, 0b01010100, 0b00011000, 0b00000000,  // e
        3, 0b00000100, 0b01111110, 0b00000101, 0b00000000, 0b00000000,  // f
        4, 0b10011000, 0b10100100, 0b10100100, 0b01111000, 0b00000000,  // g
        4, 0b01111111, 0b00000100, 0b00000100, 0b01111000, 0b00000000,  // h
        3, 0b01000100, 0b01111101, 0b01000000, 0b00000000, 0b00000000,  // i
        4, 0b01000000, 0b10000000, 0b10000100, 0b01111101, 0b00000000,  // j
        4, 0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000,  // k
        3, 0b01000001, 0b01111111, 0b01000000, 0b00000000, 0b00000000,  // l
        5, 0b01111100, 0b00000100, 0b01111100, 0b00000100, 0b01111000,  // m
        4, 0b01111100, 0b00000100, 0b00000100, 0b01111000, 0b00000000,  // n
        4, 0b00111000, 0b01000100, 0b01000100, 0b00111000, 0b00000000,  // o
        4, 0b11111100, 0b00100100, 0b00100100, 0b00011000, 0b00000000,  // p
        4, 0b00011000, 0b00100100, 0b00100100, 0b11111100, 0b00000000,  // q
        4, 0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00000000,  // r
        4, 0b01001000, 0b01010100, 0b01010100, 0b00100100, 0b00000000,  // s
        3, 0b00000100, 0b00111111, 0b01000100, 0b00000000, 0b00000000,  // t
        4, 0b00111100, 0b01000000, 0b01000000, 0b01111100, 0b00000000,  // u
        5, 0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100,  // v
        5, 0b00111100, 0b01000000, 0b00111100, 0b01000000, 0b00111100,  // w
        5, 0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100,  // x
        4, 0b10011100, 0b10100000, 0b10100000, 0b01111100, 0b00000000,  // y
        3, 0b01100100, 0b01010100, 0b01001100, 0b00000000, 0b00000000,  // z
        3, 0b00001000, 0b00110110, 0b01000001, 0b00000000, 0b00000000,  // {
        1, 0b01111111, 0b00000000, 0b00000000, 0b00000000, 0b00000000,  // |
        3, 0b01000001, 0b00110110, 0b00001000, 0b00000000, 0b00000000,  // }
        4, 0b00001000, 0b00000100, 0b00001000, 0b00000100, 0b00000000,  // ~
};

// LED matrix info.
const uint8_t led_matrix_pin_first = 28;
const uint8_t led_matrix_pin_last = 38;
const uint8_t led_matrix_pin_count = led_matrix_pin_last - led_matrix_pin_first + 1;
const uint8_t led_matrix_rows = 8;
const uint8_t led_matrix_cols = 12;

// Pixel-to-pin translation table.
// A HEX value encodes two pin numbers. The MSB is to be driven LOW,
// the LSB is to be driven HIGH.
// Example: pixel (4,2) contains the value 0x60, meaning that pin 6 must
// be driven low and pin 0 must be driven high to activate the pixel.
// The pin number is an offset to the constant led_matrix_pin_first
// Note that they all appear in pairs, so you could make the table 50%
// smaller at the cost of doing some swapping for odd or even columns.
// (0,0) is upper left corner when the board's USB connector points to the left.
const uint8_t led_matrix_pins[led_matrix_rows][led_matrix_cols] = {
        //  0     1     2     3     4     5     6     7     8     9    10     11
        { 0x37, 0x73, 0x47, 0x74, 0x43, 0x34, 0x87, 0x78, 0x83, 0x38, 0x84, 0x48 },  // 0
        { 0x07, 0x70, 0x03, 0x30, 0x04, 0x40, 0x08, 0x80, 0x67, 0x76, 0x63, 0x36 },  // 1
        { 0x64, 0x46, 0x68, 0x86, 0x60, 0x06, 0x57, 0x75, 0x53, 0x35, 0x54, 0x45 },  // 2
        { 0x58, 0x85, 0x50, 0x05, 0x56, 0x65, 0x17, 0x71, 0x13, 0x31, 0x14, 0x41 },  // 3
        { 0x18, 0x81, 0x10, 0x01, 0x16, 0x61, 0x15, 0x51, 0x27, 0x72, 0x23, 0x32 },  // 4
        { 0x24, 0x42, 0x28, 0x82, 0x20, 0x02, 0x26, 0x62, 0x25, 0x52, 0x21, 0x12 },  // 5
        { 0xa7, 0x7a, 0xa3, 0x3a, 0xa4, 0x4a, 0xa8, 0x8a, 0xa0, 0x0a, 0xa6, 0x6a },  // 6
        { 0xa5, 0x5a, 0xa1, 0x1a, 0xa2, 0x2a, 0x97, 0x79, 0x93, 0x39, 0x94, 0x49 },  // 7
};

// Every byte represents a column of the LED matrix.
// Can hold 32 5x8-font characters.
// Buffer can be smaller at the price of more code.
uint8_t led_matrix_buffer[5 * 32];

// Activate the pixel at (x,y) for ontime microseconds.
void put_pixel(uint8_t x, uint8_t y, uint32_t ontime) {
    uint8_t pins = led_matrix_pins[y][x];
    uint8_t l = (pins >> 4) + led_matrix_pin_first;
    uint8_t h = (pins & 0xf) + led_matrix_pin_first;
    pinMode(l, OUTPUT);
    digitalWrite(l, LOW);
    pinMode(h, OUTPUT);
    digitalWrite(h, HIGH);
    // If ontime = 0, pixel remains active until it is deactivated
    // by another put_pixel that happens to use the same pin(s).
    if (ontime != 0) {
        delayMicroseconds(ontime);
        pinMode(l, INPUT);
        pinMode(h, INPUT);
    }
}

// Call periodically at desired fps rate.
// ontime specifies how long a pixel remains on.
void led_matrix_buffer_show(uint32_t x_offset, uint32_t ontime) {
    for (uint8_t i = 0; i < led_matrix_cols; i++) {
        if (i + x_offset >= sizeof(led_matrix_buffer)) return;
        uint8_t col = led_matrix_buffer[i + x_offset];
        for (uint8_t row = 0; row < led_matrix_rows; row++) {
            if ((col & 0x01) != 0) {
                put_pixel(i, row, ontime);
            }
            col >>= 1;
        }
    }
}

// Write a character to the buffer.
uint8_t led_matrix_putch(uint8_t *p_buffer, uint16_t buffer_size, uint8_t ch) {
    uint8_t i;

    if (ch < ' ') return 0;
    ch -= ' ';
    uint16_t offset = 6 * ch;
    uint8_t width = font_5x8[offset];
    for (i = 0; i < width; i++) {
        offset += 1;
        // This is supposed to prevent buffer overflow.
        if (i >= buffer_size) break;
        p_buffer[i] = font_5x8[offset];
    }
    return i + 1;
}

// Write a string to the buffer.
uint16_t led_matrix_puts(uint8_t *p_buffer, uint16_t buffer_size, uint8_t *p_str) {
    uint8_t *p = p_buffer;
    while (*p_str != 0) {
        p += led_matrix_putch(p, buffer_size - (p - p_buffer), *p_str);
        p_str++;
    }
    return p - p_buffer;
}

uint32_t t_prev = 0;

#endif

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*           Pin Related Defines And Data Structures                */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


// Pin mode definitions

// INPUT defined in Arduino.h = 0
// OUTPUT defined in Arduino.h = 1
// INPUT_PULLUP defined in Arduino.h = 2
// The following are defined for arduino_telemetrix (AT)
#define AT_ANALOG 3
#define AT_MODE_NOT_SET 255

// maximum number of pins supported
#define MAX_DIGITAL_PINS_SUPPORTED 14
#define MAX_ANALOG_PINS_SUPPORTED 6


// Analog input pins are defined from
// A0 - A5.


// To translate a pin number from an integer value to its analog pin number
// equivalent, this array is used to look up the value to use for the pin.
int analog_read_pins[6] = { A0, A1, A2, A3, A4, A5 };

// a descriptor for digital pins
struct pin_descriptor {
    byte pin_number;
    byte pin_mode;
    bool reporting_enabled;  // If true, then send reports if an input pin
    int last_value;          // Last value read for input mode
};

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

// a descriptor for digital pins
struct analog_pin_descriptor {
    byte pin_number;
    byte pin_mode;
    bool reporting_enabled;  // If true, then send reports if an input pin
    int last_value;          // Last value read for input mode
    int differential;        // difference between current and last value needed
    // to generate a report
};

// an array of analog_pin_descriptors
analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

unsigned long current_millis;   // for analog input loop
unsigned long previous_millis;  // for analog input loop
uint8_t analog_sampling_interval = 19;


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*  Feature Related Defines, Data Structures and Storage Allocation */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// servo management
#ifdef SERVO_ENABLED
Servo servos[MAX_SERVOS];

// this array allows us to retrieve the servo object
// associated with a specific pin number
byte pin_to_servo_index_map[MAX_SERVOS];
#endif

// HC-SR04 Sonar Management
#define MAX_SONARS 6

#ifdef SONAR_ENABLED
struct Sonar {
    uint8_t trigger_pin;
    unsigned int last_value;
    NewPing *usonic;
};

// an array of sonar objects
Sonar sonars[MAX_SONARS];

byte sonars_index = 0;  // index into sonars struct

// used for scanning the sonar devices.
byte last_sonar_visited = 0;
#endif  //SONAR_ENABLED

unsigned long sonar_current_millis;   // for analog input loop
unsigned long sonar_previous_millis;  // for analog input loop

#ifdef SONAR_ENABLED
uint8_t sonar_scan_interval = 33;  // Milliseconds between sensor pings
// (29ms is about the min to avoid = 19;
#endif

// DHT Management
#define MAX_DHTS 6                 // max number of devices
#define READ_FAILED_IN_SCANNER 0   // read request failed when scanning
#define READ_IN_FAILED_IN_SETUP 1  // read request failed when initially setting up

#ifdef DHT_ENABLED
struct DHT {
    uint8_t pin;
    uint8_t dht_type;
    unsigned int last_value;
    DHTStable *dht_sensor;
};

// an array of dht objects
DHT dhts[MAX_DHTS];

byte dht_index = 0;  // index into dht struct

unsigned long dht_current_millis;       // for analog input loop
unsigned long dht_previous_millis;      // for analog input loop
unsigned int dht_scan_interval = 2000;  // scan dht's every 2 seconds
#endif                                  // DHT_ENABLED


/* OneWire Object*/

// a pointer to a OneWire object
#ifdef ONE_WIRE_ENABLED
OneWire *ow = NULL;
#endif

#define MAX_NUMBER_OF_STEPPERS 4

// stepper motor data
#ifdef STEPPERS_ENABLED
AccelStepper *steppers[MAX_NUMBER_OF_STEPPERS];

// stepper run modes
uint8_t stepper_run_modes[MAX_NUMBER_OF_STEPPERS];
#endif


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                       Command Functions                          */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


// A method to send debug data across the serial link
void send_debug_info(byte id, int value) {
    byte debug_buffer[5] = { (byte)4, (byte)DEBUG_PRINT, 0, 0, 0 };
    debug_buffer[2] = id;
    debug_buffer[3] = highByte(value);
    debug_buffer[4] = lowByte(value);
    Serial.write(debug_buffer, 5);
}

// a function to loop back data over the serial port
void serial_loopback() {
    byte loop_back_buffer[3] = { 2, (byte)SERIAL_LOOP_BACK, command_buffer[0] };
    Serial.write(loop_back_buffer, 3);
    delay(30);
}

void set_pin_mode()
/*
    Set a pin to digital input, digital input_pullup, digital output,
    and analog input. PWM is considered digital output, and i2c, spi, dht,
    sonar, servo, and onewire have their own init methods.
*/
{
    byte pin;
    byte mode;
    pin = command_buffer[0];
    mode = command_buffer[1];

    switch (mode) {
        case INPUT:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[2];
            pinMode(pin, INPUT);
            break;
        case INPUT_PULLUP:
            the_digital_pins[pin].pin_mode = mode;
            the_digital_pins[pin].reporting_enabled = command_buffer[2];
            pinMode(pin, INPUT_PULLUP);
            break;
        case OUTPUT:
            the_digital_pins[pin].pin_mode = mode;
            pinMode(pin, OUTPUT);
            break;
        case AT_ANALOG:
            the_analog_pins[pin].pin_mode = mode;
            the_analog_pins[pin].differential = (command_buffer[2] << 8) + command_buffer[3];
            the_analog_pins[pin].reporting_enabled = command_buffer[4];
            break;
        default:
            break;
    }
}

// set the analog scanning interval
void set_analog_scanning_interval() {
    analog_sampling_interval = command_buffer[0];
}

// set the state of digital output pin
void digital_write() {
    byte pin;
    byte value;
    pin = command_buffer[0];
    value = command_buffer[1];
    digitalWrite(pin, value);
}

// set the pwm value for a digital output pin
// The term analog is confusing here, but it is what
// Arduino uses.
void analog_write() {
    // command_buffer[0] = PIN, command_buffer[1] = value_msb,
    // command_buffer[2] = value_lsb
    byte pin;  // command_buffer[0]
    unsigned int value;

    pin = command_buffer[0];

    value = (command_buffer[1] << 8) + command_buffer[2];
    analogWrite(pin, value);
}

// This method allows you to modify what reports are generated.
// You can disable all reports, including dhts, and sonar.
// You can disable only digital and analog reports on a
// pin basis, or enable those on a pin basis.
void modify_reporting() {
    int pin = command_buffer[1];

    switch (command_buffer[0]) {
        case REPORTING_DISABLE_ALL:
            for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
                the_digital_pins[i].reporting_enabled = false;
            }
            for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
                the_analog_pins[i].reporting_enabled = false;
            }
            break;
        case REPORTING_ANALOG_ENABLE:
            if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET) {
                the_analog_pins[pin].reporting_enabled = true;
            }
            break;
        case REPORTING_ANALOG_DISABLE:
            if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET) {
                the_analog_pins[pin].reporting_enabled = false;
            }
            break;
        case REPORTING_DIGITAL_ENABLE:
            if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET) {
                the_digital_pins[pin].reporting_enabled = true;
            }
            break;
        case REPORTING_DIGITAL_DISABLE:
            if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET) {
                the_digital_pins[pin].reporting_enabled = false;
            }
            break;
        default:
            break;
    }
}

// retrieve the features byte
void get_features() {
    byte report_message[3] = { 2, FEATURES, features };
    Serial.write(report_message, 3);
}

// Return the firmware version number
void get_firmware_version() {
    byte report_message[5] = { 4, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR,
                               FIRMWARE_PATCH };
    Serial.write(report_message, 5);
}

// Query the firmware for the Arduino ID in use
void are_you_there() {
    byte report_message[3] = { 2, I_AM_HERE, ARDUINO_ID };
    Serial.write(report_message, 3);
}

/***************************************************
   Servo Commands
 **************************************************/

// Find the first servo that is not attached to a pin
// This is a helper function not called directly via the API
int find_servo() {
    int index = -1;

#ifdef SERVO_ENABLED
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (servos[i].attached() == false) {
            index = i;
            break;
        }
    }
#endif

    return index;
}

// Associate a pin with a servo
void servo_attach() {
#ifdef SERVO_ENABLED
    byte pin = command_buffer[0];
    int servo_found = -1;

    int minpulse = (command_buffer[1] << 8) + command_buffer[2];
    int maxpulse = (command_buffer[3] << 8) + command_buffer[4];

    // find the first available open servo
    servo_found = find_servo();
    if (servo_found != -1) {
        pin_to_servo_index_map[servo_found] = pin;
        servos[servo_found].attach(pin, minpulse, maxpulse);
    } else {
        // no open servos available, send a report back to client
        byte report_message[2] = { SERVO_UNAVAILABLE, pin };
        Serial.write(report_message, 2);
    }
#endif
}

// set a servo to a given angle
void servo_write() {
#ifdef SERVO_ENABLED
    byte pin = command_buffer[0];
    int angle = command_buffer[1];
    // find the servo object for the pin
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (pin_to_servo_index_map[i] == pin) {

            servos[i].write(angle);
            return;
        }
    }
#endif
}

// detach a servo and make it available for future use
void servo_detach() {
#ifdef SERVO_ENABLED
    byte pin = command_buffer[0];

    // find the servo object for the pin
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (pin_to_servo_index_map[i] == pin) {

            pin_to_servo_index_map[i] = -1;
            servos[i].detach();
        }
    }
#endif
}

/***********************************
   i2c functions
 **********************************/

// initialize i2c data transfers
void i2c_begin() {
#ifdef I2C_ENABLED
    byte i2c_port = command_buffer[0];
    if (not i2c_port) {
        Wire.begin();
    }

#ifdef SECOND_I2C_PORT
    else {
    Wire2.begin();
  }
#endif
#endif
}

// read a number of bytes from a specific i2c register
void i2c_read() {
#ifdef I2C_ENABLED
    // data in the incoming message:
    // i2c_port [0]
    // address, [1]
    // register, [2]
    // number of bytes, [3]
    // stop transmitting flag [4]
    // i2c port [5]
    // write the register [6]

    int message_size = 0;
    byte address = command_buffer[0];
    byte the_register = command_buffer[1];

    // set the current i2c port if this is for the primary i2c
    if (command_buffer[4] == 0) {
        current_i2c_port = &Wire;
    }

#ifdef SECOND_I2C_PORT
    // this is for port 2
  if (command_buffer[4] == 1) {
    current_i2c_port = &Wire2;
  }
#endif

    // write byte is true, then write the register
    if (command_buffer[5]) {
        current_i2c_port->beginTransmission(address);
        current_i2c_port->write((byte)the_register);
        current_i2c_port->endTransmission(command_buffer[3]);  // default = true
    }
    current_i2c_port->requestFrom(address, command_buffer[2]);  // all bytes are returned in requestFrom

    // check to be sure correct number of bytes were returned by slave
    if (command_buffer[2] < current_i2c_port->available()) {
        byte report_message[4] = { 3, I2C_TOO_FEW_BYTES_RCVD, 1, address };
        Serial.write(report_message, 4);
        return;
    } else if (command_buffer[2] > current_i2c_port->available()) {
        byte report_message[4] = { 3, I2C_TOO_MANY_BYTES_RCVD, 1, address };
        Serial.write(report_message, 4);
        return;
    }

    // packet length
    i2c_report_message[0] = command_buffer[2] + 5;

    // report type
    i2c_report_message[1] = I2C_READ_REPORT;

    // i2c_port
    i2c_report_message[2] = command_buffer[4];

    // number of bytes read
    i2c_report_message[3] = command_buffer[2];  // number of bytes

    // device address
    i2c_report_message[4] = address;

    // device register
    i2c_report_message[5] = the_register;

    // append the data that was read
    for (message_size = 0; message_size < command_buffer[2] && current_i2c_port->available(); message_size++) {
        i2c_report_message[6 + message_size] = current_i2c_port->read();
    }
    // send slave address, register and received bytes

    for (int i = 0; i < message_size + 6; i++) {
        Serial.write(i2c_report_message[i]);
    }
#endif
}

// write a specified number of bytes to an i2c device
void i2c_write() {
#ifdef I2C_ENABLED
    // command_buffer[0] is the number of bytes to send
    // command_buffer[1] is the device address
    // command_buffer[2] is the i2c port
    // additional bytes to write= command_buffer[3..];

    // set the current i2c port if this is for the primary i2c
    if (command_buffer[2] == 0) {
        current_i2c_port = &Wire;
    }

#ifdef SECOND_I2C_PORT
    // this is for port 2
  if (command_buffer[2] == 1) {
    current_i2c_port = &Wire2;
  }
#endif

    current_i2c_port->beginTransmission(command_buffer[1]);

    // write the data to the device
    for (int i = 0; i < command_buffer[0]; i++) {
        current_i2c_port->write(command_buffer[i + 3]);
    }
    current_i2c_port->endTransmission();
    delayMicroseconds(70);
#endif
}

/***********************************
   HC-SR04 adding a new device
 **********************************/

// associate 2 pins as trigger and echo pins for a sonar device
void sonar_new() {
#ifdef SONAR_ENABLED

    // command_buffer[0] = trigger pin,  command_buffer[1] = echo pin
    sonars[sonars_index].usonic = new NewPing((uint8_t)command_buffer[0], (uint8_t)command_buffer[1]);
    sonars[sonars_index].trigger_pin = command_buffer[0];
    sonars_index++;
#endif
}

void sonar_disable() {
    sonar_reporting_enabled = false;
}

void sonar_enable() {
    sonar_reporting_enabled = true;
}

void board_hard_reset() {
    NVIC_SystemReset();
    delay(2000);
}

#ifdef LED_MATRIX_SUPPORTED

void matrix_banner_on() {
    uint8_t message_length = command_buffer[0];
    scroll_speed = command_buffer[1];
    memset(banner_text, 0, sizeof(banner_text));
    memset(led_matrix_buffer, 0, sizeof(led_matrix_buffer));
    for (int i = 0; i < message_length; i++) {
        banner_text[i] = command_buffer[2 + i];
    }
    run_banner = 1;
    // Load text message.
    led_matrix_puts(led_matrix_buffer, sizeof(led_matrix_buffer), banner_text);
    // Ready...
    t_prev = millis();
}

void matrix_banner_off() {
    run_banner = 0;
}

#endif
/***********************************
   DHT adding a new device
 **********************************/

// associate a pin with a dht device
void dht_new() {
#ifdef DHT_ENABLED

    if (dht_index < MAX_DHTS) {
        dhts[dht_index].dht_sensor = new DHTStable();

        dhts[dht_index].pin = command_buffer[0];
        dhts[dht_index].dht_type = command_buffer[1];
        dht_index++;
    }
#endif
}

// initialize the SPI interface
void init_spi() {

#ifdef SPI_ENABLED
    int cs_pin;

    //Serial.print(command_buffer[1]);
    // initialize chip select GPIO pins
    for (int i = 0; i < command_buffer[0]; i++) {
        cs_pin = command_buffer[1 + i];
        // Chip select is active-low, so we'll initialise it to a driven-high state
        pinMode(cs_pin, OUTPUT);
        digitalWrite(cs_pin, HIGH);
    }
    SPI.begin();
#endif
}

// write a number of blocks to the SPI device
void write_blocking_spi() {
#ifdef SPI_ENABLED
    int num_bytes = command_buffer[1];

    //send_debug_info(4, spi_bit_order);
    //send_debug_info(5, spi_mode);
    SPI.beginTransaction(SPISettings(spi_clock_freq, spi_bit_order, spi_mode));
    digitalWrite(command_buffer[0], 0);
    for (int i = 0; i < num_bytes; i++) {
        SPI.transfer(command_buffer[2 + i]);
    }
    digitalWrite(command_buffer[0], 1);

    SPI.endTransaction();
#endif
}

// read a number of bytes from the SPI device
void read_blocking_spi() {
#ifdef SPI_ENABLED

    spi_report_message[0] = command_buffer[1] + 4;  // packet length
    spi_report_message[1] = SPI_REPORT;
    spi_report_message[2] = command_buffer[0];  // chip select pin
    spi_report_message[3] = command_buffer[2];  // register
    spi_report_message[4] = command_buffer[1];  // number of bytes read

    //send_debug_info(command_buffer[0], 0);
    //send_debug_info(command_buffer[1], 1);
    //send_debug_info(command_buffer[2], 2);
    //send_debug_info(command_buffer[3], 3);

    SPI.beginTransaction(SPISettings(spi_clock_freq, spi_bit_order, spi_mode));
    // command_buffer[0] == number of bytes to read
    // command_buffer[1] == read register

    // spi_report_message[0] = length of message including this element
    // spi_report_message[1] = SPI_REPORT
    // spi_report_message[2] = register used for the read
    // spi_report_message[3] = number of bytes returned
    // spi_report_message[4..] = data read

    // configure the report message
    // calculate the packet length
    digitalWrite(command_buffer[0], 0);

    // write the register out. OR it with 0x80 to indicate a read

    SPI.transfer(command_buffer[2] | 0x80);
    delay(100);

    // now read the specified number of bytes and place
    // them in the report buffer
    for (int i = 0; i < command_buffer[1]; i++) {
        spi_report_message[i + 5] = SPI.transfer(0x00);
    }
    //send_debug_info(SPI.transfer(0x00), 2);
    digitalWrite(command_buffer[0], 1);

    SPI.endTransaction();

    Serial.write(spi_report_message, command_buffer[1] + 5);

#endif
}

// modify the SPI format
void set_format_spi() {
#ifdef SPI_ENABLED

    spi_clock_freq = F_CPU / command_buffer[0];
    switch (command_buffer[2]) {
        case 0:
            spi_mode = SPI_MODE0;
            break;
        case 1:
            spi_mode = SPI_MODE1;
            break;
        case 2:
            spi_mode = SPI_MODE2;
            break;
        case 3:
            spi_mode = SPI_MODE3;
            break;
        default:
            spi_mode = SPI_MODE0;
            break;
    }

#if defined(__AVR__)
    spi_bit_order = command_buffer[1];
  //SPISettings(command_buffer[0], command_buffer[1], command_buffer[2]);
#else
    BitOrder b;

    if (command_buffer[1]) {
        b = MSBFIRST;
    } else {
        b = LSBFIRST;
    }
    spi_bit_order = b;
#endif  // avr
#endif  // SPI_ENABLED
}

// set the SPI chip select line
void spi_cs_control() {
#ifdef SPI_ENABLED
    int cs_pin = command_buffer[0];
    int cs_state = command_buffer[1];
    digitalWrite(cs_pin, cs_state);
#endif
}

// Initialize the OneWire interface
void onewire_init() {
#ifdef ONE_WIRE_ENABLED
    ow = new OneWire(command_buffer[0]);
#endif
}

// send a OneWire reset
void onewire_reset() {
#ifdef ONE_WIRE_ENABLED

    uint8_t reset_return = ow->reset();
    uint8_t onewire_report_message[] = { 3, ONE_WIRE_REPORT, ONE_WIRE_RESET, reset_return };

    Serial.write(onewire_report_message, 4);
#endif
}

// send a OneWire select
void onewire_select() {
#ifdef ONE_WIRE_ENABLED

    uint8_t dev_address[8];

    for (int i = 0; i < 8; i++) {
        dev_address[i] = command_buffer[i];
    }
    ow->select(dev_address);
#endif
}

// send a OneWire skip
void onewire_skip() {
#ifdef ONE_WIRE_ENABLED
    ow->skip();
#endif
}

// write 1 byte to the OneWire device
void onewire_write() {
#ifdef ONE_WIRE_ENABLED

    // write data and power values
    ow->write(command_buffer[0], command_buffer[1]);
#endif
}

// read one byte from the OneWire device
void onewire_read() {
#ifdef ONE_WIRE_ENABLED

    // onewire_report_message[0] = length of message including this element
    // onewire_report_message[1] = ONEWIRE_REPORT
    // onewire_report_message[2] = message subtype = 29
    // onewire_report_message[3] = data read

    uint8_t data = ow->read();

    uint8_t onewire_report_message[] = { 3, ONE_WIRE_REPORT, ONE_WIRE_READ, data };

    Serial.write(onewire_report_message, 4);
#endif
}

// Send a OneWire reset search command
void onewire_reset_search() {
#ifdef ONE_WIRE_ENABLED

    ow->reset_search();
#endif
}

// Send a OneWire search command
void onewire_search() {
#ifdef ONE_WIRE_ENABLED

    uint8_t onewire_report_message[] = { 10, ONE_WIRE_REPORT, ONE_WIRE_SEARCH,
                                         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                         0xff };

    ow->search(&onewire_report_message[3]);
    Serial.write(onewire_report_message, 11);
#endif
}

// Calculate a OneWire CRC8 on a buffer containing a specified number of bytes
void onewire_crc8() {
#ifdef ONE_WIRE_ENABLED

    uint8_t crc = ow->crc8(&command_buffer[1], command_buffer[0]);
    uint8_t onewire_report_message[] = { 3, ONE_WIRE_REPORT, ONE_WIRE_CRC8, crc };
    Serial.write(onewire_report_message, 4);
#endif
}

// Stepper Motor supported
// Stepper Motor supported
void set_pin_mode_stepper() {
#ifdef STEPPERS_ENABLED

    // motor_id = command_buffer[0]
  // interface = command_buffer[1]
  // pin1 = command_buffer[2]
  // pin2 = command_buffer[3]
  // pin3 = command_buffer[4]
  // pin4 = command_buffer[5]
  // enable = command_buffer[6]

  // instantiate a stepper object and store it in the stepper array
  steppers[command_buffer[0]] = new AccelStepper(command_buffer[1], command_buffer[2],
                                                 command_buffer[3], command_buffer[4],
                                                 command_buffer[5], command_buffer[6]);
#endif
}

void stepper_move_to() {
#ifdef STEPPERS_ENABLED

    // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]
  // polarity = command_buffer[5]

  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];
  if (command_buffer[5]) {
    position *= -1;
  }
  steppers[command_buffer[0]]->moveTo(position);
#endif
}

void stepper_move() {
#ifdef STEPPERS_ENABLED

    // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]
  // polarity = command_buffer[5]


  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[1]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];
  if (command_buffer[5]) {
    position *= -1;
  }
  steppers[command_buffer[0]]->move(position);
#endif
}

void stepper_run() {
#ifdef STEPPERS_ENABLED
    stepper_run_modes[command_buffer[0]] = STEPPER_RUN;
#endif
}

void stepper_run_speed() {
    // motor_id = command_buffer[0]
#ifdef STEPPERS_ENABLED

    stepper_run_modes[command_buffer[0]] = STEPPER_RUN_SPEED;
#endif
}

void stepper_set_max_speed() {
#ifdef STEPPERS_ENABLED

    // motor_id = command_buffer[0]
  // speed_msb = command_buffer[1]
  // speed_lsb = command_buffer[2]

  float max_speed = (float)((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setMaxSpeed(max_speed);
#endif
}

void stepper_set_acceleration() {
#ifdef STEPPERS_ENABLED

    // motor_id = command_buffer[0]
  // accel_msb = command_buffer[1]
  // accel = command_buffer[2]

  float acceleration = (float)((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setAcceleration(acceleration);
#endif
}

void stepper_set_speed() {

    // motor_id = command_buffer[0]
    // speed_msb = command_buffer[1]
    // speed_lsb = command_buffer[2]
#ifdef STEPPERS_ENABLED

    float speed = (float)((command_buffer[1] << 8) + command_buffer[2]);
  steppers[command_buffer[0]]->setSpeed(speed);
#endif
}

void stepper_get_distance_to_go() {
#ifdef STEPPERS_ENABLED
    // motor_id = command_buffer[0]

  // report = STEPPER_DISTANCE_TO_GO, motor_id, distance(8 bytes)



  byte report_message[7] = { 6, STEPPER_DISTANCE_TO_GO, command_buffer[0] };

  long dtg = steppers[command_buffer[0]]->distanceToGo();


  report_message[3] = (byte)((dtg & 0xFF000000) >> 24);
  report_message[4] = (byte)((dtg & 0x00FF0000) >> 16);
  report_message[5] = (byte)((dtg & 0x0000FF00) >> 8);
  report_message[6] = (byte)((dtg & 0x000000FF));

  // motor_id = command_buffer[0]
  Serial.write(report_message, 7);
#endif
}

void stepper_get_target_position() {
#ifdef STEPPERS_ENABLED
    // motor_id = command_buffer[0]

  // report = STEPPER_TARGET_POSITION, motor_id, distance(8 bytes)



  byte report_message[7] = { 6, STEPPER_TARGET_POSITION, command_buffer[0] };

  long target = steppers[command_buffer[0]]->targetPosition();


  report_message[3] = (byte)((target & 0xFF000000) >> 24);
  report_message[4] = (byte)((target & 0x00FF0000) >> 16);
  report_message[5] = (byte)((target & 0x0000FF00) >> 8);
  report_message[6] = (byte)((target & 0x000000FF));

  // motor_id = command_buffer[0]
  Serial.write(report_message, 7);
#endif
}

void stepper_get_current_position() {
#ifdef STEPPERS_ENABLED
    // motor_id = command_buffer[0]

  // report = STEPPER_CURRENT_POSITION, motor_id, distance(8 bytes)



  byte report_message[7] = { 6, STEPPER_CURRENT_POSITION, command_buffer[0] };

  long position = steppers[command_buffer[0]]->currentPosition();


  report_message[3] = (byte)((position & 0xFF000000) >> 24);
  report_message[4] = (byte)((position & 0x00FF0000) >> 16);
  report_message[5] = (byte)((position & 0x0000FF00) >> 8);
  report_message[6] = (byte)((position & 0x000000FF));

  // motor_id = command_buffer[0]
  Serial.write(report_message, 7);
#endif
}

void stepper_set_current_position() {
#ifdef STEPPERS_ENABLED
    // motor_id = command_buffer[0]
  // position MSB = command_buffer[1]
  // position MSB-1 = command_buffer[2]
  // position MSB-2 = command_buffer[3]
  // position LSB = command_buffer[4]

  // convert the 4 position bytes to a long
  long position = (long)(command_buffer[2]) << 24;
  position += (long)(command_buffer[2]) << 16;
  position += command_buffer[3] << 8;
  position += command_buffer[4];

  steppers[command_buffer[0]]->setCurrentPosition(position);
#endif
}

void stepper_run_speed_to_position() {
#ifdef STEPPERS_ENABLED
    stepper_run_modes[command_buffer[0]] = STEPPER_RUN_SPEED_TO_POSITION;

#endif
}

void stepper_stop() {
#ifdef STEPPERS_ENABLED
    steppers[command_buffer[0]]->stop();
  steppers[command_buffer[0]]->disableOutputs();
  stepper_run_modes[command_buffer[0]] = STEPPER_STOP;


#endif
}

void stepper_disable_outputs() {
#ifdef STEPPERS_ENABLED
    steppers[command_buffer[0]]->disableOutputs();
#endif
}

void stepper_enable_outputs() {
#ifdef STEPPERS_ENABLED
    steppers[command_buffer[0]]->enableOutputs();
#endif
}

void stepper_set_minimum_pulse_width() {
#ifdef STEPPERS_ENABLED
    unsigned int pulse_width = (command_buffer[1] << 8) + command_buffer[2];
  steppers[command_buffer[0]]->setMinPulseWidth(pulse_width);
#endif
}

void stepper_set_enable_pin() {
#ifdef STEPPERS_ENABLED
    steppers[command_buffer[0]]->setEnablePin((uint8_t)command_buffer[1]);
#endif
}

void stepper_set_3_pins_inverted() {
#ifdef STEPPERS_ENABLED
    // command_buffer[1] = directionInvert
  // command_buffer[2] = stepInvert
  // command_buffer[3] = enableInvert
  steppers[command_buffer[0]]->setPinsInverted((bool)command_buffer[1],
                                               (bool)command_buffer[2],
                                               (bool)command_buffer[3]);
#endif
}

void stepper_set_4_pins_inverted() {
    // command_buffer[1] = pin1
    // command_buffer[2] = pin2
    // command_buffer[3] = pin3
    // command_buffer[4] = pin4
    // command_buffer[5] = enable
#ifdef STEPPERS_ENABLED
    steppers[command_buffer[0]]->setPinsInverted((bool)command_buffer[1],
                                               (bool)command_buffer[2],
                                               (bool)command_buffer[3],
                                               (bool)command_buffer[4],
                                               (bool)command_buffer[5]);
#endif
}

void stepper_is_running() {
#ifdef STEPPERS_ENABLED
    // motor_id = command_buffer[0]

  // report = STEPPER_IS_RUNNING, motor_id, distance(8 bytes)


  byte report_message[3] = { 2, STEPPER_RUNNING_REPORT, command_buffer[0] };

  report_message[2] = steppers[command_buffer[0]]->isRunning();

  Serial.write(report_message, 3);
#endif
}

// stop all reports from being generated

void stop_all_reports() {
    stop_reports = true;
    delay(20);
    Serial.flush();
}

// enable all reports to be generated
void enable_all_reports() {
    Serial.flush();
    stop_reports = false;
    delay(20);
}

// The LED Matrix code is based upon
// https://github.com/ClemensAtElektor/Arduino-UNO-R4/blob/main/R4WiFi_led_matrix/R4WiFi_led_matrix.ino


// retrieve the next command from the serial link
void get_next_command() {
    byte command;
    byte packet_length;
    command_descriptor command_entry;

    // clear the command buffer
    memset(command_buffer, 0, sizeof(command_buffer));

    // if there is no command waiting, then return
    if (not Serial.available()) {
        return;
    }
    // get the packet length
    packet_length = (byte)Serial.read();

    while (not Serial.available()) {
        delay(1);
    }

    // get the command byte
    command = (byte)Serial.read();

    // uncomment the next line to see the packet length and command
    //send_debug_info(packet_length, command);
    command_entry = command_table[command];

    if (packet_length > 1) {
        // get the data for that command
        for (int i = 0; i < packet_length - 1; i++) {
            // need this delay or data read is not correct
            while (not Serial.available()) {
                delay(1);
            }
            command_buffer[i] = (byte)Serial.read();
            // uncomment out to see each of the bytes following the command
            //send_debug_info(i, command_buffer[i]);
        }
    }
    command_entry.command_func();
}

// reset the internal data structures to a known state
void reset_data() {
    // reset the data structures

    // fist stop all reporting
    stop_all_reports();

    current_millis = 0;   // for analog input loop
    previous_millis = 0;  // for analog input loop
    analog_sampling_interval = 19;

    // detach any attached servos
#ifdef SERVO_ENABLED
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (servos[i].attached() == true) {
            servos[i].detach();
        }
    }
#endif

#ifdef SONAR_ENABLED
    sonars_index = 0;  // reset the index into the sonars array

    sonar_current_millis = 0;   // for analog input loop
    sonar_previous_millis = 0;  // for analog input loop
    sonar_scan_interval = 33;   // Milliseconds between sensor pings
    memset(sonars, 0, sizeof(sonars));
#endif

#ifdef DHT_ENABLED
    dht_index = 0;  // index into dht array

    dht_current_millis = 0;    // for analog input loop
    dht_previous_millis = 0;   // for analog input loop
    dht_scan_interval = 2000;  // scan dht's every 2 seconds
#endif

#ifdef DHT_ENABLED
    memset(dhts, 0, sizeof(dhts));
#endif
    enable_all_reports();
}

// initialize the pin data structures
void init_pin_structures() {
    for (byte i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        the_digital_pins[i].pin_number = i;
        the_digital_pins[i].pin_mode = AT_MODE_NOT_SET;
        the_digital_pins[i].reporting_enabled = false;
        the_digital_pins[i].last_value = 0;
    }

    // establish the analog pin array
    for (byte i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
        the_analog_pins[i].pin_number = i;
        the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
        the_analog_pins[i].reporting_enabled = false;
        the_analog_pins[i].last_value = 0;
        the_analog_pins[i].differential = 0;
    }
}


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*    Scanning Inputs, Generating Reports And Running Steppers      */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


// scan the digital input pins for changes
void scan_digital_inputs() {
    byte value;

    // report message

    // byte 0 = packet length
    // byte 1 = report type
    // byte 2 = pin number
    // byte 3 = value
    byte report_message[4] = { 3, DIGITAL_REPORT, 0, 0 };

    for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
        if (the_digital_pins[i].pin_mode == INPUT || the_digital_pins[i].pin_mode == INPUT_PULLUP) {
            if (the_digital_pins[i].reporting_enabled) {
                // if the value changed since last read
                value = (byte)digitalRead(the_digital_pins[i].pin_number);
                if (value != the_digital_pins[i].last_value) {
                    the_digital_pins[i].last_value = value;
                    report_message[2] = (byte)i;
                    report_message[3] = value;
                    Serial.write(report_message, 4);
                    delay(1);
                }
            }
        }
    }
}

// scan the analog input pins for changes
void scan_analog_inputs() {
    int value;

    // report message

    // byte 0 = packet length
    // byte 1 = report type
    // byte 2 = pin number
    // byte 3 = high order byte of value
    // byte 4 = low order byte of value

    byte report_message[5] = { 4, ANALOG_REPORT, 0, 0, 0 };

    uint8_t adjusted_pin_number;
    int differential;

    current_millis = millis();
    if (current_millis - previous_millis > analog_sampling_interval) {
        previous_millis = current_millis;

        for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
            if (the_analog_pins[i].pin_mode == AT_ANALOG) {
                if (the_analog_pins[i].reporting_enabled) {
                    // if the value changed since last read
                    // adjust pin number for the actual read
                    adjusted_pin_number = (uint8_t)(analog_read_pins[i]);
                    value = analogRead(adjusted_pin_number);
                    differential = abs(value - the_analog_pins[i].last_value);
                    if (differential >= the_analog_pins[i].differential) {
                        //trigger value achieved, send out the report
                        the_analog_pins[i].last_value = value;
                        // input_message[1] = the_analog_pins[i].pin_number;
                        report_message[2] = (byte)i;
                        report_message[3] = highByte(value);  // get high order byte
                        report_message[4] = lowByte(value);
                        Serial.write(report_message, 5);
                        delay(1);
                    }
                }
            }
        }
    }
}

// scan the sonar devices for changes
void scan_sonars() {
#ifdef SONAR_ENABLED
    unsigned int distance;

    if (sonars_index) {
        sonar_current_millis = millis();
        if (sonar_current_millis - sonar_previous_millis > sonar_scan_interval) {
            sonar_previous_millis = sonar_current_millis;
            distance = sonars[last_sonar_visited].usonic->ping_cm();
            if (distance != sonars[last_sonar_visited].last_value) {
                sonars[last_sonar_visited].last_value = distance;

                // byte 0 = packet length
                // byte 1 = report type
                // byte 2 = trigger pin number
                // byte 3 = distance high order byte
                // byte 4 = distance low order byte
                byte report_message[5] = { 4, SONAR_DISTANCE, sonars[last_sonar_visited].trigger_pin,
                                           (byte)(distance >> 8), (byte)(distance & 0xff) };
                Serial.write(report_message, 5);
            }
            last_sonar_visited++;
            if (last_sonar_visited == sonars_index) {
                last_sonar_visited = 0;
            }
        }
    }
#endif
}

// scan dht devices for changes
void scan_dhts() {
#ifdef DHT_ENABLED
    // prebuild report for valid data
    // reuse the report if a read command fails

    // data returned is in floating point form - 4 bytes
    // each for humidity and temperature

    // byte 0 = packet length
    // byte 1 = report type
    // byte 2 = report subtype - DHT_DATA or DHT_ERROR
    // byte 3 = pin number
    // byte 4 = dht type
    // byte 5 = humidity positivity flag 0=positive, 1= negative
    // byte 6 = temperature positivity flag 0=positive, 1= negative
    // byte 7 = humidity integer portion
    // byte 8 = humidity fractional portion
    // byte 9 = temperature integer portion
    // byte 10= temperature fractional portion

    byte report_message[11] = { 10, DHT_REPORT, DHT_DATA, 0, 0, 0, 0, 0, 0, 0, 0 };

    int rv;

    // are there any dhts to read?
    if (dht_index) {
        // is it time to do the read? This should occur every 2 seconds
        dht_current_millis = millis();
        if (dht_current_millis - dht_previous_millis > dht_scan_interval) {
            // update for the next scan
            dht_previous_millis = dht_current_millis;

            // read and report all the dht sensors
            for (int i = 0; i < dht_index; i++) {
                report_message[0] = 10;  //message length
                report_message[1] = DHT_REPORT;
                // error type in report_message[2] will be set further down
                report_message[3] = dhts[i].pin;
                report_message[4] = dhts[i].dht_type;
                // read the device
                if (dhts[i].dht_type == 22) {
                    rv = dhts[i].dht_sensor->read22(dhts[i].pin);
                } else {
                    rv = dhts[i].dht_sensor->read11(dhts[i].pin);
                }
                report_message[2] = (uint8_t)rv;

                // if rv is not zero, this is an error report
                if (rv) {
                    Serial.write(report_message, 11);
                    return;
                } else {
                    float j, f;
                    float humidity = dhts[i].dht_sensor->getHumidity();
                    if (humidity >= 0.0) {
                        report_message[5] = 0;
                    } else {
                        report_message[5] = 1;
                    }
                    f = modff(humidity, &j);
                    report_message[7] = (uint8_t)j;
                    report_message[8] = (uint8_t)(f * 100);

                    float temperature = dhts[i].dht_sensor->getTemperature();
                    if (temperature >= 0.0) {
                        report_message[6] = 0;
                    } else {
                        report_message[6] = 1;
                    }

                    f = modff(temperature, &j);

                    report_message[9] = (uint8_t)j;
                    report_message[10] = (uint8_t)(f * 100);
                    Serial.write(report_message, 11);
                }
            }
        }
    }
#endif
}


void run_steppers() {
#ifdef STEPPERS_ENABLED
    boolean running;
  long target_position;


  for (int i = 0; i < MAX_NUMBER_OF_STEPPERS; i++) {
    if (stepper_run_modes[i] == STEPPER_STOP) {
      continue;
    } else {
      steppers[i]->enableOutputs();
      switch (stepper_run_modes[i]) {
        case STEPPER_RUN:
          steppers[i]->run();
          running = steppers[i]->isRunning();
          if (!running) {
            byte report_message[3] = { 2, STEPPER_RUN_COMPLETE_REPORT, (byte)i };
            Serial.write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;
          }
          break;
        case STEPPER_RUN_SPEED:
          steppers[i]->runSpeed();
          break;
        case STEPPER_RUN_SPEED_TO_POSITION:
          running = steppers[i]->runSpeedToPosition();
          target_position = steppers[i]->targetPosition();
          if (target_position == steppers[i]->currentPosition()) {
            byte report_message[3] = { 2, STEPPER_RUN_COMPLETE_REPORT, (byte)i };
            Serial.write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;
          }
          break;
        default:
          break;
      }
    }
  }
#endif
}

void run_matrix() {
#if LED_MATRIX_SUPPORTED
    if (run_banner) {

        led_matrix_buffer_show(scroll, ontime);

        // Update scroll position.
        if (millis() >= t_prev + scroll_speed) {
            t_prev = millis();
            scroll += 1;                                               // Scroll to the left.
            if (scroll > 5 * strlen((char *)banner_text)) scroll = 0;  // restart
        }
    }
#endif
}
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/*                    Setup And Loop                                */
/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

void setup() {
    // set up features for enabled features
#ifdef ONE_WIRE_ENABLED
    features |= ONEWIRE_FEATURE;
#endif

#ifdef DHT_ENABLED
    features |= DHT_FEATURE;
#endif

#ifdef STEPPERS_ENABLED
    features |= STEPPERS_FEATURE;
#endif

#ifdef SPI_ENABLED
    features |= SPI_FEATURE;
#endif

#ifdef SERVO_ENABLED
    features |= SERVO_FEATURE;
#endif

#ifdef SONAR_ENABLED
    features |= SONAR_FEATURE;
#endif

#ifdef I2C_ENABLED
    features |= I2C_FEATURE;
#endif

#ifdef LED_MATRIX_SUPPORTED
    features |= LED_MATRIX_FEATURE;

    // Initialize LED matrix pins.
    for (uint8_t i = 0; i < led_matrix_pin_count; i++) {
        pinMode(led_matrix_pin_first + i, INPUT);  // all off
    }

#endif

#ifdef STEPPERS_ENABLED

    for (int i = 0; i < MAX_NUMBER_OF_STEPPERS; i++) {
    stepper_run_modes[i] = STEPPER_STOP;
  }
#endif

    init_pin_structures();

    Serial.begin(115200);
#ifdef ENABLE_STARTING_BANNER

    //Load scroll message.
    led_matrix_puts(led_matrix_buffer, sizeof(led_matrix_buffer), banner_text);
    // Ready...
    t_prev = millis();
#endif


    pinMode(13, OUTPUT);
    for (int i = 0; i < 4; i++) {
        digitalWrite(13, HIGH);
        delay(250);
        digitalWrite(13, LOW);
        delay(250);
    }
    run_banner = true;
    run_matrix();
}

void loop() {
    // keep processing incoming commands
    get_next_command();

    if (!stop_reports) {  // stop reporting
        scan_digital_inputs();
        scan_analog_inputs();

#ifdef SONAR_ENABLED
        if (sonar_reporting_enabled) {
            scan_sonars();
        }
#endif

#ifdef DHT_ENABLED
        scan_dhts();
#endif
#ifdef STEPPERS_ENABLED
        run_steppers();
#endif

#ifdef LED_MATRIX_SUPPORTED
        run_matrix();
#endif
    }
}
