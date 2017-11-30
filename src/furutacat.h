#ifndef FURUTACAT_H
#define FURUTACAT_H

#define SAMPLE_RATE_HZ 1024

#define PIN_RUN 23
#define PIN_OUT 24
#define PIN_IN  25

#define CLI_BUFFER_SIZE   1024
#define UART_RXB_SIZE     1024

#define VAR_PACKET_BYTES  25
#define LOG_PACKET_BYTES  4
#define OUTPUT_DIR        "/home/pi/www/"
#define DATE_PREFIX_SIZE  64
#define FILENAME_SIZE     1024 // also used for gnuplot cmd
#define DATE_FORMAT       "%Y%m%d%H%M%S"
#define FILE_EXT_BIN      ".bin"
#define FILE_EXT_DAT      ".txt"
#define FILE_EXT_PLOT     ".png"

#define GNUPLOT_CMD1 "gnuplot -e 'filename_data=\""
#define GNUPLOT_CMD2 "\";filename_png=\""
#define GNUPLOT_CMD3 "\"' "
#define GNUPLOT_SCRIPT "/home/pi/.furutacat_gnuplot"

#define BATTERY_CONV_K 0.006864
#define DEGREE_CONV_K 0.08789
#define BIN_DEG_CONV_K 11.38
#define PERCENT_PWM_CONV_K 0.01686
#define INTEGRAL_CONV_K 1.0/1024.0
#define DERIVATIVE_CONV_K 1024.0

#define _NL printf("\n")



#include <text_defines.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include <termios.h>



void init_gpio(void);
void init_pi_hardware(void);
void init_uart(int* uart);
void deinit_uart(int* uart);
void uart_tx(int* uart, char* data, int bytes);
void uart_rx(int* uart, char* data, int bytes);
void tx_cmd(int* uart, char* cmd);
void rx_ack(int* uart, char* ack);

void print_msg(char* msg);
void print_appinfo(void);
void print_exit(void);  
void print_help(void);



char get_option(void);
void edit_var(int* uart, char var_ch, char* msg, uint16_t min, uint16_t max);
uint16_t getnum(char* msg, uint16_t min, uint16_t max);

void pwm_enable(void);
void pwm_disable(void);
void toggle_relay(int* uart);

void unpack_vars(void);
void unpack_log(void);
void unpack_2B_to_uint16(uint8_t* high, uint8_t* low, uint16_t* out);
void unpack_2B_to_int16(uint8_t* high, uint8_t* low, int16_t* out);
void unpack_2B_to_int32(uint8_t* high, uint8_t* low, int32_t* out);
void unpack_4B_to_int32(uint8_t* high, uint8_t* hmid, uint8_t* lmid, uint8_t* low, int32_t* out);

void int16_to_deg(int16_t* in, float* degrees);
void uint16_to_deg(uint16_t* in , float* degrees);
void int32_to_deg(int32_t* in, float* degrees);
void int32_to_percent_pwm(int32_t* in , float* percent_pwm);

void print_vars(int* uart);
void start_logging(int* uart);
void start_monitor(int* uart);
void test_packet(int* uart);



#endif

