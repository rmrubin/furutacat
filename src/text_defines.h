#ifndef TEXT_DEFINES_H
#define TEXT_DEFINES_H

#define BINNAME         "furutacat"
#define VERSION         "0.04"
#define DESC            "furutacat rotational inverted pendulum control software"
#define DATE            "2017-11-30"
#define WRITER          "randy rubin for MECA482, California State University, Chico"

#define PROMPT_TEXT     "FCAT: enter a command, ! to init gpio after reboot, h for help"
#define PROMPT_CLI      "? "
#define PROMPT_NUM      "# "

#define INVALID_CMD     "FCAT: !!! invalid command, you can try again"
#define INVALID_NUM     "FCAT: !!! not a valid number, you can try again"

#define EXIT_MESSAGE    "FCAT: exiting, bye"

#define UART_DEINIT     "UART: disconnected"
#define UART_RX_ERROR   "UART: RX ERROR"
#define UART_TX_ERROR   "UART: TX ERROR"

#define EDIT_FAIL       "ACK: !!! NOT OK !!! variable STATE UNKNOWN"
#define EDIT_PASS       "ACK: variable updated okay"

#define PROMPT_KP       "FCAT: enter new proportional gain, Kp, 0-65355"
#define PROMPT_KS       "FCAT: enter new squared gain, Ks, 0-65355"
#define PROMPT_KI       "FCAT: enter new integral gain, Ki, 0-65355"
#define PROMPT_KD       "FCAT: enter new differential gain, Kd"
#define PROMPT_TARGET   "FCAT: enter new target angle, 0-360, 180 is up"

#define RELAY_CLOSED    "ACK: !!! relay CLOSED, motor POWERED"
#define RELAY_OPEN      "ACK: relay open, motor unpowered"
#define RELAY_BADACK    "ACK: !!! NOT OK !!! current relay STATE UNKNOWN"

#define PWM_ENABLE      "RUN: !!! motor pwm ENABLED"
#define PWM_DISABLE     "STOP: motor pwm disabled"

#define LOG_PROMPT      "FCAT: enter number of log samples, 0-65355, at 1024 Hz"
#define LOG_START       "FCAT: writing log data to binary file..."
#define LOG_WRITE       "FCAT: converting binary data to CSV file..."
#define LOG_GRAPH       "FCAT: creating graphs with gnuplot..."
#define LOG_DONE        "FCAT: logging complete"

#define MONITOR_PROMPT  "FCAT: enter how many times to refresh at 5 Hz"

#define TEST_PACKET     "UART: sent test packet"
#define TEST_OK         "ACK: test passed, rx == tx"
#define TEST_BADACK     "ACK: !!! NOT OK !!! test failed, rx != tx"

#define HELP_01 "FCAT: enter a command from the menu and press enter..."
#define HELP_02 " "
#define HELP_03 "   e - enable pwm                p - edit Kp (proportional gain)"
#define HELP_04 "   x - disable pwm               s - edit Ks (squared gain)     "
#define HELP_05 "   r - toggle relay              i - edit Ki (integral gain)    "              
#define HELP_06 "   ! - re-init pi hardware       d - edit Kd (derivative gain)  "
#define HELP_07 "   v - display variables         t - edit target angle          "
#define HELP_08 "   m - monitor variables         h - display this help menu     "
#define HELP_09 "   l - start logging             q - quit futuracat             "




#endif

