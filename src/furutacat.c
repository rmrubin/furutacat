
#include <furutacat.h>


// target and current angle, 12b, TDC = 2048 
uint16_t target   = 0;
uint16_t current  = 0;

// pid coefficients
uint16_t kp = 0;
uint16_t ks = 0;
uint16_t ki = 0;
uint16_t kd = 0;
int32_t up = 0;
int32_t us = 0;
int32_t ui = 0;
int32_t ud = 0;

// pid error data
int16_t error       = 0;
int16_t previous    = 0;
int32_t square      = 0;
int32_t integral    = 0;
int16_t derivative  = 0;

// motor pwm
int32_t  pwm      = 0;

// bools
uint32_t dir      = 0;
uint32_t run      = 0;
uint32_t relay    = 0;
uint32_t range    = 0;
uint32_t horizon  = 0;
uint32_t button		= 0;

// battery adc value
float battery = 0;

// floating point variable mirrors
float f_target      = 0;
float f_current     = 0;
float f_error       = 0;
float f_previous    = 0;
float f_square      = 0;
float f_integral    = 0;
float f_derivative  = 0;
float f_pwm         = 0;

// buffers
uint8_t vars[VAR_PACKET_BYTES] = {0};
uint8_t logpak[LOG_PACKET_BYTES] = {0};






void init_gpio(void)
{
  wiringPiSetupSys();
  digitalWrite(PIN_OUT,0);
}



void init_pi_hardware(void)
{
  system("sudo systemctl stop serial-getty@ttyAMA0.service");

  system("gpio export 23 out"); // PIN_RUN
  system("gpio export 24 in");  // PIN_IN
  system("gpio export 25 out"); // PIN_OUT
}



void init_uart(int* uart)
{
  *uart = serialOpen("/dev/ttyAMA0",230400);
  
  struct termios options;
  tcgetattr(*uart, &options);
  cfmakeraw(&options);
  
  options.c_cflag &= ~CSIZE;  // clear size fiels
  options.c_cflag |= CS8;     // 8 data bits
  options.c_cflag &= ~PARENB; // no parity
  options.c_cflag &= ~CSTOPB; // 1 stop bit
  
  options.c_cflag |= CLOCAL;
  options.c_cflag |= CREAD;
  options.c_cc[VTIME]=1; 
  options.c_cc[VMIN]=100;
  tcsetattr(*uart, TCSANOW, &options);
}



void deinit_uart(int* uart)
{
  printf("\n"); 
  
  serialClose(*uart);
  print_msg(UART_DEINIT);
}



void uart_tx(int* uart, char* data, int bytes)
{
  for(int i = 0; i < bytes; ++i)
  {
    serialPutchar(*uart, data[i]);
  }
}



void uart_rx(int* uart, char* data, int bytes)
{
  for(int i = 0; i < bytes; ++i)
  {
    data[i] = serialGetchar(*uart);
  }
}



void tx_cmd(int* uart, char* cmd)
{
  serialFlush(*uart);
  uart_tx(uart, cmd, 3);
}



void rx_ack(int* uart, char* ack)
{
  uart_rx(uart, ack, 3);
}



void print_msg(char* msg)
{
	printf("  %s\n", msg);
}



void print_appinfo(void)
{
	printf("\n");
	printf("  %s v%s\n\n", BINNAME, VERSION);
	printf("  %s\n", DESC);
	printf("    by %s\n", WRITER);
	printf("    on %s\n\n", DATE);
	printf("  %s\n", PROMPT_TEXT);
}



void print_exit(void)
{
	print_msg(EXIT_MESSAGE);
	printf("\n");
}



void print_help(void)
{
  printf("\n");
  
  print_msg(HELP_01);
  print_msg(HELP_02);
  print_msg(HELP_03);
  print_msg(HELP_04);
  print_msg(HELP_05);

  print_msg(HELP_06);
  print_msg(HELP_07);
  print_msg(HELP_08);
  print_msg(HELP_09);
}



char inb[CLI_BUFFER_SIZE] = {0};

char get_option(void)
{
	printf("\n  %s", PROMPT_CLI);
	
	fgets(inb, CLI_BUFFER_SIZE, stdin);
	return (inb[0]);
}



void toggle_relay(int* uart)
{
  printf("\n");
    
  tx_cmd(uart, "r!!");
    
  char ack[3];
  rx_ack(uart, ack);
    
  if((ack[0] == 'r') && (ack[1] == '0'))
  {
    switch(ack[2])
    {
      case '0':
        print_msg(RELAY_OPEN);
        break;
        
      case '1':
        print_msg(RELAY_CLOSED);
        break;
      
      default:
        print_msg(RELAY_BADACK);
    }
  }
  else print_msg(RELAY_BADACK);
}



void print_vars(int *uart)
{
  printf("\n");

  tx_cmd(uart, "v!!");
  
  uart_rx(uart, vars, VAR_PACKET_BYTES);

	unpack_vars();

  printf("  Kp:             ");
  printf("%d\n", kp);

  printf("  Ks:             ");
  printf("%d\n", ks);

  printf("  Ki:             ");
  printf("%d\n", ki);
  
  printf("  Kd:             ");
  printf("%d\n", kd);

  printf("  target[º]:      ");
  printf("%0.1f\n", f_target);

  printf("\n");

  printf("  current[º]:     ");
  printf("%.1f\n", f_current);

  printf("  error[º]:       ");
  printf("%.1f\n", f_error);

  printf("  integral[º*s]:  ");
  printf("%.1f\n", f_integral);

  printf("  diff[º/s]:      ");
  printf("%.1f\n", f_derivative);

  printf("\n");

  printf("  pwm[%]:         ");
  printf("%.2f\n", f_pwm);

  printf("\n");

  printf("  run:            ");
  printf("%d\n", run);

  printf("  button          ");
  printf("%d\n", button);

  printf("  relay:          ");
  printf("%d\n", relay);

  printf("  range:          ");
  printf("%d\n", range);

  printf("  horizon:        ");
  printf("%d\n", horizon);
  
  printf("\n");

  printf("  battery[V]:     ");
  printf("%.2f\n", battery);
}



void unpack_vars(void)
{
  static uint16_t tmp;
  
  unpack_2B_to_uint16(&vars[0], &vars[1], &kp);
  unpack_2B_to_uint16(&vars[2], &vars[3], &ks);
  unpack_2B_to_uint16(&vars[4], &vars[5], &ki);
  unpack_2B_to_uint16(&vars[6], &vars[7], &kd);
  unpack_2B_to_uint16(&vars[8], &vars[9], &target);
  uint16_to_deg(&target, &f_target);
  unpack_2B_to_uint16(&vars[10], &vars[11], &current);
  uint16_to_deg(&current, &f_current);
  unpack_2B_to_int16(&vars[12], &vars[13], &error);
  int16_to_deg(&error, &f_error);
  unpack_4B_to_int32(&vars[14], &vars[15], &vars[16], &vars[17], &integral);
  int32_to_deg(&integral, &f_integral);
  unpack_2B_to_int16(&vars[18], &vars[19], &derivative);
  int16_to_deg(&derivative, &f_derivative);
  unpack_2B_to_int32(&vars[20], &vars[21], &pwm);

  f_integral *= INTEGRAL_CONV_K;
  f_derivative *= DERIVATIVE_CONV_K;

  if (vars[24] & (1 << 0))  dir= 1;
  else
  {
    pwm *= -1;
    dir = 0;
  }

  int32_to_percent_pwm(&pwm, &f_pwm);

  unpack_2B_to_uint16(&vars[22], &vars[23], &tmp);
  
	battery = tmp * BATTERY_CONV_K;

  if (vars[24] & (1 << 1)) run = 1;
  else run = 0;

  if (vars[24] & (1 << 2)) relay = 1;
  else relay = 0;

  if (vars[24] & (1 << 3)) range = 1;
  else range = 0;

  if (vars[24] & (1 << 4)) horizon = 1;
  else horizon = 0;

  if (vars[24] & (1 << 5)) button = 1;
  else button = 0;
}



void unpack_log(void)
{
  unpack_2B_to_uint16(&logpak[0], &logpak[1], &current);
  uint16_to_deg(&current, &f_current);
  unpack_2B_to_int32(&logpak[2], &logpak[3], &pwm);
  int32_to_percent_pwm(&pwm, &f_pwm); 
}


void unpack_2B_to_uint16(uint8_t* high, uint8_t* low, uint16_t* out)
{
  static union variable_array
  {
    uint16_t  data;
    uint8_t   byte[2];
  } va;

  va.byte[0] = *high;
  va.byte[1] = *low;
  *out = va.data;
}




void unpack_2B_to_int16(uint8_t* high, uint8_t* low, int16_t* out)
{
  static union variable_array
  {
    int16_t  data;
    uint8_t   byte[2];
  } va;

  va.byte[0] = *high;
  va.byte[1] = *low;
  *out = va.data;
}



void unpack_2B_to_int32(uint8_t* high, uint8_t* low, int32_t* out)
{
  static union variable_array
  {
    int16_t 	data; 
    uint8_t   byte[2];
  } va;

  va.byte[0] = *high;
  va.byte[1] = *low;
  *out = (int32_t)va.data;
}



void unpack_4B_to_int32(uint8_t* high, uint8_t* hmid, uint8_t* lmid, uint8_t* low, int32_t* out)
{
  static union variable_array
  {
    int32_t   data;
    uint8_t   byte[4];
  } va;

  va.byte[0] = *high;
  va.byte[1] = *hmid;
  va.byte[2] = *lmid;
  va.byte[3] = *low;
  *out = va.data;
}



void int16_to_deg(int16_t* in, float* degrees)
{
  *degrees = *in * DEGREE_CONV_K;
}



void uint16_to_deg(uint16_t* in , float* degrees)
{
  *degrees = *in * DEGREE_CONV_K;
}



void int32_to_deg(int32_t* in , float* degrees)
{
  *degrees = *in * DEGREE_CONV_K;
}



void int32_to_percent_pwm(int32_t* in, float* percent_pwm)
{
    *percent_pwm = *in * PERCENT_PWM_CONV_K;
}



void edit_var(int* uart, char var_ch, char* msg, uint16_t min, uint16_t max)
{
  printf("\n");

  uint16_t tmp = getnum(msg, min, max);
 
  if(var_ch = 't') tmp = (uint16_t)round((double)(tmp * BIN_DEG_CONV_K));
 
  uint8_t tmpb[3] = {var_ch};
  tmpb[1] = (uint8_t) (tmp >> 8);
  tmpb[2] = (uint8_t) (tmp &= 0x00FF);
  
  tx_cmd(uart, tmpb);
  
  uint8_t ack[3];
  rx_ack(uart, ack);
  
  int fail = 0;
  for (int i = 0; i < 3; ++i)
  {
    if (tmpb[i] != ack[i]) ++fail;
  }
  if (fail) print_msg(EDIT_FAIL);
  else print_msg(EDIT_PASS);
}



void start_monitor(int* uart)
{
  uint16_t refresh = getnum(MONITOR_PROMPT, 0, 65355);
  for(int i = 0; i < refresh; ++i)
  {
    system("clear");
    print_vars(uart);
    usleep(200000);
  }
}



void start_logging(int* uart)
{
  FILE* fout;
  FILE* fin;
  
  char dir[]      = OUTPUT_DIR;
  char ext_bin[]  = FILE_EXT_BIN;
  char ext_dat[]  = FILE_EXT_DAT;
  char ext_plot[] = FILE_EXT_PLOT;
  char date_prefix[DATE_PREFIX_SIZE];
  char filename[FILENAME_SIZE];

  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime(date_prefix,DATE_PREFIX_SIZE,DATE_FORMAT,timeinfo);
  
  strcpy(filename,dir);
	strcat(filename,date_prefix);
  strcat(filename,ext_bin);
 
  fout = fopen(filename,"wb");

  uint16_t packets = getnum(LOG_PROMPT, 0, 65355);

  uint8_t tmpb[3] = {'l'};
  tmpb[1] = (uint8_t) (packets >> 8);
  tmpb[2] = (uint8_t) (packets & 0x00FF);

  tx_cmd(uart, tmpb);

  uart_rx(uart, vars, VAR_PACKET_BYTES);
  unpack_vars();

  _NL;
	print_msg(LOG_START);

  char ch;
	uint32_t bytes = packets * LOG_PACKET_BYTES;

  for(int i = 0; i < bytes; ++i)
	{
    do
    {
      ch = serialGetchar(*uart);  
    }
    while(ch == -1);

    fputc(ch,fout);
  }
  
  serialFlush(*uart);
  fclose(fout);

  print_msg(LOG_WRITE);

  strcpy(filename,dir);
  strcat(filename,date_prefix);
  strcat(filename,ext_bin);

  fin = fopen(filename,"rb");

  strcpy(filename,dir);
  strcat(filename,date_prefix);
  strcat(filename,ext_dat);

  fout = fopen(filename,"w");
  fprintf(fout, "# Kp=%i Ki=%i Kd=%i target=%.1f run=%i relay=%i battery=%.2f\n", kp, ki, kd, f_target, run, relay, battery);
  fprintf(fout, "# time target current pwm\n");
  float period = (1.0/SAMPLE_RATE_HZ);

  for (int i = 0; i < packets; ++i)
  {
    for(int j = 0; j < LOG_PACKET_BYTES; ++j) logpak[j] = fgetc(fin);
    unpack_log();
    fprintf(fout, "%.4f %.1f %.1f %f\n", (i * period), f_target, f_current, f_pwm);
  }
  
  fclose(fin);
  fclose(fout);

  print_msg(LOG_GRAPH);

  strcpy(filename,GNUPLOT_CMD1);
  strcat(filename,OUTPUT_DIR);
  strcat(filename,date_prefix);
  strcat(filename,FILE_EXT_DAT);
  strcat(filename,GNUPLOT_CMD2);
  strcat(filename,OUTPUT_DIR);
  strcat(filename,date_prefix);
  strcat(filename,FILE_EXT_PLOT);
  strcat(filename,GNUPLOT_CMD3);
  strcat(filename,GNUPLOT_SCRIPT);

  system(filename);

  print_msg(LOG_DONE);
}



uint16_t getnum(char* msg, uint16_t min, uint16_t max)
{
  uint32_t num;

  while(1)
  {
    print_msg(msg);
    printf("\n  %s", PROMPT_NUM);

    fgets(inb, CLI_BUFFER_SIZE, stdin);
		
    num = (uint32_t)strtoul(inb, NULL, 0);
    if((num >= min) && (num <= max)) return((uint16_t)num);
  }
}



void pwm_enable(void)
{
	printf("\n");

	digitalWrite(PIN_RUN,1);
	print_msg(PWM_ENABLE);
}



void pwm_disable()
{
	printf("\n");

	digitalWrite(PIN_RUN,0);
	print_msg(PWM_DISABLE);
}



void test_packet(int* uart)
{
  printf("\n");

  tx_cmd(uart, "wtf");   
  print_msg(TEST_PACKET);
}



