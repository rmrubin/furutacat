
#include <furutacat.h>

int main(int argc, char** argv)
{
  int uart;
  
  init_gpio();
  init_uart(&uart);

  print_appinfo();
  
  int loop = 1;
  while(loop)
  {
    switch(get_option())
    {
      case 'h':
      case 'H':
        print_help();
        break;

      case 'e':
      case 'E':
        pwm_enable();
        break;

      case 'x':
      case 'X':
        pwm_disable();
        break;
            
      case 'r':
      case 'R':
        toggle_relay(&uart);
        break;

			case 'p':
      case 'P':
        edit_var(&uart, 'p', PROMPT_KP, 0, 65355);
        break;

      case 's':
      case 'S':
        edit_var(&uart, 's', PROMPT_KS, 0, 65355);
        break;

      case 'i':
      case 'I':
        edit_var(&uart, 'i', PROMPT_KI, 0, 65355);
        break;

      case 'd':
      case 'D':
        edit_var(&uart, 'd', PROMPT_KD, 0, 65355);
        break;

      case 't':
      case 'T':
        edit_var(&uart, 't', PROMPT_TARGET, 0, 4095);
        break;

      case 'v':
      case 'V':
        print_vars(&uart);
        break;

      case 'l':
      case 'L':
        start_logging(&uart);
        break;

      case 'm':
      case 'M':
        start_monitor(&uart);
        break;

      case 'q':
      case 'Q':
        loop = 0;
        break;

      case '!':
        init_pi_hardware();
        break;

      case 'w':
      case 'W':
        test_packet(&uart);
        break;
      
      default:
        printf("\n");
        print_msg(INVALID_CMD);
    }
  }
 
  deinit_uart(&uart);
  print_exit();
  return(0);
}

