#ifndef __CLI_UART__
#define __CLI_UART__
//#include "defs.h"


void log_print(char* fmt, ...);

#define LOGI(fmt, args...) log_print(fmt, ##args)



#endif /*__CLI_UART__*/
