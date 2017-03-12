
#include <stdarg.h>

//void putc(unsigned);
//void puts(char *);
//static void xtoa(unsigned long x, const unsigned long *dp);
//static void puth(unsigned n);
void uart_printf(char *format, ...);
char readTemp(void);
void initTemp(void);
unsigned int hex2int(char *a);
