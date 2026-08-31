#ifndef CLASSB_GCC_COMPAT_H
#define CLASSB_GCC_COMPAT_H

/* The Class-B sources were originally compiled with MDK, which supplied
 * these declarations implicitly. GCC 14+ requires explicit prototypes. */
#include "../STM32_SelfTest_Library/inc/main.h"

void Startup_Copy_Handler(void);
void __libc_init_array(void);
int main(void);

#endif
