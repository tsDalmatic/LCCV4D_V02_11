#ifndef CLASSB_GCC_COMPAT_H
#define CLASSB_GCC_COMPAT_H

/* The Class-B sources were originally compiled with MDK, which supplied
 * these declarations implicitly. GCC 14+ requires explicit prototypes. */
#include "../STM32_SelfTest_Library/inc/main.h"
#include "../STM32_SelfTest_Library/inc/stm32fxx_STLcpu.h"

/* The supplied CPU-test objects follow Keil's register convention and
 * overwrite registers that GCC requires callees to preserve. V02.11 still
 * calls the original routine names directly, so redirect those calls through
 * the preserving assembly wrappers without changing the firmware sources. */
#if defined(__GNUC__) && !defined(__ARMCC_VERSION)
ErrorStatus STL_StartUpCPUTest_GCC(void);
ErrorStatus STL_RunTimeCPUTest_GCC(void);
#define STL_StartUpCPUTest() STL_StartUpCPUTest_GCC()
#define STL_RunTimeCPUTest() STL_RunTimeCPUTest_GCC()
#endif

void Startup_Copy_Handler(void);
void __libc_init_array(void);
int main(void);

#endif
