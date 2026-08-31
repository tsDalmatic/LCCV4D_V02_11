cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

project(V4D-V02_11 LANGUAGES C ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(HAL_SOURCES
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_crc.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_crc_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_iwdg.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rtc.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rtc_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart_ex.c
    Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_wwdg.c
)

set(CLASS_B_SOURCES
    STM32_SelfTest_Library/src/stm32fxx_STLclockrun.c
    STM32_SelfTest_Library/src/stm32fxx_STLclockstart.c
    STM32_SelfTest_Library/src/stm32fxx_STLcrc32Run.c
    STM32_SelfTest_Library/src/stm32fxx_STLmain.c
    STM32_SelfTest_Library/src/stm32fxx_STLstartup.c
    STM32_SelfTest_Library/src/stm32fxx_STLtranspRam.c
)

# The MDK project compiles these files with the Class-B version of main.h.
# Force that header first because the application also has an unrelated main.h.
set_source_files_properties(${CLASS_B_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-include;${CMAKE_SOURCE_DIR}/cmake/classb_gcc_compat.h"
)

file(GLOB APPLICATION_SOURCES CONFIGURE_DEPENDS Src/*.c)
# This experimental source is present in the folder but is not part of the
# firmware target.
list(FILTER APPLICATION_SOURCES EXCLUDE REGEX "/spec_traffic_light\\.c$")

set(CLASS_B_OBJECT_DIR ${CMAKE_SOURCE_DIR}/STM32_SelfTest_Library/lib/armcc)
set(CLASS_B_OBJECTS
    ${CLASS_B_OBJECT_DIR}/stm32f0xx_stlcpustartkeil.o
    ${CLASS_B_OBJECT_DIR}/stm32f0xx_stlcpurunkeil.o
    ${CLASS_B_OBJECT_DIR}/stm32fxx_stlrammcmxkeil.o
)

foreach(object IN LISTS CLASS_B_OBJECTS)
    if(NOT EXISTS "${object}")
        message(FATAL_ERROR "Required prebuilt Class-B assembly object is missing: ${object}")
    endif()
endforeach()

add_executable(${PROJECT_NAME}
    ${APPLICATION_SOURCES}
    ${HAL_SOURCES}
    ${CLASS_B_SOURCES}
    ${CLASS_B_OBJECTS}
    cmake/classb_cpu_wrappers.s
    cmake/startup_stm32f091xc.s
    cmake/startup_copy.c
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    LINK_DEPENDS ${CMAKE_SOURCE_DIR}/STM32F091VC_CLASSB_FLASH.ld
    SUFFIX ".elf"
)

target_include_directories(${PROJECT_NAME} PRIVATE
    Inc
    Drivers/STM32F0xx_HAL_Driver/Inc
    Drivers/STM32F0xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F0xx/Include
    Drivers/CMSIS/Include
    STM32_SelfTest_Library/inc
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F091xC
    $<$<CONFIG:Release>:CLASS_B_RELEASE_BUILD>
)

target_link_libraries(${PROJECT_NAME} PRIVATE m)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} --gap-fill 0xFF -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.unpatched.bin
    COMMAND powershell -NoProfile -ExecutionPolicy Bypass
            -File ${CMAKE_SOURCE_DIR}/cmake/compute_classb_crc.ps1
            -Binary ${PROJECT_NAME}.unpatched.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} --gap-fill 0xFF -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_COMMAND} -E rm -f ${PROJECT_NAME}.unpatched.bin
)
