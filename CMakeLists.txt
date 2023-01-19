cmake_minimum_required(VERSION 3.16)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32_gcc.cmake)

project(l432kc-can-interceptor VERSION 1.0.0 LANGUAGES C ASM)

set(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src
)

add_link_options(-Wl,--print-memory-usage)

set(HAL_COMP_LIST RCC GPIO CORTEX CAN UART USART PWR)
set(CMSIS_COMP_LIST "")

list(APPEND HAL_COMP_LIST STM32L4)
list(APPEND CMSIS_COMP_LIST STM32L4)

find_package(HAL COMPONENTS "${HAL_COMP_LIST}" REQUIRED)
find_package(CMSIS COMPONENTS "${CMSIS_COMP_LIST}" REQUIRED)

set(MAIN_SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c)


add_executable(l432kc-can-interceptor 
    ${MAIN_SOURCE_FILE} 
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/stm32l4xx_hal_conf.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32l4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32l4xx_it.c
    )

target_include_directories(l432kc-can-interceptor PRIVATE ${INCLUDE_DIRS})

target_link_libraries(l432kc-can-interceptor
        HAL::STM32::L4::RCCEx
        HAL::STM32::L4::PWREx
        HAL::STM32::L4::GPIO
        HAL::STM32::L4::CAN
        HAL::STM32::L4::CORTEX
        HAL::STM32::L4::UART
        CMSIS::STM32::L432KC
        STM32::NoSys
    )

stm32_print_size_of_target(l432kc-can-interceptor)
