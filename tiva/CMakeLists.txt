cmake_minimum_required(VERSION 2.8.3)
project(tiva)

# Prepare energia libraries for tivac
set (ENERGIA_TIVA_BASE "${CMAKE_CURRENT_SOURCE_DIR}/tivac-core")
set (ENERGIA_TIVA_CORE_SOURCE "${ENERGIA_TIVA_BASE}/cores/tivac")

include_directories("${ENERGIA_TIVA_CORE_SOURCE}")
include_directories("${ENERGIA_TIVA_BASE}/variants/EK-TM4C123GXL")

set (CORE_SOURCES
    ${ENERGIA_TIVA_CORE_SOURCE}/HardwareSerial.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/IPAddress.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/MACAddress.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/Print.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/Stream.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/Tone.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/WInterrupts.c
    ${ENERGIA_TIVA_CORE_SOURCE}/WMath.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/WString.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/itoa.c
    ${ENERGIA_TIVA_CORE_SOURCE}/main.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/new.cpp
    ${ENERGIA_TIVA_CORE_SOURCE}/random.c
    ${ENERGIA_TIVA_CORE_SOURCE}/wiring.c
    ${ENERGIA_TIVA_CORE_SOURCE}/wiring_analog.c
    ${ENERGIA_TIVA_CORE_SOURCE}/wiring_digital.c
    ${ENERGIA_TIVA_CORE_SOURCE}/wiring_pulse.c
    ${ENERGIA_TIVA_CORE_SOURCE}/wiring_shift.c
)

# Prepare driver libraries
set (DRIVER_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/config")
include_directories(${DRIVER_CONFIG_DIR})

set (DRIVER_SYSTEM_LED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/SystemLED")
include_directories(${DRIVER_SYSTEM_LED_DIR})
set (DRIVER_SYSTEM_LED_SOURCES ${DRIVER_SYSTEM_LED_DIR}/SystemLED.cpp)

set (DRIVER_SOURCES 
  ${DRIVER_SYSTEM_LED_SOURCES})

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
#add_definitions(-DLED_HEARTBEAT)
#add_definitions(-DLED_COMM)
add_definitions(-DROSSERIAL_BAUDRATE=115200)
add_definitions(-DTX_BUFFER_SIZE=1024)
add_definitions(-DRX_BUFFER_SIZE=1024)
add_definitions(-std=gnu++11)
add_definitions(-DF_CPU=80000000L)

# Generate target for TivaC
generate_tivac_firmware(
  SRCS 
    ${CORE_SOURCES}
    ${DRIVER_SOURCES}
    firmware.cpp
    #ECU.cpp
    #constants.h
    #drivererror.c
    #config.cpp
    #config.h
    #led.c
    #led.h
    #kinematic.cpp
    #kinematic.h
    #ustdlib.c
    #drivers/GPS.c
    #drivers/GPS.h
    #drivers/i2c.c
    #drivers/i2c.h
    #drivers/motors.c
    #drivers/motors.h
    #drivers/mpu9250.cpp
    #drivers/mpu9250.h
    #drivers/SPI_uDMA_drv.c
    #drivers/SPI_uDMA_drv.h
    #drivers/WS2812_drv.c
    #drivers/WS2812_drv.h
  BOARD tm4c123gxl
  STARTUP ${ENERGIA_TIVA_CORE_SOURCE}/startup_gcc.c
)