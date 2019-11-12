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
# Global configuration
set (DRIVER_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/config")
include_directories(${DRIVER_CONFIG_DIR})

# Onboard LEDs
set (DRIVER_SYSTEM_LED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/SystemLED")
include_directories(${DRIVER_SYSTEM_LED_DIR})
set (DRIVER_SYSTEM_LED_SOURCES ${DRIVER_SYSTEM_LED_DIR}/SystemLED.cpp)

# WS2812 driver
set (DRIVER_WS2812_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/WS2812_drv/lib")
include_directories(${DRIVER_WS2812_DIR})
set (DRIVER_WS2812_SOURCES 
  ${DRIVER_WS2812_DIR}/SPI_uDMA_drv.c 
  ${DRIVER_WS2812_DIR}/WS2812_drv.c
  )

# Demonstrator LEDs (WS2812 stribes)
set (DRIVER_DEMO_LED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/DemoLED")
include_directories(${DRIVER_DEMO_LED_DIR})
set (DRIVER_DEMO_LED_SOURCES ${DRIVER_DEMO_LED_DIR}/DemoLED.cpp)

# I2C/TWI/Wire driver
set (DRIVER_WIRE_DIR "${ENERGIA_TIVA_BASE}/libraries/Wire")
include_directories(${DRIVER_WIRE_DIR})
set (DRIVER_WIRE_SOURCES 
  ${DRIVER_WIRE_DIR}/Wire.cpp
  )

# IMU driver from linorobot
set (DRIVER_IMU_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/imu")
include_directories(${DRIVER_IMU_DIR})
set (DRIVER_IMU_SOURCES 
  ${DRIVER_IMU_DIR}/I2Cdev.cpp
  ${DRIVER_IMU_DIR}/MPU9250.cpp
  )

# PID controller from linorobot
set (DRIVER_PID_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/pid")
include_directories(${DRIVER_PID_DIR})
set (DRIVER_PID_SOURCES 
  ${DRIVER_PID_DIR}/PID.cpp
  )

# Wheel motor driver
set (DRIVER_MOTOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/motor")
include_directories(${DRIVER_MOTOR_DIR})
set (DRIVER_MOTOR_SOURCES 
  ${DRIVER_MOTOR_DIR}/Motor.cpp
  )

# Wheel encoder driver
set (DRIVER_ENCODER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/encoder")
include_directories(${DRIVER_ENCODER_DIR})
set (DRIVER_ENCODER_SOURCES 
  ${DRIVER_ENCODER_DIR}/Encoder.cpp
  )

# Kinematics controller from linorobot
set (DRIVER_KINEMATICS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/kinematics")
include_directories(${DRIVER_KINEMATICS_DIR})
set (DRIVER_KINEMATICS_SOURCES 
  ${DRIVER_KINEMATICS_DIR}/Kinematics.cpp
  )

# TinyGPS++ GPS message decoder
set (DRIVER_TINY_GPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/TinyGPSPlus/src")
include_directories(${DRIVER_TINY_GPS_DIR})
set (DRIVER_TINY_GPS_SOURCES 
  ${DRIVER_TINY_GPS_DIR}/TinyGPS++.cpp
  )

# Opticar GPS driver
set (DRIVER_GPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/GPS")
include_directories(${DRIVER_GPS_DIR})
set (DRIVER_GPS_SOURCES 
  ${DRIVER_GPS_DIR}/GPS.cpp
  )

# Collect all driver sources
set (DRIVER_SOURCES 
  ${DRIVER_SYSTEM_LED_SOURCES}
  ${DRIVER_WS2812_SOURCES}
  ${DRIVER_DEMO_LED_SOURCES}
  ${DRIVER_WIRE_SOURCES}
  ${DRIVER_IMU_SOURCES}
  ${DRIVER_PID_SOURCES}
  ${DRIVER_MOTOR_SOURCES}
  ${DRIVER_ENCODER_SOURCES}
  ${DRIVER_KINEMATICS_SOURCES}
  ${DRIVER_TINY_GPS_SOURCES}
  ${DRIVER_GPS_SOURCES}
  )

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DROSSERIAL_BAUDRATE=115200)
add_definitions(-DTX_BUFFER_SIZE=1024)
add_definitions(-DRX_BUFFER_SIZE=1024)
add_definitions(-std=gnu++11)
add_definitions(-DF_CPU=80000000L)
add_definitions(-DUSE_MPU9250_IMU)
add_definitions(-DARDUINO=200)

# Generate target for TivaC
generate_tivac_firmware(
  SRCS 
    ${CORE_SOURCES}
    ${DRIVER_SOURCES}
    firmware.cpp
  BOARD tm4c123gxl
  STARTUP ${ENERGIA_TIVA_CORE_SOURCE}/startup_gcc.c
)