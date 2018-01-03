cmake_minimum_required(VERSION 2.8.3)
project(tiva)

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
#add_definitions(-DLED_HEARTBEAT)
#add_definitions(-DLED_COMM)
add_definitions(-DROSSERIAL_BAUDRATE=115200)
add_definitions(-DTX_BUFFER_SIZE=1024)
add_definitions(-DRX_BUFFER_SIZE=1024)

# Generate target for TivaC
generate_tivac_firmware(
  SRCS 
    ECU.cpp
    constants.h
    drivererror.c
    config.cpp
    config.h
    led.c
    led.h
    kinematic.cpp
    kinematic.h
    ustdlib.c
    drivers/GPS.c
    drivers/GPS.h
    drivers/i2c.c
    drivers/i2c.h
    drivers/motors.c
    drivers/motors.h
    drivers/mpu9250.cpp
    drivers/mpu9250.h
    drivers/SPI_uDMA_drv.c
    drivers/SPI_uDMA_drv.h
    drivers/WS2812_drv.c
    drivers/WS2812_drv.h
  BOARD tm4c123gxl
)