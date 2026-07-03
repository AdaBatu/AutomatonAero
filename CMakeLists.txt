cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME AutomatonAero)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Compile selected settings from the human-editable JSON configuration.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/flight_config.json")
file(READ "${CMAKE_SOURCE_DIR}/flight_config.json" FLIGHT_CONFIG_JSON)
string(JSON FLIGHT_BATTERY_VOLTAGE_V GET
    "${FLIGHT_CONFIG_JSON}" voltage_sensing battery_voltage_v)
string(JSON FLIGHT_SYSTEM_ARMED GET
    "${FLIGHT_CONFIG_JSON}" airframe system_armed)
string(JSON FLIGHT_MAX_RC_ATTITUDE_DEGREES GET
    "${FLIGHT_CONFIG_JSON}" airframe max_rc_attitude_degrees)
string(JSON FLIGHT_ROLL_KP GET "${FLIGHT_CONFIG_JSON}" pid roll kp)
string(JSON FLIGHT_ROLL_KI GET "${FLIGHT_CONFIG_JSON}" pid roll ki)
string(JSON FLIGHT_ROLL_KD GET "${FLIGHT_CONFIG_JSON}" pid roll kd)
string(JSON FLIGHT_PITCH_KP GET "${FLIGHT_CONFIG_JSON}" pid pitch kp)
string(JSON FLIGHT_PITCH_KI GET "${FLIGHT_CONFIG_JSON}" pid pitch ki)
string(JSON FLIGHT_PITCH_KD GET "${FLIGHT_CONFIG_JSON}" pid pitch kd)
string(JSON FLIGHT_BARO_ZERO_SAMPLES GET
    "${FLIGHT_CONFIG_JSON}" barometer_ms5611 startup_altitude_zero_samples)
string(JSON FLIGHT_CONFIG_RX_INTERVAL_MS GET
    "${FLIGHT_CONFIG_JSON}" lora configuration_receive_window_interval_ms)
string(JSON FLIGHT_CONFIG_RX_DURATION_MS GET
    "${FLIGHT_CONFIG_JSON}" lora configuration_receive_window_duration_ms)
if(FLIGHT_SYSTEM_ARMED)
    set(FLIGHT_SYSTEM_ARMED_VALUE 1)
else()
    set(FLIGHT_SYSTEM_ARMED_VALUE 0)
endif()
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated")
configure_file(
    "${CMAKE_SOURCE_DIR}/Core/Inc/flight_config_generated.h.in"
    "${CMAKE_BINARY_DIR}/generated/flight_config_generated.h"
    @ONLY
)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/mpu6050.c
    Core/Src/ms5611.c
    Core/Src/gps_nmea.c
    Core/Src/sx1278.c
    Core/Src/kalman.c
    Core/Src/pid.c
    Core/Src/servo_esc.c
    Core/Src/adc_sensors.c
    Core/Src/telemetry.c
    Core/Src/rc_input.c
    Core/Src/serial_telemetry.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    "${CMAKE_BINARY_DIR}/generated"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
