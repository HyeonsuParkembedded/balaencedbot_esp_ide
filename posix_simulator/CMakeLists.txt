# BalanceBot POSIX Simulator CMakeLists.txt
# FreeRTOS POSIX port를 사용하여 BalanceBot을 Linux에서 시뮬레이션

cmake_minimum_required(VERSION 3.16)

# Project configuration
project(BalanceBot_POSIX_Simulator C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter")
set(CMAKE_C_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")

# Include directories
include_directories(
    # Mock FreeRTOS includes (for IDE and Windows compatibility)
    ${CMAKE_CURRENT_SOURCE_DIR}/mock_freertos
    
    # Mock layer includes  
    ${CMAKE_CURRENT_SOURCE_DIR}
    
    # Original project includes
    ${CMAKE_CURRENT_SOURCE_DIR}/logic
    ${CMAKE_CURRENT_SOURCE_DIR}/input
    ${CMAKE_CURRENT_SOURCE_DIR}/output
    ${CMAKE_CURRENT_SOURCE_DIR}/system
)

# Preprocessor definitions
add_definitions(
    -DprojCOVERAGE_TEST=0
    -DRUN_FREERTOS_ON_CORE=0
    -DBALANCEBOT_POSIX_SIM=1
)

# Mock layer source files  
set(MOCK_SOURCES
    mock_esp.c
    mock_bsw.c
    mock_freertos.c
)

# BalanceBot logic source files (platform independent)
set(BALANCEBOT_LOGIC_SOURCES
    logic/pid_controller.c
    logic/kalman_filter.c
)

# BalanceBot system files (adapted for POSIX)
set(BALANCEBOT_SYSTEM_SOURCES
    system/protocol.c
    system/config_manager_mock.c
)

# BalanceBot output files (mocked)
set(BALANCEBOT_OUTPUT_SOURCES
    output/ble_controller_mock.c
)

# BalanceBot input/output (mocked)
set(BALANCEBOT_IO_SOURCES
    input/imu_sensor_mock.c
    mock_components.c
)

# Main application
set(APP_SOURCES
    main_posix.c
)

# All source files (excluding real FreeRTOS for now)
set(ALL_SOURCES
    ${MOCK_SOURCES}
    ${BALANCEBOT_LOGIC_SOURCES}
    ${BALANCEBOT_SYSTEM_SOURCES}
    ${BALANCEBOT_OUTPUT_SOURCES}
    ${BALANCEBOT_IO_SOURCES}
    ${APP_SOURCES}
)

# Create executable
add_executable(balancebot_posix ${ALL_SOURCES})

# Link libraries
target_link_libraries(balancebot_posix 
    pthread
    m  # math library
)

# Installation
install(TARGETS balancebot_posix 
    RUNTIME DESTINATION bin
)

# Custom target for running the simulator
add_custom_target(run_sim
    COMMAND ./balancebot_posix
    DEPENDS balancebot_posix
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running BalanceBot POSIX Simulator"
)

# Print configuration info
message(STATUS "BalanceBot POSIX Simulator Configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Using Mock FreeRTOS Headers")
message(STATUS "  Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  C Flags: ${CMAKE_C_FLAGS}")