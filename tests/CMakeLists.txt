cmake_minimum_required(VERSION 3.14)
project(FELPS_TESTS)

set(TEST_DIR "./")
set(REL_FIRMWARE_DIR "../firmware")
set(REL_EVENTS_LIB_DIR "${REL_FIRMWARE_DIR}/src/events")
set(REL_OS_LIB_DIR "${REL_FIRMWARE_DIR}/src/os")

add_executable(felps-tests)
target_compile_definitions(felps-tests PUBLIC
    FELPS_TEST_BUILD
)
target_sources(felps-tests PUBLIC
    ${TEST_DIR}/test_events.cpp
    ${TEST_DIR}/test_serial_handler.cpp
    ${TEST_DIR}/main.cpp
)
target_include_directories(felps-tests PUBLIC
    ${REL_FIRMWARE_DIR}/
    ${REL_EVENTS_LIB_DIR}/
    ${REL_OS_LIB_DIR}/
)