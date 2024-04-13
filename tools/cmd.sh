#!/bin/bash

SKETCH_INPUT="/workspace/felps"
BUILD_OUTPUT="/workspace/felps/output"

SUPPRESSED_WARNINGS="-Wno-register"
VERBOSE_DEBUG_PRINTS="-DVERBOSE_DEBUG" # This enables verbose debug prints in the code
FQBN="STMicroelectronics:stm32:GenC0"
BOARD_OPTIONS="pnum=GENERIC_C031C6TX"
#FQBN="STMicroelectronics:stm32:GenG0"
#BOARD_OPTIONS="pnum=GENERIC_G050C8TX"

# Display help function
function display_help {
    echo "---------------------------------------------------------------------"
    echo "Usage: $0 [options...] {help,build,clean,analyze}"
    echo "Options:"
    echo "-h,--help     display this message"
    echo "Commands:"
    echo "help:         display this message"
    echo "build:        build FELPS firmware image"
    echo "clean:        clean the output directory"
    echo "analyze:      analyze firmware image and print a report to stdout"
    echo "---------------------------------------------------------------------"
}

# Build firmware
function build_firmware {
    arduino-cli compile \
    --log \
    --clean \
    --build-property compiler.cpp.extra_flags="$SUPPRESSED_WARNINGS" \
    --build-property build.extra_flags="$VERBOSE_DEBUG_PRINTS" \
    --warnings more \
    --fqbn $FQBN \
    --board-options $BOARD_OPTIONS \
    --output-dir $BUILD_OUTPUT \
    $SKETCH_INPUT
}

# Clean build directory
function clean_build {
    rm -r $BUILD_OUTPUT/*
}

# Analyze built firmware
function analyze_firmware {
    echo "not yet implemented!"
}

# Associate functions with commands
declare -A cmds
cmds=(
    ["help"]=display_help
    ["build"]=build_firmware
    ["clean"]=clean_build
    ["analyze"]=analyze_firmware
)

# Parse cmd line args
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -h|--help) display_help ;;
        *) break ;;
    esac
    shift
done

# Handle the positional arg
if [[ -n ${cmds[$1]} ]]; then
    # If it does, call the function
    ${cmds[$1]}
else
    echo "Error: Unknown argument '$1'"
fi
