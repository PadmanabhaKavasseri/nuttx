cmake_minimum_required(VERSION 3.10)

# Project name
project(libqrc C)

# Set C compiler
if (NOT CMAKE_C_COMPILER) 
	set(CMAKE_C_COMPILER /usr/bin/gcc)
endif()
# Compiler flags
set(CMAKE_C_FLAGS "-g -Wall -DQRC_RB5")

# Source files
file(GLOB_RECURSE SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/qrc/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/qrc/tinyframe/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/*.c
)

# Add library 
add_library(libqrc STATIC ${SOURCES})

# Include directories
target_include_directories(libqrc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/app_msg
    ${CMAKE_CURRENT_SOURCE_DIR}/include/module
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/qrc
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/qrc/tinyframe
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/tinyframe
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Add library
#add_library(libqrc STATIC ${SOURCES})

# Install the library and headers
install(TARGETS libqrc DESTINATION lib)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include)

