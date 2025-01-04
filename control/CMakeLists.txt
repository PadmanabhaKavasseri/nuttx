cmake_minimum_required(VERSION 3.10)

# Project name
project(motor_control)

# Set C and C++ compilers
# Check if the C and C++ compilers are already set
if (NOT CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER /usr/bin/gcc)
endif()
if (NOT CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER /usr/bin/g++)
endif()

# Compiler flags
set(CMAKE_C_FLAGS "-g -Wall -DQRC_RB5")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}")

# Find Python 3 with Interpreter
find_package(Python3 COMPONENTS Development Interpreter REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})

# Set the path for pybind11
set(pybind11_DIR "/usr/local/lib/python3.10/dist-packages/pybind11/share/cmake/pybind11")

# Find pybind11
find_package(pybind11 REQUIRED)

# Add subdirectory for libqrc
add_subdirectory(inc/libqrc)

# Include directories for other includes
include_directories(
    ${pybind11_INCLUDE_DIRS}
    ./inc/
)

# Source files for main project
set(CPP_SOURCES src/motor_control.cpp src/qrc_utils.cpp)
set(SOURCES ${CPP_SOURCES})

# Output names
set(EXECUTABLE_TARGET_NAME "motor_control_exe")
set(LIBRARY_TARGET_NAME "motor_control_lib")
execute_process(COMMAND python3-config --extension-suffix OUTPUT_VARIABLE TARGET_EXT OUTPUT_STRIP_TRAILING_WHITESPACE)
set(LIBRARY_NAME "${LIBRARY_TARGET_NAME}${TARGET_EXT}")

# Library dependencies
set(LDLIBS -lpthread ${Python3_LIBRARIES})

# Add executable
add_executable(${EXECUTABLE_TARGET_NAME} ${CPP_SOURCES})

# Target for shared library
add_library(${LIBRARY_TARGET_NAME} SHARED ${SOURCES})

# Set properties to avoid double extension
set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES PREFIX "" SUFFIX "${TARGET_EXT}")

# Link libraries
target_link_libraries(${EXECUTABLE_TARGET_NAME} ${LDLIBS} libqrc)
target_link_libraries(${LIBRARY_TARGET_NAME} ${LDLIBS} libqrc)
target_compile_options(${LIBRARY_TARGET_NAME} PRIVATE -O3 -shared -std=c++11 -fPIC)

# Link pybind11
target_link_libraries(${LIBRARY_TARGET_NAME} pybind11::module)

