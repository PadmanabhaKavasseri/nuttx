cmake_minimum_required(VERSION 3.5)
project(qrc_udriver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-g -Wall -Wextra -Wpedantic -O0 -fPIC)

include_directories(
  include
)

add_library(qrc_udriver SHARED src/qti_qrc_udriver.c src/qti_qrc_uart.c)
SET_TARGET_PROPERTIES(qrc_udriver PROPERTIES VERSION 1.0)
target_link_libraries(qrc_udriver gpiod)
set_target_properties(qrc_udriver PROPERTIES PUBLIC_HEADER include/qti_qrc_udriver.h)


add_executable(qrc_udriver_test test/qti_qrc_test.c)
target_link_libraries(qrc_udriver_test qrc_udriver)

install(TARGETS
    qrc_udriver
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)

install(TARGETS
    qrc_udriver_test
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
