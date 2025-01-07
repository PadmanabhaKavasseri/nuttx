cmake_minimum_required(VERSION 3.5)
project(qrc)

# Set the CMake message log level to DEBUG 
set(CMAKE_MESSAGE_LOG_LEVEL DEBUG)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(INCLUDE_DIRS
  include/app_msg/
  include/module/
  protocol/qrc/
  protocol/qrc/tinyframe/
  protocol/tinyframe/
)

include_directories(
  ${INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../libqrc-udriver/include
)

set(LIBQRC_SRCS
  protocol/qrc_msg_management.c
  protocol/qrc/qrc.c
  protocol/qrc/qrc_threadpool.c
  protocol/tinyframe/TinyFrame.c
)

message(DEBUG "here1")


link_directories(${CMAKE_SOURCE_DIR}/../libqrc-udriver/build)

add_library(${PROJECT_NAME} SHARED
  ${LIBQRC_SRCS}
)

message(DEBUG "here2")

# Set the directory containing libqrc_udriver.so

target_link_libraries(${PROJECT_NAME} qrc_udriver)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  QRC_RB5
)

set(EXPORT_INCLUDE_DIRS
  include/app_msg/
  include/module/
)

install(
  DIRECTORY ${EXPORT_INCLUDE_DIRS}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

