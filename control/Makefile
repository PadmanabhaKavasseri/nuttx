# the C compiler to use
CC = gcc

# the C++ compiler to use
CXX = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -g -Wall
CXXFLAGS = $(CFLAGS)

INCLUDE_FILES = \
	-I./inc/libqrc/include/app_msg \
    -I./inc/libqrc/include/module \
    -I./inc/libqrc/protocol/qrc \
	-I./inc/libqrc/protocol/qrc/tinyframe \
	-I./inc/libqrc/protocol/tinyframe \
	-I./inc/ \

CFLAGS += $(INCLUDE_FILES)
CFLAGS += -DQRC_RB5
CXXFLAGS += $(INCLUDE_FILES)

# the name to use for both the target source file, and the output file:
TARGET = main

# source files
C_SRCS := $(shell find ./inc/libqrc/ -name "*.c")


$(info $$C_SRCS is [${C_SRCS}])

CPP_SRCS = src/$(TARGET).cpp
CPP_SRCS += src/qrc_utils.cpp

# object files
C_OBJS = $(C_SRCS:.c=.o)
CPP_OBJS = $(CPP_SRCS:.cpp=.o)

OBJS = $(C_OBJS) $(CPP_OBJS)

LDLIBS = -lpthread

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	$(RM) $(TARGET) $(OBJS)



	
