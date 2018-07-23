# This makefile is just an easy way to standardise the compiler flags we're using
# between different samples/mini-libraries.
#
# Just put "-include ../common/flags.mk" in your makefile.

CPP_STANDARD = c++14
CXXFLAGS += -std=$(CPP_STANDARD) -Wall -Wpedantic -Wextra -MMD -MP
ifdef DEBUG
	CXXFLAGS += -g -O0 -DDEBUG
endif

LINK_FLAGS += -lm -lstdc++ -pthread

ifndef NO_OPENCV
	CXXFLAGS += `pkg-config --cflags opencv`
	LINK_FLAGS += `pkg-config --libs opencv`
endif