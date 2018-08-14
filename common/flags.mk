# This makefile is just an easy way to standardise the compiler flags we're using
# between different samples/mini-libraries.
#
# Just put "include ../common/flags.mk" in your makefile.

# Which processor architecture to build for
ARCH ?= native
CXXFLAGS += -march=$(ARCH)

# This variable is for GeNN (GeNN currently defaults to c++11, which won't work with some of our code)
CPP_STANDARD ?= c++14

# Build flags
CXXFLAGS += -std=$(CPP_STANDARD) -Wall -Wpedantic -Wextra -MMD -MP
ifdef DEBUG
	CXXFLAGS += -g -O0 -DDEBUG
else
	CXXFLAGS += -O2
endif

# Linking flags (-lm and -lstdc++ needed for clang)
LINK_FLAGS += -lm -lstdc++ -pthread

# Build with OpenCV
ifndef NO_OPENCV
	CXXFLAGS += `pkg-config --cflags opencv`
	LINK_FLAGS += `pkg-config --libs opencv`
endif

ifdef WITH_MATPLOTLIBCPP
	PYTHON_VERSION       := $(shell python --version 2>&1 | awk '{ print $$2 }' | sed -r 's/\.[0-9]+$$//g')
	PYTHON_NUMPY_INCLUDE ?= $(shell find $$(python-config --prefix)/lib -type d -path "*/site-packages/numpy/core/include" | grep -m 1 $(PYTHON_VERSION))

	CXXFLAGS += $(shell python-config --includes) -I$(PYTHON_NUMPY_INCLUDE)
	LINK_FLAGS += $(shell python-config --libs)
endif