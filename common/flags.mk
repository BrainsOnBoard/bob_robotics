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
	PYTHON_BIN ?= python

	# This is gross but there doesn't seem to be an easy way to get the include path for numpy
	PYTHON_VERSION := $(shell $(PYTHON_BIN) --version |& awk '{ print $$2 }' | sed 's/\.[0-9]*$$//g')
	NUMPY_PATH := /usr/lib/python$(PYTHON_VERSION)/site-packages/numpy/core/include

	PYTHON_MAJOR_VERSION := $(shell echo $(PYTHON_VERSION) | head -c 1)
	CXXFLAGS += `pkg-config --cflags --libs python$(PYTHON_MAJOR_VERSION)` -I$(NUMPY_PATH)
endif
