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

# Improves build time
CXXFLAGS += -pipe

# Linking flags (-lm and -lstdc++ needed for clang)
LINK_FLAGS += -lm -lstdc++ -pthread

# Build with OpenCV
ifndef NO_OPENCV
	CXXFLAGS += `pkg-config --cflags opencv`
	LINK_FLAGS += `pkg-config --libs opencv`
endif

ifdef WITH_MATPLOTLIBCPP
	PYTHON_BIN 			 ?= python
	PYTHON_CONFIG        := $(PYTHON_BIN)-config
	PYTHON_INCLUDE       ?= $(shell $(PYTHON_CONFIG) --includes)

	# extract python version from PYTHON_INCLUDE
	PYTHON_VERSION       := $(shell echo $(PYTHON_INCLUDE) | cut -f 1 -d " " | grep -e python... -o)
	PYTHON_NUMPY_INCLUDE ?= $(shell find $$($(PYTHON_CONFIG) --prefix)/lib -type d -path "*/*-packages/numpy/core/include" | grep -m 1 $(PYTHON_VERSION))

	CXXFLAGS += $(shell $(PYTHON_CONFIG) --includes) -I$(PYTHON_NUMPY_INCLUDE)
	LINK_FLAGS += $(shell $(PYTHON_CONFIG) --libs)
endif
