# This makefile is just an easy way to standardise the compiler flags we're using
# between different samples/mini-libraries.
#
# Just put "include (path to BoB robotics)/make_common/bob_robotics.mk" in your makefile.

# Which processor architecture to build for
ARCH ?= native
CXXFLAGS += -march=$(ARCH)

# This variable is for GeNN (GeNN currently defaults to c++11, which won't work with some of our code)
CPP_STANDARD ?= c++14

# Build flags
CXXFLAGS += -std=$(CPP_STANDARD) -Wall -Wpedantic -Wextra -MMD -MP

# Include the root BoB robotics folder
CURRENT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BOB_ROBOTICS_ROOT := $(CURRENT_DIR)/..
CXXFLAGS += -I$(BOB_ROBOTICS_ROOT)

# Debug mode: includes debug symbols, disables optimisation and sets DEBUG macro
ifdef DEBUG
	CXXFLAGS += -g -O0 -DDEBUG
else
	CXXFLAGS += -O2
endif

# For printing network messages (see net/ folder)
ifdef TRACE_NET
	CXXFLAGS += -DTRACE_NET
endif

# Improves build time
CXXFLAGS += -pipe

# Enable only a subset of functionality in units.h to speed up compile time
CXXFLAGS += -DDISABLE_PREDEFINED_UNITS \
		-DENABLE_PREDEFINED_LENGTH_UNITS \
		-DENABLE_PREDEFINED_TIME_UNITS \
		-DENABLE_PREDEFINED_ANGLE_UNITS \
		-DENABLE_PREDEFINED_VELOCITY_UNITS \
		-DENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS

# Linking flags (-lm and -lstdc++ needed for clang)
LINK_FLAGS += -lm -lstdc++ -pthread

# Add support for libantworld
ifdef WITH_LIBANTWORLD
	LINK_FLAGS += -L$(BOB_ROBOTICS_ROOT)/libantworld -lantworld -lglfw -lGL -lGLU -lGLEW
endif

# Build with OpenCV
ifndef NO_OPENCV
ifndef OPENCV_PKG_NAME
	OPENCV_PKG_NAME := opencv
endif
	CXXFLAGS += `pkg-config --cflags $(OPENCV_PKG_NAME)`
	LINK_FLAGS += `pkg-config --libs $(OPENCV_PKG_NAME)`
endif

ifdef WITH_LIBBEBOP
	# libbebop
	LINK_FLAGS += -L$(BOB_ROBOTICS_ROOT)/libbebop -lbebop

	# ARSDK
	# We set the rpath so that compiled programs can find the folder with the ARSDK
	# libs in, but apparently setting the rpath is deprecated; I'm sure there is a
	# nicer way to do this.
	ifndef ARSDK_ROOT
$(error Environment variable ARSDK_ROOT must be defined)
	endif
	AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
	AR_LIB_PATH=$(AR_STAGING_PATH)/usr/lib
	CXXFLAGS+=-I$(AR_STAGING_PATH)/usr/include \
		-Wl,-rpath,$(AR_LIB_PATH),--disable-new-dtags
	LINK_FLAGS+=$(AR_LIB_PATH)/libarsal.so $(AR_LIB_PATH)/libardiscovery.so \
		$(AR_LIB_PATH)/libarcontroller.so $(AR_LIB_PATH)/libarnetworkal.so \
		$(AR_LIB_PATH)/libarcommands.so $(AR_LIB_PATH)/libmux.so \
		$(AR_LIB_PATH)/libpomp.so $(AR_LIB_PATH)/libjson-c.so.2 \
		$(AR_LIB_PATH)/libarstream.so $(AR_LIB_PATH)/libarstream2.so \
		$(AR_LIB_PATH)/libarnetwork.so $(AR_LIB_PATH)/librtsp.so \
		$(AR_LIB_PATH)/libsdp.so $(AR_LIB_PATH)/libulog.so \
		$(AR_LIB_PATH)/libarmedia.so $(AR_LIB_PATH)/libfutils.so

	# various ffmpeg libraries, needed for video decoding
	LINK_FLAGS += -lavcodec -lavformat -lavutil -lswscale

	# Fixes compiler warnings for code deep inside ARSDK
	CXXFLAGS += -Wno-implicit-fallthrough
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

ifdef WITH_I2C
	ifeq (0,$(shell $(CURRENT_DIR)/is_i2c_tools_new.sh; echo $$?))
	LINK_FLAGS += -li2c
	endif
endif
