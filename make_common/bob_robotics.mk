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

# For building with Google Test
ifdef WITH_GTEST
	DEBUG := 1
	CXXFLAGS += `pkg-config --cflags gtest`
	LINK_FLAGS += `pkg-config --libs gtest`
endif

# Debug mode: includes debug symbols, disables optimisation and sets DEBUG macro
ifdef DEBUG
	CXXFLAGS += -g -O0 -DDEBUG
else
	CXXFLAGS += -O2
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

# Always build using our logging library
EXTRA_DEPS += plog
CXXFLAGS += -I$(BOB_ROBOTICS_ROOT)/third_party/plog/include

# Add support for libantworld
ifdef WITH_LIBANTWORLD
	LINK_FLAGS += -L$(BOB_ROBOTICS_ROOT)/libantworld -lantworld -lglfw -lGL -lGLU -lGLEW
	EXTRA_DEPS += libantworld
endif

ifdef WITH_IMGUI
	CXXFLAGS += -I$(BOB_ROBOTICS_ROOT)/third_party/imgui/imgui
	LINK_FLAGS += -L$(BOB_ROBOTICS_ROOT)/third_party/imgui -limgui -lglfw -lGL -lGLU -lGLEW
	EXTRA_DEPS += imgui
endif

ifdef WITH_SDL2
	CXXFLAGS += `pkg-config --cflags sdl2`
	LINK_FLAGS += `pkg-config --libs sdl2`
endif

ifdef WITH_GTK
	CXXFLAGS += `pkg-config --cflags gtk+-3.0`
	LINK_FLAGS += `pkg-config --libs gtk+-3.0`
endif

ifdef WITH_OSM_GPS_MAP
	CXXFLAGS += `pkg-config --cflags osmgpsmap-1.0`
	LINK_FLAGS += `pkg-config --libs osmgpsmap-1.0`
	CXXFLAGS += `pkg-config --cflags libffi`
	LINK_FLAGS += `pkg-config --libs libffi`
	
endif


ifdef WITH_LIBBEBOP
	# libbebop
	LINK_FLAGS += -L$(BOB_ROBOTICS_ROOT)/libbebop -lbebop \
					-Wl,-rpath,$(AR_LIB_PATH),--disable-new-dtags

	# ARSDK
	# We set the rpath so that compiled programs can find the folder with the ARSDK
	# libs in, but apparently setting the rpath is deprecated; I'm sure there is a
	# nicer way to do this.
	ifndef ARSDK_ROOT
$(error Environment variable ARSDK_ROOT must be defined)
	endif
	AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
	AR_LIB_PATH=$(AR_STAGING_PATH)/usr/lib
	CXXFLAGS+=-I$(AR_STAGING_PATH)/usr/include
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

	# For default.mk
	EXTRA_DEPS += libbebop
endif

# Build with OpenCV
ifndef NO_OPENCV
ifndef OPENCV_PKG_NAME
	OPENCV_PKG_NAME := opencv
endif
	CXXFLAGS += `pkg-config --cflags $(OPENCV_PKG_NAME)`
	LINK_FLAGS += `pkg-config --libs $(OPENCV_PKG_NAME)`
endif

ifdef WITH_MATPLOTLIBCPP
	PYTHON_BIN 			 ?= python
	PYTHON_CONFIG        := $(PYTHON_BIN)-config
	PYTHON_INCLUDE       ?= $(shell $(PYTHON_CONFIG) --includes)

	# extract python version from PYTHON_INCLUDE
	PYTHON_VERSION       := $(shell echo $(PYTHON_INCLUDE) | cut -f 1 -d " " | grep -e python... -o)
	PYTHON_NUMPY_INCLUDE ?= $(shell $(PYTHON_BIN) $(CURRENT_DIR)/find_numpy.py)

	CXXFLAGS += $(shell $(PYTHON_CONFIG) --includes) -I$(PYTHON_NUMPY_INCLUDE)
	LINK_FLAGS += $(shell $(PYTHON_CONFIG) --libs)
endif

ifdef WITH_I2C_ROBOT
ifdef NO_I2C_ROBOT
	CXXFLAGS += -DNO_I2C_ROBOT
else
	WITH_I2C := 1
endif
endif

ifdef WITH_I2C
	ifeq (0,$(shell $(CURRENT_DIR)/is_i2c_tools_new.sh; echo $$?))
	LINK_FLAGS += -li2c
	endif
endif

ifdef WITH_EIGEN
	CXXFLAGS += `pkg-config --cflags eigen3` -fopenmp
	LINK_FLAGS += -fopenmp
endif

ifdef WITH_SFML_GRAPHICS
        CXXFLAGS += `pkg-config --cflags sfml-graphics`
        LINK_FLAGS += `pkg-config --libs sfml-graphics`
endif

ifdef WITH_LIBSERIAL
        LINK_FLAGS += `pkg-config libserial --libs`
        CXXFLAGS += `pkg-config libserial --cflags`
endif
