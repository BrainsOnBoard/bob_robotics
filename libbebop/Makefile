CXXFLAGS+=-std=c++14 -g -O2 -march=native -Wall -Wpedantic -I..

# OpenCV
CXXFLAGS+=`pkg-config --cflags opencv`

# The current version of the avcodec code uses some deprecated functions, so
# let's ignore compiler warnings about this. I should probably fix the code up
# properly, though - AD
CXXFLAGS+=-Wno-deprecated-declarations

# AR SDK
ifndef ARSDK_ROOT
	$(error Environment variable ARSDK_ROOT must be defined)
endif
AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
CXXFLAGS+=-I$(AR_STAGING_PATH)/usr/include

.PHONY: all clean

all: bebop.o

bebop.o: bebop.cc
	$(CXX) $(CXXFLAGS) -c -o bebop.o bebop.cc

clean:
	rm -f bebop.o