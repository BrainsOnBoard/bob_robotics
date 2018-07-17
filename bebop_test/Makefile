CXXFLAGS+=-std=c++14 -march=native -I.. -Wall -Wpedantic
LDFLAGS=-pthread -lstdc++ -lm

# OpenCV
CXXFLAGS+=`pkg-config --cflags opencv`
LDFLAGS+=`pkg-config --libs opencv`

# python
CXXFLAGS+=`pkg-config --cflags python3`
LDFLAGS+=`pkg-config --libs python3`

# various ffmpeg libraries, needed for video decoding
LDFLAGS+=-lavcodec -lavformat -lavutil -lswscale

# AR SDK
ifndef ARSDK_ROOT
	$(error Environment variable ARSDK_ROOT must be defined)
endif
AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
AR_LIB_PATH=$(AR_STAGING_PATH)/usr/lib
CXXFLAGS+=-I$(AR_STAGING_PATH)/usr/include \
	-Wl,-rpath,$(AR_LIB_PATH),--disable-new-dtags -Wno-deprecated-declarations
LDFLAGS+=$(AR_LIB_PATH)/libarsal.so $(AR_LIB_PATH)/libardiscovery.so \
	$(AR_LIB_PATH)/libarcontroller.so $(AR_LIB_PATH)/libarnetworkal.so \
	$(AR_LIB_PATH)/libarcommands.so $(AR_LIB_PATH)/libmux.so \
	$(AR_LIB_PATH)/libpomp.so $(AR_LIB_PATH)/libjson-c.so.2 \
	$(AR_LIB_PATH)/libarstream.so $(AR_LIB_PATH)/libarstream2.so \
	$(AR_LIB_PATH)/libarnetwork.so $(AR_LIB_PATH)/librtsp.so \
	$(AR_LIB_PATH)/libsdp.so $(AR_LIB_PATH)/libulog.so \
	$(AR_LIB_PATH)/libarmedia.so $(AR_LIB_PATH)/libfutils.so

.PHONY: all clean

TARGET := bebop_test

all: $(TARGET)

$(TARGET): $(TARGET).cc ../robots/bebop.h
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(TARGET).cc $(LDFLAGS)

clean:
	rm -f $(TARGET)
