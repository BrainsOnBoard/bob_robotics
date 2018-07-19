CXXFLAGS+=-std=c++14 -march=native -I.. -Wall -Wpedantic
LDFLAGS=-g -pthread -lstdc++ -lm ../libbebop/bebop.o

# OpenCV
CXXFLAGS+=`pkg-config --cflags opencv`
LDFLAGS+=`pkg-config --libs opencv`

# various ffmpeg libraries, needed for video decoding
LDFLAGS+=-lavcodec -lavformat -lavutil -lswscale

# AR SDK
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
LDFLAGS+=$(AR_LIB_PATH)/libarsal.so $(AR_LIB_PATH)/libardiscovery.so \
	$(AR_LIB_PATH)/libarcontroller.so $(AR_LIB_PATH)/libarnetworkal.so \
	$(AR_LIB_PATH)/libarcommands.so $(AR_LIB_PATH)/libmux.so \
	$(AR_LIB_PATH)/libpomp.so $(AR_LIB_PATH)/libjson-c.so.2 \
	$(AR_LIB_PATH)/libarstream.so $(AR_LIB_PATH)/libarstream2.so \
	$(AR_LIB_PATH)/libarnetwork.so $(AR_LIB_PATH)/librtsp.so \
	$(AR_LIB_PATH)/libsdp.so $(AR_LIB_PATH)/libulog.so \
	$(AR_LIB_PATH)/libarmedia.so $(AR_LIB_PATH)/libfutils.so

.PHONY: all clean libbebop

TARGET := bebop_test

all: $(TARGET)

$(TARGET): $(TARGET).cc libbebop
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(TARGET).cc $(LDFLAGS)

libbebop:
	$(MAKE) -C ../libbebop

clean:
	rm -f $(TARGET)
