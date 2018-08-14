include ../common/flags.mk

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

SOURCES := bebop.cc video.cc
OBJECTS := $(SOURCES:.cc=.o)
DEPS	:= $(SOURCES:.cc=.d)

.PHONY: all clean

TARGET := libbebop.a

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(AR) -rcs $@ $(OBJECTS)

-include $(DEPS)

%.o: %.cc %.d
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.d: ;

clean:
	rm -f $(OBJECTS) $(DEPS) $(TARGET)
