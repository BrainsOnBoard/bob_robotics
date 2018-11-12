include ../make_common/bob_robotics.mk

# The current version of the avcodec code uses some deprecated functions, so
# let's ignore compiler warnings about this. I should probably fix the code up
# properly, though - AD
CXXFLAGS+=-Wno-deprecated-declarations -Wno-implicit-fallthrough

# AR SDK
ifndef ARSDK_ROOT
$(error Environment variable ARSDK_ROOT must be defined)
endif
AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
CXXFLAGS+=-I$(AR_STAGING_PATH)/usr/include

# We don't want these definitions, because we'll get linking errors if they're
# defined in whatever's linking against libbebop
CXXFLAGS += -DNO_BACKGROUND_EXCEPTION_CATCHER_DEFINITIONS

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
