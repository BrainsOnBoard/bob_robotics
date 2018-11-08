include ../common/flags.mk

# The current version of the avcodec code uses some deprecated functions, so
# let's ignore compiler warnings about this. I should probably fix the code up
# properly, though - AD
CXXFLAGS+=-Wno-deprecated-declarations -Wno-implicit-fallthrough

# AR SDK
ifndef ARSDK_ROOT
ARSDK_ROOT := ../third_party/arsdk3
USE_LOCAL_ARSDK := 1
endif
AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
CXXFLAGS+=-I$(AR_STAGING_PATH)/usr/include

# We don't want these definitions, because we'll get linking errors if they're
# defined in whatever's linking against libbebop
CXXFLAGS += -DNO_BACKGROUND_EXCEPTION_CATCHER_DEFINITIONS

SOURCES := bebop.cc video.cc
OBJECTS := $(SOURCES:.cc=.o)
DEPS	:= $(SOURCES:.cc=.d)

.PHONY: all clean arsdk3

TARGET := libbebop.a

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(AR) -rcs $@ $(OBJECTS)

-include $(DEPS)

%.o: %.cc %.d arsdk3
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.d: ;

clean:
	rm -f $(OBJECTS) $(DEPS) $(TARGET)

arsdk3:
ifdef USE_LOCAL_ARSDK
	$(MAKE) -C $(ARSDK_ROOT)
endif
	@echo Using ARSDK at: $(ARSDK_ROOT)
