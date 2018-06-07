# AR SDK
AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
AR_LIB_PATH=$(AR_STAGING_PATH)/usr/lib
CXXFLAGS+=-O2 -g -I$(AR_STAGING_PATH)/usr/include \
	-Wl,-rpath,$(AR_LIB_PATH),--disable-new-dtags \
	-Wno-deprecated-declarations -std=c++11

.PHONY: all clean

all: bebop.o

%.o: src/%.cc
	$(_PREFIX)$(CXX) -o $@ -c $< $(CXXFLAGS)

clean:
	rm -f *.o
