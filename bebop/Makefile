# AR SDK
AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
AR_LIB_PATH=$(AR_STAGING_PATH)/usr/lib
CXXFLAGS+=-I$(AR_STAGING_PATH)/usr/include \
	-Wl,-rpath,$(AR_LIB_PATH),--disable-new-dtags \
	-Wno-deprecated-declarations

.PHONY: all clean

all: bebop.o video.o

bebop.o: bebop.cc bebop.h semaphore.h
	$(CXX) $(CXXFLAGS) -c -o bebop.o bebop.cc

video.o: video.cc video.h
	$(CXX) $(CXXFLAGS) -c -o video.o video.cc

clean:
	rm -rf *.o
