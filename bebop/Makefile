# AR SDK
AR_STAGING_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging
AR_LIB_PATH=$(AR_STAGING_PATH)/usr/lib
CXXFLAGS+=-O2 -I$(AR_STAGING_PATH)/usr/include \
	-Wl,-rpath,$(AR_LIB_PATH),--disable-new-dtags \
	-Wno-deprecated-declarations

.PHONY: all clean

all: bebop.o xbox.o video.o

bebop.o: bebop.cc bebop.h semaphore.h
	$(CXX) $(CXXFLAGS) -c -o bebop.o bebop.cc

xbox.o: bebop_xbox.cc bebop_xbox.h xbox.h
	$(CXX) $(CXXFLAGS) -c -o xbox.o bebop_xbox.cc

video.o: video.cc video.h
	$(CXX) $(CXXFLAGS) -c -o video.o video.cc

xbox_test:
	$(CXX) -o xbox_test -pthread xbox_test.cc

clean:
	rm -f xbox_test *.o
