include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

all: camera_360_test

camera_360_test:
	$(CXX) camera_360_test.cc $(CXXFLAGS) $(LINK_FLAGS) -o camera_360_test

clean:
	rm -f camera_360_test *.d

%.d: ;

-include camera_360_test.d
