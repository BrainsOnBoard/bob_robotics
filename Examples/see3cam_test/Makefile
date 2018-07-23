include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

all: see3cam_test

see3cam_test:
	$(CXX) see3cam_test.cc $(CXXFLAGS) $(LINK_FLAGS) -o see3cam_test

clean:
	rm -f see3cam_test *.d

-include see3cam_test.d