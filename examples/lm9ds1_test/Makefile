include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

all: lm9ds1_test

lm_9ds1_test:
	$(CXX) lm9ds1_test.cc $(CXXFLAGS) $(LINK_FLAGS) -o lm9ds1_test

clean:
	rm -f lm9ds1_test *.d

-include lm9ds1_test.d