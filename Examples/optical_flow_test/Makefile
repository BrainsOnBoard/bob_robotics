include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

all: optical_flow_test

optical_flow_test:
	$(CXX) optical_flow_test.cc $(CXXFLAGS) $(LINK_FLAGS) -o optical_flow_test

clean:
	rm -f optical_flow_test *.d

-include optical_flow_test.d