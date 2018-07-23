include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

all: display_test unwrap_display_test

display_test:
	$(CXX) -o $@ display_test.cc $(CXXFLAGS) $(LINK_FLAGS)

-include display_test.d

unwrap_display_test:
	$(CXX) -o $@ unwrap_display_test.cc $(CXXFLAGS) $(LINK_FLAGS)

-include unwrap_display_test.d

%.d: ;

clean:
	rm -f display_test unwrap_display_test *.d
