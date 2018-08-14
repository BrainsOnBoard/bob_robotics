include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

all: display_test unwrap_display_test

-include display_test.d

display_test: display_test.cc display_test.d
	$(CXX) -o $@ display_test.cc $(CXXFLAGS) $(LINK_FLAGS)

-include unwrap_display_test.d

unwrap_display_test: unwrap_display_test.cc unwrap_display_test.d
	$(CXX) -o $@ unwrap_display_test.cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f display_test unwrap_display_test *.d
