WITH_MATPLOTLIBCPP := 1
include ../../common/flags.mk
CXXFLAGS += -I../.. `pkg-config --cflags eigen3`
LINK_FLAGS += -fopenmp

.PHONY: all clean

all: infomax_route_example infomax_matlab_comparison

-include infomax_matlab_comparison.d

infomax_matlab_comparison: infomax_matlab_comparison.cc infomax_matlab_comparison.d
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) -o $@ $<

-include infomax_route_example.d

infomax_route_example: infomax_route_example.cc infomax_route_example.d
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) -o $@ $<

%.d: ;

clean:
	rm -f infomax_matlab_comparison infomax_route_example *.d