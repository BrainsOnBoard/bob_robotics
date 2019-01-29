WITH_MATPLOTLIBCPP := 1
WITH_EIGEN := 1
include ../../make_common/bob_robotics.mk


.PHONY: all clean

all: infomax_route_example infomax_matlab_comparison

-include infomax_matlab_comparison.d

infomax_matlab_comparison: infomax_matlab_comparison.cc infomax_matlab_comparison.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include infomax_route_example.d

infomax_route_example: infomax_route_example.cc infomax_route_example.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f infomax_matlab_comparison infomax_route_example *.d