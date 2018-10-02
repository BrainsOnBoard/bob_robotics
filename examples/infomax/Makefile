WITH_MATPLOTLIBCPP := 1
include ../../common/flags.mk
CXXFLAGS += -I../.. `pkg-config --cflags eigen3`
LINK_FLAGS += -fopenmp

EXECUTABLE := infomax_matlab_comparison

.PHONY: all clean

-include infomax_example.d

infomax_example: infomax_example.cc infomax_example.d
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) -o $@ $<

-include infomax_matlab_comparison.d

infomax_matlab_comparison: infomax_matlab_comparison.cc infomax_matlab_comparison.d
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) -o $@ $<

-include infomax_antworld_example.d

infomax_antworld_example: infomax_antworld_example.cc infomax_antworld_example.d
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) -o $@ $<

%.d: ;

all: infomax_example infomax_antworld_example infomax_matlab_comparison

clean:
	rm -f infomax_matlab_comparison infomax_example *.d
