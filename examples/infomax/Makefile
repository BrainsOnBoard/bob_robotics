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

%.d: ;

clean:
	rm -f infomax_matlab_comparison infomax_matlab_comparison.d