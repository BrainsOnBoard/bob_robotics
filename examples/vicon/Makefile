include ../../make_common/flags.mk

.PHONY: all clean

all: vicon_test vicon_velocity_test vicon_plot_test

-include vicon_test.d

vicon_test: vicon_test.cc vicon_test.d
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) vicon_test.cc -o vicon_test

-include vicon_velocity_test.d

vicon_velocity_test: vicon_velocity_test.cc vicon_velocity_test.d
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) vicon_velocity_test.cc -o vicon_velocity_test

vicon_plot_test: vicon_plot_test.cc
	$(MAKE) -f MakefilePlotTest

%.d: ;

clean:
	rm -f vicon_test vicon_plot_test vicon_velocity_test *.d
