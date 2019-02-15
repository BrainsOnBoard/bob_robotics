WITH_MATPLOTLIBCPP := 1
NO_OPENCV := 1
include ../../make_common/bob_robotics.mk

.PHONY: all clean

all: simulated robot robot_tune

-include robot.d

robot: robot.cc robot.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include robot_tune.d

robot_tune: robot_tune.cc robot_tune.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)


-include simulated.d

simulated: simulated.cc simulated.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f simulated robot robot_tune *.d