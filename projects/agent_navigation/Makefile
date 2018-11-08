WITH_MATPLOTLIBCPP := 1
WITH_LIBANTWORLD := 1
include ../../make_common/bob_robotics.mk

# optionally disable I2C
ifdef NO_I2C_ROBOT
	CXXFLAGS += -DNO_I2C_ROBOT
endif

.PHONY: all clean

all: norbot_navigation robot

-include norbot_navigation.d

norbot_navigation: norbot_navigation.cc norbot_navigation.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include robot.d

robot: robot.cc robot.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include ant_agent_navigation.d

ant_agent_navigation: ant_agent_navigation.cc ant_agent_navigation.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f norbot_navigation robot *.d
