WITH_MATPLOTLIBCPP := 1
WITH_LIBANTWORLD := 1
include ../../make_common/bob_robotics.mk
CXXFLAGS += `pkg-config --cflags sfml-graphics`
LINK_FLAGS += `pkg-config --libs sfml-graphics`

# optionally disable I2C
ifdef NO_I2C_ROBOT
	CXXFLAGS += -DNO_I2C_ROBOT
endif

.PHONY: all clean libantworld

all: norbot_navigation ant_agent_navigation

-include norbot_navigation.d

norbot_navigation: norbot_navigation.cc norbot_navigation.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include ant_agent_navigation.d

ant_agent_navigation: ant_agent_navigation.cc ant_agent_navigation.d libantworld
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld

%.d: ;

clean:
	rm -f ant_agent_navigation norbot_navigation *.d
