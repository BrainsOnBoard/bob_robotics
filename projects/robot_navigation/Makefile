include ../../common/flags.mk
CXXFLAGS += -I../..

# optionally disable I2C
ifdef NO_I2C_ROBOT
	CXXFLAGS += -DNO_I2C_ROBOT
endif

.PHONY: all clean

all: computer robot

-include computer.d

computer: computer.cc computer.d
	$(CXX) -o $@ computer.cc $(CXXFLAGS) $(LINK_FLAGS)

-include robot.d

robot: robot.cc robot.d
	$(CXX) -o $@ robot.cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f computer robot computer.d robot.d
