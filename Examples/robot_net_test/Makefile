include ../../common/flags.mk
CXXFLAGS += -I../..

# optionally disable I2C
ifdef NO_I2C_ROBOT
	CXXFLAGS += -DNO_I2C_ROBOT
endif

.PHONY: all clean

all: computer robot

computer:
	$(CXX) -o $@ computer.cc $(CXXFLAGS) $(LINK_FLAGS)

-include computer.d

robot:
	$(CXX) -o $@ robot.cc $(CXXFLAGS) $(LINK_FLAGS)

-include robot.d

%.d: ;

clean:
	rm -f computer robot computer.d robot.d
