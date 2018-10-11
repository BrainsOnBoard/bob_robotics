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
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) -o $@ $<

-include robot.d

robot: robot.cc robot.d
	$(CXX) $(CXXFLAGS) $(LINK_FLAGS) -o $@ $<

%.d: ;

clean:
	rm -f computer robot *.d