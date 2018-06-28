CXXFLAGS := -std=c++14 -g -Wall -Wpedantic -Wextra -MMD -MP -I.. `pkg-config --cflags opencv`
LDFLAGS := -pthread -lm `pkg-config --libs opencv`

# optionally disable I2C
ifdef NO_I2C_ROBOT
	CXXFLAGS += -DNO_I2C_ROBOT
endif

.PHONY: all clean

all: computer robot

computer: computer.cc computer.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

-include computer.d

robot: robot.cc robot.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

-include robot.d

%.d: ;

clean:
	rm -f computer robot computer.d robot.d
