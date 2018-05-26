CXXFLAGS := -std=c++11 -g -Wall -I.. `pkg-config --cflags opencv`
LDFLAGS := -pthread `pkg-config --libs opencv`

# optionally disable I2C
ifdef NO_I2C_ROBOT
	CXXFLAGS += -DNO_I2C_ROBOT
endif

.PHONY: all clean

all: computer robot

computer: computer.cc
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

robot: robot.cc
	$(CXX) -o $@ $< $(CXXFLAGS) $(LDFLAGS)

clean:
	rm -f computer robot
