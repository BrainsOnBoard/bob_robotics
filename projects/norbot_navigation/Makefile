WITH_MATPLOTLIBCPP := 1
include ../../common/flags.mk
CXXFLAGS += -I../..

# optionally disable I2C
ifdef NO_I2C_ROBOT
	CXXFLAGS += -DNO_I2C_ROBOT
endif

EXECUTABLE := norbot_navigation

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
