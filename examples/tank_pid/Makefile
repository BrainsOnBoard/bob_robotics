WITH_I2C_ROBOT := 1
WITH_MATPLOTLIBCPP := 1
include ../../make_common/bob_robotics.mk

# Some versions of gcc are really stubborn about letting you ignore a return
# value from certain functions, even when you want to.
#
# See: https://stackoverflow.com/questions/7271939/warning-ignoring-return-value-of-scanf-declared-with-attribute-warn-unused-r
CXXFLAGS += -Wno-unused-result

EXECUTABLE := tank_pid

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
