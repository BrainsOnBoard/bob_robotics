WITH_EIGEN := 1
WITH_SFML_GRAPHICS := 1
include ../../make_common/bob_robotics.mk

EXECUTABLE := collision

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
