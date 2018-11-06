WITH_LIBBEBOP := 1
include ../../make_common/bob_robotics.mk

.PHONY: all clean libbebop

EXECUTABLE := bebop_test

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d libbebop
	$(CXX) $(CXXFLAGS) -o $(EXECUTABLE) $(EXECUTABLE).cc $(LINK_FLAGS)

$(EXECUTABLE).d: ;

libbebop:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libbebop

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
