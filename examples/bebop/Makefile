WITH_LIBBEBOP := 1
include ../../make_common/bob_robotics.mk

.PHONY: all clean libbebop

EXECUTABLE := bebop_test

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d libbebop
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include bebop_relative_move_test.d

bebop_relative_move_test: bebop_relative_move_test.cc bebop_relative_move_test.d libbebop
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

libbebop:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libbebop

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
