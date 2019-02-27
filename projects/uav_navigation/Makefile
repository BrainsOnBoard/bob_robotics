WITH_LIBBEBOP := 1
WITH_MATPLOTLIBCPP := 1
include ../../make_common/bob_robotics.mk

EXECUTABLE := uav_navigation

.PHONY: all clean libbebop

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d libbebop
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

libbebop:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libbebop

%.d: ;

clean:
	rm -f $(EXECUTABLE) *.d
