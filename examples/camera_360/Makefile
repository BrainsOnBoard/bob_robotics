include ../../make_common/flags.mk

.PHONY: all clean

EXECUTABLE := camera_360_test

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS) -o $(EXECUTABLE)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d