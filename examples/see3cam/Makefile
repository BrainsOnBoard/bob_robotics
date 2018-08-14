include ../../common/flags.mk
CXXFLAGS += -I../..

EXECUTABLE := see3cam_test

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS) -o $(EXECUTABLE)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d