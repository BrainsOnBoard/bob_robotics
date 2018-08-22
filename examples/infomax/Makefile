include ../../common/flags.mk
CXXFLAGS += -I../.. `pkg-config --cflags eigen3`

EXECUTABLE := infomax_test

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $(EXECUTABLE) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d