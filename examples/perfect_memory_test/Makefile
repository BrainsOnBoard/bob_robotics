include ../../common/flags.mk
CXXFLAGS += -I../..

EXECUTABLE := perfect_memory_test

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $(EXECUTABLE) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d