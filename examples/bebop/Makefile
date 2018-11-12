LIBBEBOP_PATH := ../../libbebop
WITH_LIBBEBOP := 1
include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean libbebop

EXECUTABLE := bebop_test

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d libbebop
	$(CXX) $(CXXFLAGS) -o $(EXECUTABLE) $(EXECUTABLE).cc $(LINK_FLAGS)

$(EXECUTABLE).d: ;

libbebop:
	$(MAKE) -C $(LIBBEBOP_PATH)

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
