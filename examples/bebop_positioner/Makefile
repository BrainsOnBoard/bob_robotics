WITH_MATPLOTLIBCPP := 1
WITH_LIBBEBOP := 1
LIBBEBOP_PATH := ../../libbebop
include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean

EXECUTABLE := bebop_positioner

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d libbebop
	$(CXX) -o $(EXECUTABLE) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

libbebop:
	$(MAKE) -C $(LIBBEBOP_PATH)

%.d: ;

clean:
	rm -f $(EXECUTABLE) *.d
