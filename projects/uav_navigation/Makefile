WITH_LIBBEBOP := 1
LIBBEBOP_PATH := ../../libbebop
include ../../common/flags.mk
CXXFLAGS += -I../..

EXECUTABLE := uav_navigation

.PHONY: all clean libbebop

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d libbebop
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

libbebop:
	$(MAKE) -C $(LIBBEBOP_PATH)

%.d: ;

clean:
	rm -f $(EXECUTABLE) *.d