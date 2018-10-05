LIBBEBOP_PATH := ../../libbebop
WITH_LIBBEBOP := 1
WITH_MATPLOTLIBCPP := 1
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