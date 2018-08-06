NO_OPENCV := 1
include ../../common/flags.mk
CXXFLAGS += -I../..

ifeq (1,$(shell ../../common/is_ubuntu_1604_or_older.sh; echo $$?))
LINK_FLAGS += -li2c
endif

EXECUTABLE := lm9ds1_test

.PHONY: all clean

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS) -o $(EXECUTABLE)

%d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
