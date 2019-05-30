include ../../make_common/bob_robotics.mk

SOURCES			:=simulator.cc visualizationCommon.cc
GENERATED_CODE_DIR	:=stone_cx_CODE

OBJECTS			:=$(SOURCES:.cc=.o)
DEPS			:=$(SOURCES:.cc=.d)

ifdef RECORD_ELECTROPHYS
    CXXFLAGS += -DRECORD_ELECTROPHYS
endif


.PHONY: all clean generated_code

all: simulator

simulator: $(OBJECTS) generated_code
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o simulator $(LINK_FLAGS) -L$(GENERATED_CODE_DIR) -lrunner -Wl,-rpath $(GENERATED_CODE_DIR)

-include $(DEPS)

%.o: %.cc %.d
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.d: ;

generated_code:
	$(MAKE) -C $(GENERATED_CODE_DIR)

clean:
	@rm -f simulator *.o *.d
