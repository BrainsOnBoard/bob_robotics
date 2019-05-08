WITH_LIBSERIAL=1
include ../..//make_common/bob_robotics.mk

GENERATED_CODE_DIR	:=Obs_net_CODE

.PHONY: all clean generated_code

all: HD_network

HD_network: simulator.cc generated_code
	$(CXX) $(CXXFLAGS)  simulator.cc -o HD_network -L$(GENERATED_CODE_DIR) $(LINK_FLAGS) -lrunner -Wl,-rpath $(GENERATED_CODE_DIR)

generated_code:
	$(MAKE) -C $(GENERATED_CODE_DIR)
