include ../..//make_common/bob_robotics.mk

LINK_FLAGS += $(shell pkg-config libserial --libs)
CXXFLAGS += -I/usr/include $(shell pkg-config libserial --cflags)

all: enose

enose: enose.cc
	$(CXX) enose.cc -o enose $(CXXFLAGS) $(LINK_FLAGS) 
