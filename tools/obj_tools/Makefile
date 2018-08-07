NO_OPENCV := 1
include ../common/flags.mk
CXXFLAGS += -I../..

obj_process:
	$(CXX) obj_process.cc $(CXXFLAGS) -O3 -o obj_process

-include obj_process.d