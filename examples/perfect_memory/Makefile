WITH_MATPLOTLIBCPP := 1
include ../../make_common/flags.mk
CXXFLAGS += -I../..

ifndef DEBUG
CXXFLAGS += -O3
endif

EXECUTABLE := perfect_memory_test

.PHONY: all clean libantworld

all: $(EXECUTABLE) perfect_memory_antworld_test plot_ridf

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $(EXECUTABLE) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

-include perfect_memory_antworld_test.d

perfect_memory_antworld_test: perfect_memory_antworld_test.cc perfect_memory_antworld_test.d libantworld
	$(CXX) -o perfect_memory_antworld_test perfect_memory_antworld_test.cc $(CXXFLAGS) $(LINK_FLAGS) -L../../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW

-include plot_ridf.d

plot_ridf: plot_ridf.cc plot_ridf.d
	$(CXX) -o plot_ridf plot_ridf.cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) plot_ridf perfect_memory_antworld_test *.d

libantworld:
	$(MAKE) -C ../../libantworld