WITH_LIBANTWORLD := 1
WITH_MATPLOTLIBCPP := 1
include ../../make_common/flags.mk

.PHONY: all clean libantworld

all: perfect_memory_test perfect_memory_antworld_test plot_ridf

-include perfect_memory_test.d

perfect_memory_test: perfect_memory_test.cc perfect_memory_test.d libantworld
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include perfect_memory_antworld_test.d

perfect_memory_antworld_test: perfect_memory_antworld_test.cc perfect_memory_antworld_test.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

-include plot_ridf.d

plot_ridf: plot_ridf.cc plot_ridf.d
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C $(BOB_ROBOTICS_ROOT)/libantworld

%.d: ;

clean:
	rm -f perfect_memory_test perfect_memory_antworld_test plot_ridf *.d
