WITH_MATPLOTLIBCPP := 1
include ../../common/flags.mk
CXXFLAGS += -I../..

EXECUTABLE := perfect_memory_test

.PHONY: all clean

all: $(EXECUTABLE) plot_ridf

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $(EXECUTABLE) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

-include plot_ridf.d

plot_ridf: plot_ridf.cc plot_ridf.d
	$(CXX) -o plot_ridf plot_ridf.cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) plot_ridf *.d
