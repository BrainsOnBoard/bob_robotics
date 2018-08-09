include ../../common/flags.mk
CXXFLAGS += -I../..

EXECUTABLE := perfect_memory_test

# This is gross but there doesn't seem to be an easy way to get the include path for numpy
PYTHON_VERSION := $(shell python3 --version | awk '{ print $$2 }' | sed 's/\.[0-9]$$//g')
NUMPY_PATH := /usr/lib/python$(PYTHON_VERSION)/site-packages/numpy/core/include

.PHONY: all clean

all: $(EXECUTABLE) plot_ridf

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $(EXECUTABLE) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

-include plot_ridf.d

plot_ridf: plot_ridf.cc plot_ridf.d

	$(CXX) -o plot_ridf plot_ridf.cc $(CXXFLAGS) `pkg-config --cflags --libs python3` \
			-I$(NUMPY_PATH) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
