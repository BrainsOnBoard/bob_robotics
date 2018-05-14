.PHONY: all clean

CXXFLAGS=-g -I../src `pkg-config --cflags --libs opencv`

all: yaml_read_test yaml_write_test

yaml_read_test: yaml_read_test.cc ../common/opencv_unwrap_360.h
	$(CXX) -o $@ $< $(CXXFLAGS)

yaml_write_test: yaml_write_test.cc ../common/opencv_unwrap_360.h
	$(CXX) -o $@ $< $(CXXFLAGS)

clean:
	rm -f yaml_read_test yaml_write_test{,.yaml}
