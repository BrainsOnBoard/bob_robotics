TARGET := unwrapconfig

ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

CXXFLAGS += -I.. -g -std=c++11 -O2 `pkg-config --libs --cflags opencv`
ifdef ffmpeg_path
	CXXFLAGS += -DFFMPEG_PATH=\"$(ffmpeg_path)\"
endif

.PHONY: clean all install

all: $(TARGET) unwrapfile

$(TARGET): $(TARGET)_src/$(TARGET).cc ../common/opencv_unwrap_360.h
	$(CXX) -o $(TARGET) $(TARGET)_src/$(TARGET).cc $(CXXFLAGS)

unwrapfile: unwrapfile_src/unwrapfile.cc
	$(CXX) -o $@ $< $(CXXFLAGS)

clean:
	rm -f unwrapconfig unwrapfile

install: $(TARGET)
	install -Dm 755 unwrapconfig /usr/local/bin
