TARGET := unwrapconfig

ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

CXXFLAGS += -I.. -g -std=c++14 -O2 -Wall -Wpedantic -Wextra -MMD -MP `pkg-config --libs --cflags opencv`
ifdef ffmpeg_path
	CXXFLAGS += -DFFMPEG_PATH=\"$(ffmpeg_path)\"
endif

.PHONY: clean all install

all: unwrapconfig unwrapfile

unwrapconfig: unwrapconfig_src/unwrapconfig.cc unwrapconfig.d
	$(CXX) -o unwrapconfig $(TARGET)_src/$(TARGET).cc $(CXXFLAGS)

-include unwrapconfig.d

unwrapfile: unwrapfile_src/unwrapfile.cc unwrapfile.d
	$(CXX) -o $@ $< $(CXXFLAGS)

-include unwrapfile.d

%.d: ;

clean:
	rm -f unwrapconfig unwrapfile unwrapconfig.d unwrapfile.d

install: $(TARGET)
	install -Dm 755 unwrapconfig /usr/local/bin
