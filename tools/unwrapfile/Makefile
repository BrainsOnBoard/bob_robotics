include ../../common/flags.mk
CXXFLAGS += -I../..

ifdef ffmpeg_path
	CXXFLAGS += -DFFMPEG_PATH=\"$(ffmpeg_path)\"
endif

EXECUTABLE := unwrapfile

ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif
.PHONY: clean all install

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d
	$(CXX) -o $(EXECUTABLE) $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d

install: $(EXECUTABLE)
	install -Dm 755 $(EXECUTABLE) $(PREFIX)/$(EXECUTABLE)
