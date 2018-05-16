ifeq ($(PREFIX),)
    PREFIX := /usr
endif

ifdef ffmpeg_path
	CXXFLAGS += -DFFMPEG_PATH=\"$(ffmpeg_path)\"
endif

TARGET := punwrap
$(TARGET): src/$(TARGET).cpp
	$(CXX) $(CXXFLAGS) -o $(TARGET) src/$(TARGET).cpp `pkg-config --libs --cflags opencv`

install: $(TARGET)
	install -m 755 $(TARGET) $(DESTDIR)$(PREFIX)/bin/

clean:
	rm -f $(TARGET) *.o

