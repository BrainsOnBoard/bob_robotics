.PHONY: all clean

all:
ifeq ("$(wildcard ev3dev-lang-cpp/CMakeLists.txt)","")
	git submodule update --init ev3dev-lang-cpp
endif
ifeq ("$(wildcard build/libev3dev.a)","")
	mkdir -p build
	cd build && cmake ../ev3dev-lang-cpp -DEV3DEV_PLATFORM=EV3 \
		-DCMAKE_C_COMPILER=/usr/bin/arm-linux-gnueabi-gcc \
		-DCMAKE_CXX_COMPILER=/usr/bin/arm-linux-gnueabi-g++ && $(MAKE)
endif

clean:
	rm -rf build