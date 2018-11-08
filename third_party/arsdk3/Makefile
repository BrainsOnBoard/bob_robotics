.PHONY: checkout build

checkout:
	@git submodule update --init arsdk_manifests
ifeq ("$(wildcard build.sh)","")
	@repo init -u arsdk_manifests --no-clone-bundle --depth=1
	@repo sync
	$(MAKE) build
endif
ifeq ("$(wildcard out/arsdk-native/staging/usr)","")
	$(MAKE) build
else
	@echo ARSDK is already built
endif

build:
	./build.sh -p arsdk-native -t build-sdk -j
