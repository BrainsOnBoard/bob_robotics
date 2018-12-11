.PHONY: checkout build

checkout:
	@# Check out git submodule
	@git submodule update --init arsdk_manifests

	@# If build.sh doesn't exist, then checkout repo
ifeq ("$(wildcard build.sh)","")
	@repo init -u arsdk_manifests --no-clone-bundle --depth=1
	@repo sync
	$(MAKE) build
endif

	@# If target doesn't exist, run build script. Note that this doesn't check
	@# whether the ARSDK finished building, so if it's half built you should run:
	@# 	make build
ifeq ("$(wildcard out/arsdk-native/staging/usr)","")
	$(MAKE) build
else
	@echo ARSDK is already built
endif

# Build ARSDK with Parrot's script
build:
	./build.sh -p arsdk-native -t build-sdk -j
