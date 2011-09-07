all: installed

VER = 1.1.2
TARBALL       = build/ann_$(VER).tar.gz
TARBALL_URL   = http://pr.willowgarage.com/downloads/ann_$(VER).tar.gz
UNPACK_CMD    = tar xfz
SOURCE_DIR    = build/ann_$(VER)
TARBALL_PATCH = gcc43_shared.patch fPIC.patch distance.patch
MD5SUM_FILE   = ann_$(VER).tar.gz.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked Makefile
	echo "Building ANN..."
	if test `uname` = Darwin; then \
	  cd build/ann_$(VER) && make macosx-g++; \
	else\
	  cd build/ann_$(VER) && make linux-shared-g++; \
	fi
	cp -r $(SOURCE_DIR)/lib lib
	cp -r $(SOURCE_DIR)/include include
	touch installed
clean: wipe
wipe:
	rm -rf build installed
