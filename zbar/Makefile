all: installed

TARBALL = build/zbar-0.10.tar.bz2
TARBALL_URL =  http://sourceforge.net/projects/zbar/files/zbar/0.10/zbar-0.10.tar.bz2/download
SOURCE_DIR = build/zbar-0.10
UNPACK_CMD = tar xvjf
INSTALL_DIR=`rospack find zbar`

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked Makefile
	cd $(SOURCE_DIR) && ./configure --prefix=$(INSTALL_DIR) --without-python --disable-video
	make -C $(SOURCE_DIR)
	make -C $(SOURCE_DIR) install 
	touch installed

clean:  wipe

wipe:
	rm -rf build installed $(TARBALL) include bin lib share

