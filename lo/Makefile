# include $(shell rospack find mk)/cmake.mk

BUILD_DIR=build
CMAKE=cmake
CMAKE_ARGS= -DCMAKE_INSTALL_PREFIX=`rospack find lo`/local

all:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(BUILD_DIR) && make && make install

clean:
	rm -rf $(BUILD_DIR)
	rm -rf `rospack find lo`/local
   