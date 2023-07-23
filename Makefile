# pico-instal.sh is actual a wrapper script for the following command:
# openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -s tcl -c "program hfp_746.elf verify reset exit"

.PHONY: build clean debug install-debug install

build:
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w .. && make -j4

# Show debug messages
debug:
	mkdir -p debug
	cd debug && DEBUG_MODE=1 cmake -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=pico_w .. && make -j4

clean:
	rm -rf build
	rm -rf debug

install: build
	cd build && pico-install.sh hfp_746

install-debug: debug
	cd debug && pico-install.sh hfp_746

install-bt-debug: bt-debug
	cd bt-debug && pico-install.sh hfp_746
