TARGET = touchbends
OPT = -Os -flto
LDFLAGS += -flto
CPP_STANDARD = -std=gnu++17
C_DEFS += -DDAISY_NO_USB_HOST

CPP_SOURCES = \
    src/main.cpp \
    src/surface/knobs.cpp \
    src/surface/pads.cpp \
    src/surface/switches.cpp

C_INCLUDES = \
    -Isrc \
    -Isrc/app \
    -Isrc/surface \
    -Isrc/circuit \
    -Isrc/delay \
    -Isrc/parts \
    -Isrc/bends \
    -Isrc/dsp

LIBDAISY_DIR ?= lib/libDaisy
DAISYSP_DIR ?= lib/DaisySP

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

setup-libs:
	git submodule update --init --recursive
	cd $(LIBDAISY_DIR) && git checkout facb66c7 && patch -p1 -N -r - < ../../patches/libdaisy_flash_opt.patch || true
	cd $(DAISYSP_DIR) && git checkout 599511b740f8f3a9b8db72a0642aa45b8a23c3a3
	$(MAKE) libs

libs:
	$(MAKE) -C $(LIBDAISY_DIR) -j8
	$(MAKE) -C $(DAISYSP_DIR) -j8

clean-libs:
	$(MAKE) -C $(LIBDAISY_DIR) clean
	$(MAKE) -C $(DAISYSP_DIR) clean

test:
	mkdir -p build
	clang++ -std=c++17 -Isrc -I$(DAISYSP_DIR)/Source tests/test_delay_host.cpp -o build/test_delay_host
	./build/test_delay_host

.PHONY: libs clean-libs setup-libs test
