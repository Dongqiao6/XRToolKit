CXX ?= g++
XRT_SERVICE_ROOT ?= /opt/apps/roboticsservice
HOST_ARCH := $(shell uname -m)
TARGET := xrt_receive_test

ifneq ($(filter aarch64 arm64,$(HOST_ARCH)),)
XRT_ARCH_DIRS := aarch64 arm64 linux_aarch64/64 linux_aarch64
else ifneq ($(filter x86_64 amd64,$(HOST_ARCH)),)
XRT_ARCH_DIRS := x64 linux/64 linux/x64
else
XRT_ARCH_DIRS := x64 aarch64 arm64 linux_aarch64/64 linux_aarch64
endif

XRT_ARCH_LIBS := $(foreach dir,$(XRT_ARCH_DIRS),\
	$(wildcard $(XRT_SERVICE_ROOT)/SDK/$(dir)/libPXREARobotSDK.so))
XRT_SDK_DIR ?= $(patsubst %/libPXREARobotSDK.so,%,$(firstword $(XRT_ARCH_LIBS)))

XRT_HEADER_CANDIDATES := $(wildcard $(XRT_SERVICE_ROOT)/SDK/include/PXREARobotSDK.h) \
	$(wildcard $(XRT_SERVICE_ROOT)/SDK/*/include/PXREARobotSDK.h) \
	$(wildcard $(XRT_SERVICE_ROOT)/SDK/linux_aarch64/64/../include/PXREARobotSDK.h)
XRT_SDK_HEADER ?= $(firstword $(XRT_HEADER_CANDIDATES))
XRT_SDK_INCLUDE_DIR ?= $(patsubst %/PXREARobotSDK.h,%,$(XRT_SDK_HEADER))

CPPFLAGS += -I$(XRT_SDK_INCLUDE_DIR)
CXXFLAGS += -std=c++17 -O2 -g -Wall -Wextra -Wpedantic
LDFLAGS += -L$(XRT_SDK_DIR) -Wl,-rpath,$(XRT_SDK_DIR)
LDLIBS += -lPXREARobotSDK -pthread

.PHONY: all run clean check-sdk info

all: $(TARGET)

check-sdk:
	@echo "Host architecture: $(HOST_ARCH)"
	@echo "SDK header: $(XRT_SDK_INCLUDE_DIR)/PXREARobotSDK.h"
	@echo "SDK library: $(XRT_SDK_DIR)/libPXREARobotSDK.so"
	@test -f "$(XRT_SDK_INCLUDE_DIR)/PXREARobotSDK.h" || \
		(echo "ERROR: PXREARobotSDK.h was not found under $(XRT_SERVICE_ROOT)/SDK" && \
		find "$(XRT_SERVICE_ROOT)/SDK" -name PXREARobotSDK.h -print 2>/dev/null; exit 1)
	@test -f "$(XRT_SDK_DIR)/libPXREARobotSDK.so" || \
		(echo "ERROR: libPXREARobotSDK.so was not found for $(HOST_ARCH)" && \
		find "$(XRT_SERVICE_ROOT)/SDK" -name libPXREARobotSDK.so -print 2>/dev/null; exit 1)

info: check-sdk
	@echo "Compiler: $(CXX) --version"
	@$(CXX) --version | sed -n '1p'

$(TARGET): xrt_receive_test.cpp check-sdk
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET)
