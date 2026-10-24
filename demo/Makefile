CXX ?= g++
XRT_SDK_DIR ?= /opt/apps/roboticsservice/SDK/x64
XRT_SDK_INCLUDE_DIR ?= $(XRT_SDK_DIR)/../include
TARGET := xrt_receive_test

CPPFLAGS += -I$(XRT_SDK_INCLUDE_DIR)
CXXFLAGS += -std=c++17 -O2 -g -Wall -Wextra -Wpedantic
LDFLAGS += -L$(XRT_SDK_DIR) -Wl,-rpath,$(XRT_SDK_DIR)
LDLIBS += -lPXREARobotSDK -pthread

.PHONY: all run clean check-sdk

all: $(TARGET)

check-sdk:
	@test -f "$(XRT_SDK_INCLUDE_DIR)/PXREARobotSDK.h" || \
		(echo "Missing header: $(XRT_SDK_INCLUDE_DIR)/PXREARobotSDK.h" && exit 1)
	@test -f "$(XRT_SDK_DIR)/libPXREARobotSDK.so" || \
		(echo "Missing library: $(XRT_SDK_DIR)/libPXREARobotSDK.so" && exit 1)

$(TARGET): xrt_receive_test.cpp check-sdk
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET)
