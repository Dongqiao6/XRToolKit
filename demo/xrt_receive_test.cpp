#include <PXREARobotSDK.h>

#include <csignal>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>

namespace {
volatile std::sig_atomic_t g_running = 1;

struct TestContext {
    std::mutex mutex;
    std::ofstream log;
    std::uint64_t state_count = 0;
};

void handle_signal(int) {
    g_running = 0;
}

const char* callback_name(PXREAClientCallbackType type) {
    switch (type) {
        case PXREAServerConnect:    return "PXREAServerConnect";
        case PXREAServerDisconnect: return "PXREAServerDisconnect";
        case PXREADeviceFind:       return "PXREADeviceFind";
        case PXREADeviceMissing:    return "PXREADeviceMissing";
        case PXREADeviceConnect:    return "PXREADeviceConnect";
        case PXREADeviceStateJson:  return "PXREADeviceStateJson";
        default:                    return "UnknownCallback";
    }
}

void on_pxrea_callback(void* context,
                       PXREAClientCallbackType type,
                       int status,
                       void* userData) {
    auto* ctx = static_cast<TestContext*>(context);
    const char* payload = userData ? static_cast<const char*>(userData) : nullptr;

    if (type == PXREADeviceStateJson) {
        if (ctx == nullptr) {
            std::cerr << "[DATA] callback received with null context; status="
                      << status << std::endl;
            return;
        }
        if (payload == nullptr) {
            std::lock_guard<std::mutex> lock(ctx->mutex);
            std::cout << "[DATA] callback received with null payload; status="
                      << status << std::endl;
            return;
        }

        std::lock_guard<std::mutex> lock(ctx->mutex);
        ++ctx->state_count;

        // Save every received state message as one line, for later inspection.
        if (ctx->log.is_open()) {
            ctx->log << payload << '\n';
            ctx->log.flush();
        }

        // Avoid flooding the terminal at high update rates: print the first 5
        // messages, then every 30th. All messages are still saved to the log.
        if (ctx->state_count <= 5 || ctx->state_count % 30 == 0) {
            std::cout << "\n[DATA #" << ctx->state_count << "] status=" << status
                      << "\n" << payload << "\n" << std::flush;
        }
        return;
    }

    std::lock_guard<std::mutex> lock(ctx->mutex);
    std::cout << "[EVENT] " << callback_name(type)
              << " (type=" << static_cast<int>(type)
              << ", status=" << status << ")";
    if (payload != nullptr && payload[0] != '\0') {
        std::cout << " userData=" << payload;
    }
    std::cout << std::endl;
}
} // namespace

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    TestContext context;
    context.log.open("xrt_received.jsonl", std::ios::out | std::ios::app);
    if (context.log.is_open()) {
        std::cout << "Raw data log: xrt_received.jsonl (append mode)" << std::endl;
    } else {
        std::cerr << "WARNING: cannot open xrt_received.jsonl; data will only print to terminal."
                  << std::endl;
    }

    std::cout << "Initializing XRoboToolkit SDK client..." << std::endl;
    const int init_result = PXREAInit(&context, on_pxrea_callback, PXREAFullMask);
    std::cout << "PXREAInit returned: " << init_result << std::endl;
    std::cout << "Keep Quest XRoboToolkit open, enable Tracking options and Send.\n"
              << "Press Ctrl+C to stop this test." << std::endl;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Stopping SDK client..." << std::endl;
    PXREADeinit();
    std::cout << "Received state messages: " << context.state_count << std::endl;
    return 0;
}
