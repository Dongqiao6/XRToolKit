#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <PXREARobotSDK.h>

namespace {

volatile std::sig_atomic_t g_running = 1;

void handle_signal(int) {
    g_running = 0;
}

const char* callback_name(PXREAClientCallbackType type) {
    switch (type) {
        case PXREAServerConnect:
            return "SERVER_CONNECT";
        case PXREAServerDisconnect:
            return "SERVER_DISCONNECT";
        case PXREADeviceFind:
            return "DEVICE_FIND";
        case PXREADeviceMissing:
            return "DEVICE_MISSING";
        case PXREADeviceConnect:
            return "DEVICE_CONNECT";
        case PXREADeviceStateJson:
            return "DEVICE_STATE_JSON";
        case PXREADeviceCustomMessage:
            return "DEVICE_CUSTOM_MESSAGE";
        default:
            return "UNKNOWN";
    }
}

template <std::size_t N>
std::string fixed_string(const char (&value)[N]) {
    return std::string(value, strnlen(value, N));
}

std::string json_function_name(const std::string& json) {
    const std::string key = "\"functionName\"";
    std::size_t position = json.find(key);
    if (position == std::string::npos) {
        return {};
    }

    position = json.find(':', position + key.size());
    if (position == std::string::npos) {
        return {};
    }

    position = json.find('"', position + 1);
    if (position == std::string::npos) {
        return {};
    }

    const std::size_t end = json.find('"', position + 1);
    if (end == std::string::npos) {
        return {};
    }
    return json.substr(position + 1, end - position - 1);
}

struct TestContext {
    std::mutex mutex;
    std::ofstream output;
    std::uint64_t state_count = 0;
    std::uint64_t custom_message_count = 0;
    std::string last_device_id;
    std::chrono::steady_clock::time_point started_at = std::chrono::steady_clock::now();
};

void print_device_event(TestContext& context,
                        PXREAClientCallbackType type,
                        int status,
                        void* user_data) {
    std::lock_guard<std::mutex> lock(context.mutex);
    std::cout << "[EVENT] " << callback_name(type) << " status=" << status;
    if (user_data != nullptr) {
        const char* text = static_cast<const char*>(user_data);
        if (text[0] != '\0') {
            std::cout << " device=" << text;
        }
    }
    std::cout << std::endl;
}

void on_pxrea_callback(void* callback_context,
                       PXREAClientCallbackType type,
                       int status,
                       void* user_data) {
    auto* context = static_cast<TestContext*>(callback_context);
    if (context == nullptr) {
        std::cerr << "[ERROR] Callback context is null" << std::endl;
        return;
    }

    switch (type) {
        case PXREAServerConnect:
        case PXREAServerDisconnect: {
            std::lock_guard<std::mutex> lock(context->mutex);
            std::cout << "[EVENT] " << callback_name(type)
                      << " status=" << status << std::endl;
            return;
        }

        case PXREADeviceFind:
        case PXREADeviceMissing:
        case PXREADeviceConnect:
            print_device_event(*context, type, status, user_data);
            return;

        case PXREADeviceStateJson: {
            if (user_data == nullptr) {
                std::lock_guard<std::mutex> lock(context->mutex);
                std::cerr << "[WARN] DEVICE_STATE_JSON with null payload" << std::endl;
                return;
            }

            // The SDK passes PXREADevStateJson, not a plain NUL-terminated string.
            const auto& state = *static_cast<const PXREADevStateJson*>(user_data);
            const std::string device_id = fixed_string(state.devID);
            const std::string json = fixed_string(state.stateJson);

            std::lock_guard<std::mutex> lock(context->mutex);
            ++context->state_count;
            context->last_device_id = device_id;

            if (context->output.is_open()) {
                // One device state JSON per line. The file can be analysed with jq.
                context->output << json << '\n';
                context->output.flush();
            }

            if (context->state_count == 1) {
                const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - context->started_at);
                std::cout << "[DATA] First Quest state received after "
                          << elapsed.count() << " ms" << std::endl;
            }

            // Tracking can arrive near 90 Hz. Print a small sample and keep all
            // messages in the JSONL file to avoid flooding the terminal.
            if (context->state_count <= 3 || context->state_count % 90 == 0) {
                const std::string function = json_function_name(json);
                std::cout << "\n[DATA #" << context->state_count
                          << "] device=" << device_id
                          << " status=" << status;
                if (!function.empty()) {
                    std::cout << " function=" << function;
                }
                std::cout << "\n" << json << "\n" << std::flush;
            }
            return;
        }

        case PXREADeviceCustomMessage: {
            std::lock_guard<std::mutex> lock(context->mutex);
            ++context->custom_message_count;
            if (user_data != nullptr) {
                const auto& message = *static_cast<const PXREADevCustomMessage*>(user_data);
                std::cout << "[CUSTOM] device=" << fixed_string(message.devID)
                          << " size=" << message.dataSize << " bytes" << std::endl;
            }
            return;
        }

        default: {
            std::lock_guard<std::mutex> lock(context->mutex);
            std::cout << "[EVENT] UNKNOWN type=" << static_cast<int>(type)
                      << " status=" << status << std::endl;
            return;
        }
    }
}

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [seconds] [output.jsonl]\n"
              << "  seconds       Capture duration; 0 means run until Ctrl+C (default: 0)\n"
              << "  output.jsonl  Output file (default: xrt_received.jsonl)\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc > 1 &&
        (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0)) {
        print_usage(argv[0]);
        return 0;
    }

    int duration_seconds = 0;
    if (argc > 1) {
        try {
            duration_seconds = std::stoi(argv[1]);
        } catch (const std::exception&) {
            std::cerr << "Invalid duration: " << argv[1] << std::endl;
            print_usage(argv[0]);
            return 2;
        }
        if (duration_seconds < 0) {
            std::cerr << "Duration must be zero or positive" << std::endl;
            return 2;
        }
    }
    const std::string output_path = argc > 2 ? argv[2] : "xrt_received.jsonl";

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    TestContext context;
    context.output.open(output_path, std::ios::out | std::ios::trunc);
    if (context.output.is_open()) {
        std::cout << "[INFO] Raw state JSON: " << output_path << std::endl;
    } else {
        std::cerr << "[WARN] Cannot open " << output_path
                  << "; data will only be printed to the terminal" << std::endl;
    }

    std::cout << "[INFO] Initializing XRoboToolkit PC-service client..." << std::endl;
    const int init_result = PXREAInit(&context, on_pxrea_callback, PXREAFullMask);
    if (init_result != 0) {
        std::cerr << "[ERROR] PXREAInit failed, code=" << init_result << '\n'
                  << "Check that PC-service is running and libPXREARobotSDK.so is loadable."
                  << std::endl;
        return 1;
    }

    std::cout << "[INFO] SDK initialized. Keep the Quest app open and enable "
                 "Tracking/Send.\n"
              << "[INFO] Waiting for Quest data";
    if (duration_seconds > 0) {
        std::cout << " for " << duration_seconds << " seconds";
    } else {
        std::cout << "; press Ctrl+C to stop";
    }
    std::cout << "..." << std::endl;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(duration_seconds);
    while (g_running &&
           (duration_seconds == 0 || std::chrono::steady_clock::now() < deadline)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[INFO] Stopping SDK client..." << std::endl;
    PXREADeinit();

    std::lock_guard<std::mutex> lock(context.mutex);
    if (context.output.is_open()) {
        context.output.close();
    }
    std::cout << "[INFO] Received " << context.state_count << " state messages";
    if (!context.last_device_id.empty()) {
        std::cout << " from device " << context.last_device_id;
    }
    if (context.custom_message_count > 0) {
        std::cout << ", " << context.custom_message_count << " custom messages";
    }
    std::cout << std::endl;

    return context.state_count > 0 ? 0 : 3;
}
