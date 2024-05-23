#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <ostream>
#include <random>
#include <source_location>
#include <span>
#include <string.h>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/std.h>
#include <nng/nng.h>
#include <nng/protocol/pubsub0/pub.h>
#include <nng/protocol/pubsub0/sub.h>
#include <spdlog/async.h>
#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

#include <asio.hpp>

#include <google/protobuf/message_lite.h>

#include "services.h"
