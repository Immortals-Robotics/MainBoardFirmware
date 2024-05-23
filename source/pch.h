#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <thread>

#include <array>
#include <deque>
#include <set>
#include <unordered_map>
#include <vector>

#include <asio.hpp>
#include <google/protobuf/message_lite.h>
#include <google/protobuf/util/delimited_message_util.h>
#include <pigpiod_if2.h>
#include <quill/Quill.h>
#include <toml++/toml.h>

#include "config/config.h"
#include "logging/logging.h"
#include "network/udp_client.h"
#include "network/udp_server.h"
#include "services.h"
#include "setting.h"
