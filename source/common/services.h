#pragma once

namespace Immortals::Common
{
namespace Config
{
class Config;
} // namespace Config

class Logger;
class Timer;

struct Services
{
    static bool initialize();
    static void shutdown();

    static Config::Config &config()
    {
        return *s_config;
    }

    static Logger &logger()
    {
        return *s_logger;
    }

    static Timer &global_timer()
    {
        return *s_global_timer;
    }

private:
    static inline Config::Config *s_config;
    static inline Logger         *s_logger;
    static inline Timer          *s_global_timer;
};

inline static Config::Config &config()
{
    return Services::config();
}

inline static Logger &logger()
{
    return Services::logger();
}

inline static Timer &global_timer()
{
    return Services::global_timer();
}
} // namespace Immortals::Common

#include "logging/macros.h"
