#pragma once
#include "logging/logging.h"
#include "setting.h"

namespace Immortals
{
struct Services
{
    static void initialize()
    {
        const ConfigReader config("config.toml");
        s_setting = new Setting();
        s_setting->load(config.getRoot());

        s_logger = new Logger();
    }

    static void shutdown()
    {
        delete s_setting;
        delete s_logger;
    }

    static const Setting &setting()
    {
        return *s_setting;
    }

    static Logger &logger()
    {
        return *s_logger;
    }

private:
    static inline Setting *s_setting;
    static inline Logger  *s_logger;
};

static const Setting &setting()
{
    return Services::setting();
}

static Logger &logger()
{
    return Services::logger();
}
} // namespace Immortals

#include "logging/macros.h"
