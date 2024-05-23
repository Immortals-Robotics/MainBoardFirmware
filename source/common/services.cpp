#include "services.h"

#include "config/config.h"
#include "logging/logging.h"
#include "time/timer.h"

namespace Immortals::Common
{
bool Services::initialize()
{
    s_logger = new Logger();

    s_config = new Config::Config("config.toml");
    s_config->load();

    s_global_timer = new Timer();
    s_global_timer->start();

    return true;
}

void Services::shutdown()
{
    delete s_global_timer;
    delete s_config;
    delete s_logger;
}
} // namespace Immortals::Common
