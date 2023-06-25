#pragma once

#include "immortals/micro.pb.h"

namespace Immortals
{
class Micro
{
public:
    Micro() = default;
    ~Micro() = default;

    void requestStatus();

    const Immortals::Protos::MicroStatus& getStatus()
    {
        return m_status;
    }

    void sendCommand(const Immortals::Protos::MicroCommand& t_command);

private:
    Immortals::Protos::MicroStatus  m_status;

    static constexpr size_t kBufferSize = 128;

    char m_rx_buf[kBufferSize] = {};
    char m_tx_buf[kBufferSize] = {};
};
} // namespace Immortals
