#pragma once

namespace Immortals::Daemon
{
class Micro
{
public:
    Micro()  = default;
    ~Micro() = default;

    void requestStatus();

    const Protos::Immortals::MicroStatus &getStatus()
    {
        return m_status;
    }

    void sendCommand(const Protos::Immortals::MicroCommand &t_command);

private:
    Protos::Immortals::MicroStatus m_status;

    static constexpr size_t kBufferSize = 128;

    char m_rx_buf[kBufferSize] = {};
    char m_tx_buf[kBufferSize] = {};
};
} // namespace Immortals::Daemon
