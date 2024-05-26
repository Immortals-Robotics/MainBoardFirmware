#pragma once

namespace Immortals::Daemon
{
class Command
{
public:
     Command() = default;
    ~Command() = default;

    void setRobotId(const uint8_t t_id)
    {
        m_id = t_id;
    }

    bool connect();
    bool isConnected() const;

    bool receive();

    const Protos::Immortals::Command &getCommand() const
    {
        return m_pb_command;
    }

private:
    uint8_t m_id;

    // UDP_connection
    std::unique_ptr<Common::UdpClient> m_udp;

    // last received command
    Protos::Immortals::Command m_pb_command;
};
} // namespace Immortals::Daemon
