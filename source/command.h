#pragma once

namespace Immortals
{
class Command
{
    public:
    Command() = default;
    ~Command() = default;

    void setRobotId(uint8_t id)
    {
        m_id = id;
    }

    bool connect();
    bool isConnected() const;

    bool receive();

    const Immortals::Protos::RobotCommand &getCommand() const
    {
        return m_pb_command;
    }
    

private:
    uint8_t m_id;

    // UDP_connection
    std::unique_ptr<Immortals::UdpClient> m_udp;

    // last received command
    Immortals::Protos::RobotCommand m_pb_command;
};
} // namespace Immortals
