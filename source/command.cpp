#include "command.h"

namespace Immortals
{
bool Command::connect()
{
    logInfo("Receiving commands at {}:{}", setting().commands_address.ip, setting().commands_address.port);

    m_udp = std::make_unique<UdpClient>(setting().commands_address);

    return isConnected();
}

bool Command::isConnected() const
{
    return m_udp->isConnected();
}

bool Command::receive()
{
    if (!isConnected())
        return false;

    Protos::Immortals::CommandsWrapper pb_packet{};
    if (m_udp->receive(&pb_packet))
    {
        for (auto &command_pb : pb_packet.command())
        {
            if (command_pb.vision_id() == m_id)
            {
                m_pb_command = command_pb;
                return true;
            }
        }
    }

    return false;
}
} // namespace Immortals
