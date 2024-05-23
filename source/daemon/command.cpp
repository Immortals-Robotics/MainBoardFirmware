#include "command.h"

namespace Immortals::Daemon
{
bool Command::connect()
{
    Common::logInfo("Receiving commands at {}", Common::config().commands_address);

    m_udp = std::make_unique<Common::UdpClient>(Common::config().commands_address);

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
} // namespace Immortals::Daemon
