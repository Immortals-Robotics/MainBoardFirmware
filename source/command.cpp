#include "command.h"

#include "immortals/command.pb.h"

namespace Immortals
{
bool Command::connect()
{
    std::cout << "Receiving commands at " << setting().commands_address.ip
              << " , on port : " << setting().commands_address.port << std::endl;

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
