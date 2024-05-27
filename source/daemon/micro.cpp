#include "micro.h"

#include "spi_hal.h"

namespace Immortals::Daemon
{
void Micro::requestStatus()
{
    sendCommand({});
}

void Micro::sendCommand(const Protos::Immortals::MicroCommand &t_command)
{
    google::protobuf::io::ArrayOutputStream output_stream{m_tx_buf.data(), kBufferSize};
    google::protobuf::util::SerializeDelimitedToZeroCopyStream(t_command, &output_stream);

    micro_xfer(m_tx_buf.data(), m_rx_buf.data(), kBufferSize);

    m_status.Clear();
    google::protobuf::io::ArrayInputStream input_stream{m_rx_buf.data(), kBufferSize};
    google::protobuf::util::ParseDelimitedFromZeroCopyStream(&m_status, &input_stream, nullptr);
}
} // namespace Immortals::Daemon
