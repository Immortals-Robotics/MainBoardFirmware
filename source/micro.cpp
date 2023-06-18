#include "micro.h"

#include <google/protobuf/util/delimited_message_util.h>

#include "spi_hal.h"

void Micro::requestStatus()
{
    Immortals::Protos::MicroCommand command{};

    sendCommand(command);
}

void Micro::sendCommand(const Immortals::Protos::MicroCommand &t_command)
{
    google::protobuf::io::ArrayOutputStream output_stream{m_tx_buf, kBufferSize};
    google::protobuf::util::SerializeDelimitedToZeroCopyStream(t_command, &output_stream);

    micro_xfer(m_tx_buf, m_rx_buf, kBufferSize);

    google::protobuf::io::ArrayInputStream input_stream{m_rx_buf, kBufferSize};
    google::protobuf::util::ParseDelimitedFromZeroCopyStream(&m_status, &input_stream, nullptr);
}
