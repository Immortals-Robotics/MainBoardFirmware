#pragma once

#include "../network/address.h"
#include "file.h"

namespace Immortals::Common::Config
{
class Config
{
private:
    Config(const std::filesystem::path &t_file_name) : m_file(t_file_name)
    {}

    ~Config() = default;

    friend struct ::Immortals::Common::Services;

    File m_file;

public:
    Config(const Config &)            = delete;
    Config &operator=(const Config &) = delete;

    void load()
    {
        m_file.load();

        const toml::table &root = m_file.root();

        const auto network = root["network"];

        commands_address.load(network["commands"]);
    }

    void save() const
    {
        m_file.save();
    }

    const toml::table &root() const
    {
        return m_file.root();
    }

    toml::table &root()
    {
        return m_file.root();
    }

    static constexpr size_t kMaxUdpPacketSize = 1472;

    NetworkAddress commands_address = {"224.5.23.2", 10010};
};
} // namespace Immortals::Common::Config
