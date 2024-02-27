#pragma once

#include "config/config.h"

namespace Immortals
{
struct NetworkAddress final : public IConfig
{
    NetworkAddress() = default;

    NetworkAddress(const std::string_view ip, const unsigned short port)
        : ip(ip), port(port)
    {}

    void load(toml::node_view<const toml::node> t_node) override
    {
        ip   = t_node["ip"].value_or(ip);
        port = t_node["port"].value_or(port);
    }

    std::string    ip;
    unsigned short port = 0;
};

template <typename Enum>
void fillEnum(toml::node_view<const toml::node> t_node, Enum &t_enum)
{
    t_enum = static_cast<Enum>(t_node.value_or(static_cast<int>(t_enum)));
}

struct Setting : IConfig
{
private:
    Setting()  = default;
    ~Setting() = default;

    void load(toml::node_view<const toml::node> t_node) override;

    friend struct Services;

public:
    Setting(const Setting &)            = delete;
    Setting &operator=(const Setting &) = delete;

    static constexpr size_t kMaxUdpPacketSize = 1472;

     NetworkAddress commands_address = {"224.5.23.2", 10010};
};

inline void Setting::load(const toml::node_view<const toml::node> t_node)
{
    const auto network = t_node["network"];

    commands_address.load(network["commands"]);
}

} // namespace Immortals
