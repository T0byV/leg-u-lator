#pragma once

#include <common.hpp>
#include <hw/SyncI2CMaster.hpp>

class PJ85775 {
    public:
    PJ85775() = default;
    PJ85775(SyncI2CMaster* bus, uint8_t addr): _bus{bus}, _i2c_addr{addr} {
        _setup_val = read_reg(reg_setup);
        _setup_val |= ((0b11 << 13) | (1 << 10) | (1 << 8)); // 0.25 Hz conversion, Active-High polarity, Shutdown
        write_reg(reg_setup, _setup_val);
    }

    void begin_conversion() noexcept {
        write_reg(reg_setup, _setup_val | (1 << 15)); // Set One-shot bit
        _conversion_running = true;
    }

    // If conversion has not been started yet, starts conversion
    int32_t block_for_conversion(uint16_t timeout_ms = 100) noexcept {
        if(!_conversion_running)
            begin_conversion();
        
        auto time_timeout = time_us_32() + 1000 * timeout_ms;
        while((read_reg(reg_setup) & (1 << 8)) == 0) {
            if(time_us_32() > time_timeout)
                return INT32_MAX;
        }

        auto frac = read_reg(reg_data);
        _conversion_running = false;

        // static_cast<int12_t>(frac >> 4)) * 625); of dit in 10^-4kelvin
        int32_t millis = (static_cast<int32_t>(static_cast<int8_t>(frac >> 8)) * 1000) + ((frac >> 7) & 1) * 500 + ((frac >> 6) & 1) * 250 + ((frac >> 5) & 1) * 125 + ((frac >> 4) & 1) * 63;
        return millis;
    }

    private:
    void write_reg(uint8_t reg, uint16_t data) noexcept {
        _bus->write<3>(_i2c_addr, {(uint8_t)reg, static_cast<uint8_t>((data >> 8) & 0xFF), static_cast<uint8_t>(data & 0xFF)}, true);
    }

    uint16_t read_reg(uint8_t reg) noexcept {
        _bus->write<1>(_i2c_addr, {(uint8_t)reg}, false);
        auto data = _bus->read<2>(_i2c_addr, true);

        return (uint16_t)data[1] | ((uint16_t)data[0] << 8);
    }

    SyncI2CMaster* _bus;
    uint8_t _i2c_addr;
    uint16_t _setup_val;
    bool _conversion_running;

    constexpr static uint8_t reg_data = 0;
    constexpr static uint8_t reg_setup = 1;
};

template<int N>
class Cluster {
    public:
    Cluster() = default;
    Cluster(SyncI2CMaster* bus, const std::array<uint8_t, N>& addresses) {
        for(int i = 0; i < N; i++)
            _sensors[i] = PJ85775{bus, addresses[i]};
    }

    void begin_conversion() noexcept {
        for(auto& sensor : _sensors)
            sensor.begin_conversion();

        _conversion_running = true;
    }

    std::array<int32_t, N> block_for_conversion() noexcept {
        if(!_conversion_running)
            begin_conversion();
        
        std::array<int32_t, N> temps{};
        for(int i = 0; i < N; i++)
            temps[i] = _sensors[i].block_for_conversion();

        _conversion_running = false;

        return temps;
    }

    private:
    std::array<PJ85775, N> _sensors;
    bool _conversion_running;
};

//PJ85775 sensor{bus0, 0x52};
//Cluster<2> cluster{&bus0, {0x52, 0x5A}};
template<int N, int M>
class Sensor {
    public:
    Sensor(SyncI2CMaster* bus, const std::array<std::array<uint8_t, M>, N>& cluster_addresses) {
        for(int i = 0; i < N; i++)
            _clusters[i] = Cluster<M>{bus, cluster_addresses[i]};
    }

    std::array<std::array<int32_t, M>, N> operator()() noexcept {
        for(auto& cluster : _clusters)
            cluster.begin_conversion();
        
        std::array<std::array<int32_t, M>, N> temps{};
        for(int i = 0; i < N; i++)
            temps[i] = _clusters[i].block_for_conversion();

        return temps;
    }

    private:
    std::array<Cluster<M>, N> _clusters;
};