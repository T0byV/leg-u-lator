#pragma once

#include <common.hpp>
#include <i2c/SyncI2CMaster.hpp>

#include <concepts>

class MB85RC64TA {
    public:
    MB85RC64TA(SyncI2CMaster* bus, uint8_t addr): _bus{bus}, _i2c_addr{addr} { }

    void write(uint16_t addr, uint8_t value) noexcept {
        _bus->write<3>(_i2c_addr, {(uint8_t)((addr >> 8) & 0x1F), (uint8_t)(addr & 0xFF), value}, true);
    }

    uint8_t read(uint16_t addr) const noexcept {
        _bus->write<2>(_i2c_addr, {(uint8_t)((addr >> 8) & 0x1F), (uint8_t)(addr & 0xFF)}, true);

        return _bus->read<1>(_i2c_addr, true)[0];
    }

    private:
    SyncI2CMaster* _bus;
    uint8_t _i2c_addr;
};