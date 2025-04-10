#pragma once

#include <common.hpp>
#include <hw/SyncI2CMaster.hpp>

#include <concepts>

class MB85RC64TA {
    public:
    MB85RC64TA(SyncI2CMaster& bus, uint8_t addr): bus{bus}, i2c_addr{addr} { }

    template<typename T> requires std::integral<T>
    void write(uint16_t addr, T value) noexcept {
        bus.write<2>(i2c_addr, {(uint8_t)(addr & 0xFF), (uint8_t)((addr >> 8) & 0x1F)}, false);

        for(int i = 0; i < sizeof(T); i++)
            bus.write<1>(i2c_addr, {(uint8_t)((value >> 8 * i) & 0xFF)}, i == (sizeof(T) - 1));
    }

    template<typename T> requires std::integral<T>
    T read(uint16_t addr) const noexcept {
        bus.write<2>(i2c_addr, {(uint8_t)(addr & 0xFF), (uint8_t)((addr >> 8) & 0x1F)}, true);

        T tmp = 0;
        for(int i = 0; i < sizeof(T); i++)
            tmp |= bus.read<1>(i2c_addr, i == (sizeof(T) - 1))[0] << (8 * i);

        return tmp;
    }

    private:
    SyncI2CMaster& bus;
    uint8_t i2c_addr;
};