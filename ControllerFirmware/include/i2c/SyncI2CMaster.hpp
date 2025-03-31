#pragma once

#include <common.hpp>
#include "hardware/i2c.h"

#include <array>
#include <span>

class SyncI2CMaster {
    public:
    SyncI2CMaster(uint8_t instance_number, uint8_t sda_pin = PICO_DEFAULT_I2C_SDA_PIN, uint8_t scl_pin = PICO_DEFAULT_I2C_SCL_PIN, bool use_internal_pullups = false) : instance_number{instance_number}, instance{I2C_INSTANCE(instance_number)} {
        ASSERT(i2c_init(instance, bus_frequency) == bus_frequency); // Initialize I2C peripheral

        gpio_set_function(sda_pin, GPIO_FUNC_I2C); // Setup pins
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);

        if(use_internal_pullups) {
            gpio_pull_up(sda_pin);
            gpio_pull_up(scl_pin);
        }
        
        bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C)); // Make the I2C pins available to picotool
    }

    template<int N>
    std::array<uint8_t, N> read(uint8_t address) const noexcept {
        std::array<uint8_t, N> data{};

        auto n = data.size() * sizeof(uint8_t); // Useless mult, but prevents possible copy-paste errors

        ASSERT(i2c_read_blocking(instance, address, data.data(), n, false) == n);

        return data;
    }

    template<int N>
    void write(uint8_t address, const std::span<uint8_t, N>& data) noexcept {
        ASSERT(i2c_write_blocking(instance, address, data.data(), data.size_bytes(), false) == data.size_bytes());
    }

    private:
    uint8_t instance_number;
    i2c_inst_t* instance;

    constexpr static size_t bus_frequency = 100 * 1000; // Use 100kHz by default
};
