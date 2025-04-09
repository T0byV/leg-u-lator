#pragma once


#include <common.hpp>
#include <i2c/SyncI2CMaster.hpp>

class INA219 {
    public:

    enum class BusVoltageRange { FSR16V = 0, FSR32V = 1 };
    enum class PGARange { _40mV, _80mV, _160mV, _320mV };


    INA219() = default;
    INA219(SyncI2CMaster* bus, uint8_t addr, double max_current, double r_shunt): _bus{bus}, _i2c_addr{addr} {
        _setup_val = read_reg(reg_config);
        _setup_val |= (0b1111 << 3) | (0b1111 << 7) | (0b111); // 12 bit 128 averages for both ADCs, both shunt and bus voltage continuous
        write_reg(reg_config, _setup_val);

        set_fullscale_range(BusVoltageRange::FSR32V);
        //set_pga_range(PGARange::_40mV);

        current_lsb = max_current / 32768;
        _cal_val = static_cast<uint16_t>(0.04096 / (current_lsb * r_shunt));
        write_reg(reg_calib, _cal_val);
    }

    void reset() noexcept {
        write_reg(reg_config, _setup_val | (1 << 15));
    }

    void set_fullscale_range(BusVoltageRange range) noexcept {
        _setup_val = read_reg(reg_config);

        using enum BusVoltageRange;
        switch (range) {
            case FSR16V: _setup_val &= ~(1 << 13); break;
            case FSR32V: _setup_val |= (1 << 13); break;
        }

        write_reg(reg_config, _setup_val);
    }

    void set_pga_range(PGARange range) {
        _setup_val = read_reg(reg_config);
        _setup_val &= ~(0b11 << 11);

        using enum PGARange;
        switch (range) {
            case _40mV: _setup_val |= (0b00 << 11); break;
            case _80mV: _setup_val |= (0b01 << 11); break;
            case _160mV: _setup_val |= (0b10 << 11); break;
            case _320mV: _setup_val |= (0b11 << 11); break;
        }

        write_reg(reg_config, _setup_val);
    }

    // Returns shunt voltage in signed mV
    float read_shunt_voltage() noexcept {
        return static_cast<float>(read_reg(reg_shunt_v)) / 100.0f;
    }

    // Returns shunt current in mA
    float read_current() noexcept {
        write_reg(reg_calib, _cal_val);
        return (static_cast<float>(read_reg(reg_current)) * current_lsb) * 1000.0f;
    }

    // Returns bus voltage in mV
    float read_bus_voltage() noexcept {
        return 4 * (read_reg(reg_bus_v) >> 3);
    }

    float read_power() noexcept {
        return (static_cast<float>(read_reg(reg_power)) * current_lsb * 20.0f) * 1000.0f;
    }

    private:
    void write_reg(uint8_t reg, uint16_t data) noexcept {
        _bus->write<3>(_i2c_addr, {(uint8_t)reg, (data >> 8) & 0xFF, data & 0xFF}, true);
    }

    uint16_t read_reg(uint8_t reg) noexcept {
        _bus->write<1>(_i2c_addr, {(uint8_t)reg}, false);
        auto data = _bus->read<2>(_i2c_addr, true);

        return (uint16_t)data[1] | ((uint16_t)data[0] << 8);
    }

    SyncI2CMaster* _bus;
    uint8_t _i2c_addr;
    uint16_t _setup_val, _cal_val;
    double current_lsb;

    constexpr static uint8_t reg_config = 0;
    constexpr static uint8_t reg_shunt_v = 1;
    constexpr static uint8_t reg_bus_v = 2;
    constexpr static uint8_t reg_power = 3;
    constexpr static uint8_t reg_current = 4;
    constexpr static uint8_t reg_calib = 5;
};