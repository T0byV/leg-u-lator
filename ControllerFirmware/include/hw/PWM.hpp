#pragma once

#include <common.hpp>
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <array>

template<int N>
class PWM {
    public:
        // Minimum frequency seems to be 5kHz
        PWM(const std::array<uint,N>& pwm_pins, uint16_t max_duty_cycle = 40, uint16_t wrap_resolution = 100, uint32_t pwm_frequency_hz = 5000) : pwm_pins{pwm_pins}, wrap_resolution{wrap_resolution}
        {
            uint32_t clk_hz = clock_get_hz(clk_sys);
            float divider = static_cast<float>(clk_hz) / (wrap_resolution * pwm_frequency_hz);

            for (uint pin : pwm_pins)
            {
                gpio_set_function(pin, GPIO_FUNC_PWM);
                uint slice_num = pwm_gpio_to_slice_num(pin);

                pwm_set_clkdiv(slice_num, divider);

                pwm_set_wrap(slice_num, wrap_resolution);

                // Initialize to 0% duty cycle
                pwm_set_gpio_level(pin, 0);
            }

            // Only enable once we're sure all pins are initaliased at 0% duty-cycle
            enable_all();
        }

        void set_duty_cycle(uint pin, uint16_t duty_percentage) noexcept {
            if (duty_percentage <= 0)
                duty_percentage = 0;
            if (duty_percentage >= 100)
                duty_percentage = 100;
    
            uint level = wrap_resolution * duty_percentage * 0.01;
            pwm_set_gpio_level(pin, level);
        }

        void set_duty_cycle_safe(uint pin, uint16_t duty_percentage) noexcept {
            if (duty_percentage > max_duty_cycle) {
                duty_percentage = max_duty_cycle;
            }
            set_duty_cycle(pin, duty_percentage);
        }

        void enable_all() {
            for (uint pin : pwm_pins)
                pwm_set_enabled(pwm_gpio_to_slice_num(pin), true);
        }
        void disable_all() {
            for (uint pin : pwm_pins)
                pwm_set_enabled(pwm_gpio_to_slice_num(pin), false);
        }

    private:
        const std::array<uint, N>& pwm_pins;
        uint16_t max_duty_cycle;
        uint16_t wrap_resolution;
};