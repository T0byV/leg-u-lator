#pragma once

#include <common.hpp>
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <array>

class PWM {
    public:
        // Minimum frequency seems to be 5kHz
        PWM(const uint gpio_pin, float max_duty_cycle = 1.0, uint16_t wrap_resolution = 5000, uint32_t pwm_frequency_hz = 5000) : gpio_pin{gpio_pin}, max_duty_cycle{max_duty_cycle}, wrap_resolution{wrap_resolution}
        {
            uint32_t clk_hz = clock_get_hz(clk_sys);
            float divider = static_cast<float>(clk_hz) / (wrap_resolution * pwm_frequency_hz);

            gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
            uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
            if (debug) printf("pin: %d, slice: %d\n", gpio_pin, slice_num);
            pwm_set_clkdiv(slice_num, divider);

            pwm_set_wrap(slice_num, wrap_resolution);

            // Initialize to 0% duty cycle
            pwm_set_gpio_level(gpio_pin, current_set_duty_cycle);
        }

        void enable() {
            pwm_set_enabled(pwm_gpio_to_slice_num(gpio_pin), true);
        }

        float set_duty_cycle_safe(float new_duty_cycle) noexcept {
            if (new_duty_cycle > max_duty_cycle) {
                new_duty_cycle = max_duty_cycle;
            }    
            return set_duty_cycle(new_duty_cycle);
        }

        float get_current_set_duty_cycle() {
            return current_set_duty_cycle;
        }

    private:
        uint gpio_pin;
        float max_duty_cycle;
        float current_set_duty_cycle = 0.0;
        uint16_t wrap_resolution;

        float set_duty_cycle(float new_duty_cycle) noexcept {
            if (new_duty_cycle <= 0.0) new_duty_cycle = 0.0;
            if (new_duty_cycle >= 1.0) new_duty_cycle = 1.0;

            current_set_duty_cycle = new_duty_cycle;

            uint16_t wrap_corrected_duty_cycle = static_cast<uint16_t>(wrap_resolution * new_duty_cycle);
            pwm_set_gpio_level(gpio_pin, wrap_corrected_duty_cycle);
            return (static_cast<float>(wrap_corrected_duty_cycle) / static_cast<float>(wrap_resolution));
        }
};