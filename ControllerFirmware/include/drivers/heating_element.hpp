#pragma once

#include <common.hpp>
#include <hw/PWM.hpp>
#include <drivers/current_sensor.hpp>

class HeatingElement {
    public:
        HeatingElement(uint8_t element_idx, PWM pwm_channel, INA219 power_sensor, float max_power_mw, float calibration_duty_cycle = 0.3, bool debug = false): element_idx{element_idx}, pwm_channel{pwm_channel}, power_sensor{power_sensor}, max_power_mw{max_power_mw}, calibration_duty_cycle{calibration_duty_cycle}, debug{debug} {}

        float calibrate_impedance(int8_t times_averaging = 3) {
            if (debug) printf("DEBUG: calibrating element %d\n", element_idx);

            float pre_calibration_duty_cycle = pwm_channel.get_current_set_duty_cycle();
            float actual_duty_cycle = pwm_channel.set_duty_cycle_safe(calibration_duty_cycle);

            sleep_ms(300);

            float sum_impedance = 0.0;
            for (int i = 0; i < times_averaging; i++)
            {
                sleep_ms(200);
                float bus_voltage_mv = power_sensor.read_bus_voltage();
                float current_ma = power_sensor.read_current();
                sum_impedance = sum_impedance + ((bus_voltage_mv * actual_duty_cycle) / fabs(current_ma));
                if (debug) printf("_impedance: %.3f\tbus_voltage_mv: %.3f\tcurrent_ma: %.3f\tactual_duty_cycle: %.3f\tcalibration_duty_cycle: %.3f\n", _impedance, bus_voltage_mv, current_ma, actual_duty_cycle, calibration_duty_cycle);
            }
            _impedance = sum_impedance / times_averaging;

            // if (debug) printf("_impedance: %.3f\n", _impedance);

            pwm_channel.set_duty_cycle_safe(pre_calibration_duty_cycle);
            return _impedance;
        }

        float set_power_safe(float new_power_mw) {
            if (new_power_mw > max_power_mw) {
                new_power_mw = max_power_mw;
            }    
            return set_power(new_power_mw);
        }

        float get_current_power() {
            float bus_voltage_mv = power_sensor.read_bus_voltage();
            float duty_cycle = pwm_channel.get_current_set_duty_cycle();
            return (0.001 * (bus_voltage_mv * duty_cycle) * (bus_voltage_mv * duty_cycle) / _impedance);
        }

    private:
        uint8_t element_idx;
        PWM pwm_channel;
        INA219 power_sensor;
        float max_power_mw;
        float current_set_power = 0;
        float calibration_duty_cycle;
        bool debug;
        float _impedance = 100000.0;

        float set_power(float new_power_mw) {
            float new_voltage_mv = 1000 * sqrt(0.001 * new_power_mw * _impedance);
            float bus_voltage_mv = power_sensor.read_bus_voltage();
            float new_duty_cycle = new_voltage_mv / bus_voltage_mv;
            float confirmed_new_duty_cycle = pwm_channel.set_duty_cycle_safe(new_duty_cycle);
            if (debug) printf("DEBUG: Element %d\t new_duty_cycle=%.3f\t confirmed_new_duty_cycle=%.3f\n", element_idx, new_duty_cycle, confirmed_new_duty_cycle);
            current_set_power = 0.001 * (bus_voltage_mv * confirmed_new_duty_cycle) * (bus_voltage_mv * confirmed_new_duty_cycle) / _impedance;
            return current_set_power;
        }
};