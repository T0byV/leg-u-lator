#include <common.hpp>

#include <hw/PWM.hpp>
#include <hw/SyncI2CMaster.hpp>
#include <hw/UART.hpp>
#include <drivers/fram.hpp>
#include <drivers/current_sensor.hpp>
//#include <drivers/fram.hpp>
#include <drivers/temp_sensor.hpp>

// See: https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for core interaction
void core1_entry() {
    while (1)
        ;
}

int main() {
    stdio_init_all();
    sleep_ms(5000);
    printf("Hello world!\n");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    multicore_launch_core1(core1_entry);

    float max_duty_cycle = 0.3;
    PWM<4> heating_pwm{
        {{6, 7, 8, 9}}, // GPIO pins used
        max_duty_cycle
    };

    // std::array<PWM, 4> heating_pwm_channels = {{

    // }}
    // EXAMPLE: heating_pwm.set_duty_cycle_safe(6, 40); sets pin 6 to 40% duty cycle

    SyncI2CMaster bus0{0, 20, 21, false};
    Sensor<6, 2> sensing{&bus0, {{
        {0x50, 0x58}, // 1 Red
        {0x51, 0x59}, // 2 Orange
        {0x52, 0x5A}, // 3 Yellow
        {0x53, 0x5B}, // 4 Green
        {0x54, 0x5C}, // 5 Blue
        {0x55, 0x5D} // 6 Purple
    }}};
    
    std::array<INA219, 4> heating_power_sensors = {{
        {&bus0, 0x45, 1, 0.101}, // Channel 1
        {&bus0, 0x44, 1, 0.1012}, // Channel 2
        {&bus0, 0x41, 1, 0.1005}, // Channel 3
        {&bus0, 0x40, 1, 0.1006}  // Channel 4
    }};

    UART uart_bus{uart1, 4, 5};

    // bool s = false;

    float actuator_impedances[4] = {};
    float calibration_duty_cycle = 0.3;

    printf("INFO: Calibrating\n");
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    for (int i = 0; i < 4; i++)
    {
        printf("element %d: ", i);

        heating_pwm.set_duty_cycle_safe((i + 6), calibration_duty_cycle);
        sleep_ms(750);
        float bus_voltage_mv = heating_power_sensors[i].read_bus_voltage();
        float bus_current_iv = heating_power_sensors[i].read_current();
        float impedance = (bus_voltage_mv * calibration_duty_cycle) / fabs(bus_current_iv);
        actuator_impedances[i] = impedance;
        printf("voltage: %.2fmV, current: %.2fmA, impedance: %.2f\n", bus_voltage_mv, bus_current_iv, impedance);
        heating_pwm.set_duty_cycle_safe((i + 6), 0.0);
    }
    gpio_put(PICO_DEFAULT_LED_PIN, false);


    float desired_power_mw = 150;
    int calibration_cycle_counter = 0;
    int calibration_after_cycles = 20;
    int cycle_duration_ms = 1000;

    float set_duty_cycle[4] = {0.0, 0.0, 0.0, 0.0};
    float confirmed_cycle[4] = {0.0, 0.0, 0.0, 0.0};
    while (true)
    {
        // gpio_put(PICO_DEFAULT_LED_PIN, s);
        // s = !s;
        
        printf("Cycle start\n");

        if (calibration_cycle_counter >= calibration_after_cycles) {
            printf("INFO: Calibrating\n");
            gpio_put(PICO_DEFAULT_LED_PIN, true);
            for (int i = 0; i < 4; i++)
            {
                printf("element %d: ", i);
                float old_duty_cycle = set_duty_cycle[i];
                heating_pwm.set_duty_cycle_safe((i + 6), calibration_duty_cycle);
                sleep_ms(750);
                float bus_voltage_mv = heating_power_sensors[i].read_bus_voltage();
                float bus_current_iv = heating_power_sensors[i].read_current();
                float impedance = (bus_voltage_mv * calibration_duty_cycle) / fabs(bus_current_iv);
                actuator_impedances[i] = impedance;
                printf("voltage: %.2fmV, current: %.2fmA, impedance: %.2f\n", bus_voltage_mv, bus_current_iv, impedance);
                heating_pwm.set_duty_cycle_safe((i + 6), old_duty_cycle);
                sleep_ms(750);
            }
            gpio_put(PICO_DEFAULT_LED_PIN, false);
            calibration_cycle_counter = 0;
        }
        calibration_cycle_counter++;


        for (int i = 0; i < 4; i++)
        {
            float current = (confirmed_cycle[i] * heating_power_sensors[i].read_bus_voltage());

            float measured_power_mw = 0.001 * (current * current) / actuator_impedances[i];
            float delta_power_mw = desired_power_mw - measured_power_mw;

            // start controller
            if (delta_power_mw <= 0.0)
                set_duty_cycle[i] = set_duty_cycle[i] - 0.001;
            else if (delta_power_mw > 0.0)
                set_duty_cycle[i] = set_duty_cycle[i] + 0.001;
            
            if (set_duty_cycle[i] > max_duty_cycle) set_duty_cycle[i] = max_duty_cycle;
            if (set_duty_cycle[i] < 0.0) set_duty_cycle[i] = 0.0;
            // end controller
            confirmed_cycle[i] = heating_pwm.set_duty_cycle_safe((i + 6), set_duty_cycle[i]);
            printf("Delta: %.2fmW, MeasPow: %.2fmW, DesiredPow: %.2fmW, current: %.2fmA, NewDutyCycle: %.3f, ConfirmedDutyCycle: %.3f\n", delta_power_mw, measured_power_mw, desired_power_mw, current, set_duty_cycle[i], confirmed_cycle[i]);
        }

        sleep_ms(cycle_duration_ms);

        // uart_bus.write("b47#");
        // uart_bus.write("c25452#");
        // uart_bus.write("d25000#");
        // // uart_bus.write("u26000#");
        // uart_bus.write("e3#");
        // uart_bus.write("w5#");
        // printf("%f %f %f %f %f\n", heating_power_sensors[0].read_bus_voltage(), heating_power_sensors[0].read_current(), heating_power_sensors[1].read_current(), heating_power_sensors[2].read_current(), heating_power_sensors[3].read_current());
        //printf("%f %f %f %f\n", heating_power_sensors[0].read_bus_voltage(), heating_power_sensors[0].read_shunt_voltage(), heating_power_sensors[0].read_current(), (float)heating_power_sensors[0].read_shunt_voltage() / (float)heating_power_sensors[0].read_current());

        // auto temps = sensing();

        // printf("%d,%d,%d,%d,%d,%d,%d\n", time_us_32(), temps[0][0], temps[1][0], temps[2][0], temps[3][0], temps[4][0], temps[5][0]);
        // printf("%d, %d, %d\n", time_us_32(), temps[0][0], temps[0][1]);

        // printf("%d,%d\n", time_us_32(), temps[0]);

    }
}
