#include <common.hpp>

#include <hw/PWM.hpp>
#include <hw/SyncI2CMaster.hpp>
#include <hw/UART.hpp>
#include <drivers/fram.hpp>
#include <drivers/current_sensor.hpp>
//#include <drivers/fram.hpp>
#include <drivers/temp_sensor.hpp>
#include <drivers/heating_element.hpp>

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

    float max_duty_cycle = 0.2;

    std::array<PWM, 4> heating_pwm_channels = {{
        {6, max_duty_cycle},
        {7, max_duty_cycle},
        {8, max_duty_cycle},
        {9, max_duty_cycle}
    }};

    float calibration_duty_cycle = 0.3;
    float max_power_mw_per_element = 2000;
    std::array<HeatingElement, 4> heating_elements = {{
        {0, heating_pwm_channels[0], heating_power_sensors[0], max_power_mw_per_element, calibration_duty_cycle},
        {1, heating_pwm_channels[1], heating_power_sensors[1], max_power_mw_per_element, calibration_duty_cycle},
        {2, heating_pwm_channels[2], heating_power_sensors[2], max_power_mw_per_element, calibration_duty_cycle},
        {3, heating_pwm_channels[3], heating_power_sensors[3], max_power_mw_per_element, calibration_duty_cycle}
    }};

    UART uart_bus{uart1, 4, 5};

    // bool s = false;

    printf("First calibration\n");
    for (int i = 0; i < 4; i++)
    {
        printf("%.2fohm\t", heating_elements[i].calibrate_impedance());
    }
    printf("\n");


    float desired_power_mw = 150;
    int calibration_cycle_counter = 0;
    int calibration_after_cycles = 10;
    int cycle_duration_ms = 1000;

    while (true)
    {
        // gpio_put(PICO_DEFAULT_LED_PIN, s);
        // s = !s;
        
        printf("Cycle start\n");

        if (calibration_cycle_counter >= calibration_after_cycles) {
            printf("Calibrating\n");
            for (int i = 0; i < 4; i++){
                printf("%.2fohm\t", heating_elements[i].calibrate_impedance());
            }
            printf("\n");
            calibration_cycle_counter = 0;
        }
        calibration_cycle_counter++;


        for (int i = 0; i < 4; i++)
        {
            heating_elements[i].set_power_safe(desired_power_mw);
            sleep_ms(200);
            printf("%.2fmW\t", heating_elements[i].get_current_power());
        }
        printf("\n");

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
