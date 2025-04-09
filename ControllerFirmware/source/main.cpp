#include <common.hpp>

#include <i2c/SyncI2CMaster.hpp>
//#include <drivers/fram.hpp>
//#include <drivers/temp_sensor.hpp>
#include <drivers/current_sensor.hpp>

// See: https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for core interaction
void core1_entry() {
    while (1)
        ;
}

int main() {
    stdio_init_all();

    multicore_launch_core1(core1_entry);

    SyncI2CMaster bus0{0, 4, 5, true};
    /*Sensor<6, 2> sensing{&bus0, {{
        {0x50, 0x58}, // 1 Red
        {0x51, 0x59}, // 2 Orange
        {0x52, 0x5A}, // 3 Yellow
        {0x53, 0x5B}, // 4 Green
        {0x54, 0x5C}, // 5 Blue
        {0x55, 0x5D} // 6 Purple
    }}};*/
    
    std::array<INA219, 4> heating_power_sensors = {{
        {&bus0, 0x45, 1, 0.101}, // Channel 1
        {&bus0, 0x44, 1, 0.1012}, // Channel 2
        {&bus0, 0x41, 1, 0.1005}, // Channel 3
        {&bus0, 0x40, 1, 0.1006}  // Channel 4
    }};

    bool s = false;
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, s);
        s = !s;

        printf("%f %f %f %f %f\n", heating_power_sensors[0].read_bus_voltage(), heating_power_sensors[0].read_current(), heating_power_sensors[1].read_current(), heating_power_sensors[2].read_current(), heating_power_sensors[3].read_current());
        //printf("%f %f %f %f\n", heating_power_sensors[0].read_bus_voltage(), heating_power_sensors[0].read_shunt_voltage(), heating_power_sensors[0].read_current(), (float)heating_power_sensors[0].read_shunt_voltage() / (float)heating_power_sensors[0].read_current());

        //auto temps = sensing();
        //printf("%d,%d,%d,%d,%d,%d,%d\n", time_us_32(), temps[0][0], temps[1][0], temps[2][0], temps[3][0], temps[4][0], temps[5][0]);
        //printf("%d,%d\n", time_us_32(), temps[0]);
    }
}
