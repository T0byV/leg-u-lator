#include <common.hpp>

#include <i2c/SyncI2CMaster.hpp>
#include <drivers/fram.hpp>
#include <drivers/current_sensor.hpp>
#include <drivers/temp_sensor.hpp>
#include <functions/powerdata.hpp>
#include <functions/safetycheck.hpp>

// See: https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for core interaction
void core1_entry() {
    while (1)
        ;
}

int main() {
    stdio_init_all();
    printf("Hello world!\n");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

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

    // Battery processing, to be moved somewhere
    Battery bat;                                                    // Initialize battery data processing
    //bat.startup(float powerdata_voltage);                           // Initialize battery data, needs some I2C magic measured voltage in mV

    //bat.update_soc(float powerdata_voltage, float powerdata_current); // Every few seconds: Update battery SoC estimate, needs some I2C magic measured voltage [mV] and current [mA]
    //bat.estimate_life(float pwr_usage_now);                         // Every few seconds: Update estimated battery hours left, needs estimated power usage from feedback model

    // Safety controller, to be moved somewhere
    SafetyControl safety;
    //safety.check_startup(bool i2c_status_power, bool i2c_status_sensorsheating, bool uart_status_ui);         // Check startup connection errors, needs a status boolean of the two I2C lines and UART line
    //safety.check_safety(bool i2c_power, float powerdata_current, float powerdata_voltage, bool i2c_sensorsheating, const std::array<std::array<int32_t, 2>, 6>& temps, const std::array<float, 4>& currents, float pwr_usage_now, const std::array<float, 4>& pwm_heating, int bat_soc, bool uart_ui);   // Every few seconds: Checks for safety concerns, uses basically all data available                                                                                 // Every few seconds: Check for safety concerns
    //safety.alarm(bool severe_error);                                                                          // WIP: behaviour when the safety controller has raised the severe error flag
    //cout << "UI message after safety check + severe flag: " << safety.ui_msg << " - " << safety.severe_error << "\n";         // debugging line
  
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
