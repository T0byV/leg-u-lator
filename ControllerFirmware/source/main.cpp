#include <common.hpp>

#include <hw/PWM.hpp>
#include <hw/SyncI2CMaster.hpp>
#include <hw/UART.hpp>
#include <drivers/fram.hpp>
#include <drivers/current_sensor.hpp>
//#include <drivers/fram.hpp>
#include <drivers/temp_sensor.hpp>

UART* irq_userptrs[2] = { nullptr, nullptr };

extern void on_uart0_irq() {
    if (irq_userptrs[0]) irq_userptrs[0]->on_uart_rx();
}

extern void on_uart1_irq() {
    if (irq_userptrs[1]) irq_userptrs[0]->on_uart_rx();
}
// See: https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for core interaction
void core1_entry() {
    while (1)
        ;
}

int main() {
    stdio_init_all();
    sleep_ms(3000);
    printf("Hello world!\n");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    multicore_launch_core1(core1_entry);

    PWM<4> heating_pwm{
        {{6, 7, 8, 9}} // GPIO pins used
    };
    // EXAMPLE: heating_pwm.set_duty_cycle(6, 40); sets pin 6 to 40% duty cycle

    SyncI2CMaster bus0{0, 4, 5, true};
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

    UART mcu2_uart{};

    bool s = true;
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, s);
        s = !s;

        printf("Cycle");

        mcu2_uart.write("hello");
        // printf("%f %f %f %f %f\n", heating_power_sensors[0].read_bus_voltage(), heating_power_sensors[0].read_current(), heating_power_sensors[1].read_current(), heating_power_sensors[2].read_current(), heating_power_sensors[3].read_current());
        //printf("%f %f %f %f\n", heating_power_sensors[0].read_bus_voltage(), heating_power_sensors[0].read_shunt_voltage(), heating_power_sensors[0].read_current(), (float)heating_power_sensors[0].read_shunt_voltage() / (float)heating_power_sensors[0].read_current());

        // auto temps = sensing();

        // printf("%d,%d,%d,%d,%d,%d,%d\n", time_us_32(), temps[0][0], temps[1][0], temps[2][0], temps[3][0], temps[4][0], temps[5][0]);
        // printf("%d, %d, %d\n", time_us_32(), temps[0][0], temps[0][1]);

        // printf("%d,%d\n", time_us_32(), temps[0]);

        sleep_ms(100);
    }
}
