#include <common.hpp>

#include <hw/PWM.hpp>
#include <hw/SyncI2CMaster.hpp>
#include <hw/UART.hpp>
#include <drivers/fram.hpp>
#include <drivers/current_sensor.hpp>
//#include <drivers/fram.hpp>
#include <drivers/temp_sensor.hpp>
#include <drivers/heating_element.hpp>
#include <control/PIDController.h>
#include <control/WeightedTemperaturePoints.hpp>

#include <functions/powerdata.hpp>
#include <functions/safetycheck.hpp>

// See: https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for core interaction
void core1_entry() {
    while (1)
        ;
}

constexpr float DELTA_MILLIKELVIN_MILLICELSIUS = 273150.0;

constexpr bool info = true;
constexpr bool debug = false;

int main() {
    stdio_init_all();
    sleep_ms(5000);
    if (info) printf("Controller startup.\n");

    if (debug) printf("Init LED pin.\n");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    if (debug) printf("Init multicore.\n");
    multicore_launch_core1(core1_entry);

    const uint8_t number_of_heating_zones = 4;

    if (debug) printf("Init I2C bus.\n");
    SyncI2CMaster bus0{0, 20, 21, false};

    if (debug) printf("Init UART bus.\n");
    UART uart_bus{uart1, 4, 5};

    if (info) printf("Init temperature sensors.\n");
    Sensor<6, 2> sensing{&bus0, {{
        {0x50, 0x58}, // 1 Red
        {0x51, 0x59}, // 2 Orange
        {0x52, 0x5A}, // 3 Yellow
        {0x53, 0x5B}, // 4 Green
        {0x54, 0x5C}, // 5 Blue
        {0x55, 0x5D} // 6 Purple
    }}};

    if (debug) printf("Setup zone temperature conversion.\n");
    WeightedTemperaturePoints<6, 2, 6, 4> zone_temperatures{&sensing, {{
        {0.90, 0.00, 0.00, 0.00},
        {0.50, 0.34, 0.00, 0.00},
        {0.50, 0.00, 0.45, 0.00},
        {0.00, 0.23, 0.00, 0.45},
        {0.00, 0.00, 0.15, 0.55},
        {0.00, 0.43, 0.40, 0.00}
    }}};

    if (info) printf("Init heating power sensors.\n");
    std::array<INA219, number_of_heating_zones> heating_power_sensors = {{
        {&bus0, 0x45, 1, 0.101}, // Channel 1
        {&bus0, 0x44, 1, 0.1012}, // Channel 2
        {&bus0, 0x41, 1, 0.1005}, // Channel 3
        {&bus0, 0x40, 1, 0.1006}  // Channel 4
    }};
  
    if (info) printf("Init heating power sensors.\n");
    Battery bat;
    // bat.startup()

    SafetyControl safety;


    float max_duty_cycle = 0.2;
    std::array<PWM, number_of_heating_zones> heating_pwm_channels = {{
        {6, max_duty_cycle},
        {7, max_duty_cycle},
        {8, max_duty_cycle},
        {9, max_duty_cycle}
    }};

    float calibration_duty_cycle = 0.3;
    float max_power_mw_per_element = 2000;
    std::array<HeatingElement, number_of_heating_zones> heating_elements = {{
        {0, &heating_pwm_channels[0], &heating_power_sensors[0], max_power_mw_per_element, calibration_duty_cycle},
        {1, &heating_pwm_channels[1], &heating_power_sensors[1], max_power_mw_per_element, calibration_duty_cycle},
        {2, &heating_pwm_channels[2], &heating_power_sensors[2], max_power_mw_per_element, calibration_duty_cycle},
        {3, &heating_pwm_channels[3], &heating_power_sensors[3], max_power_mw_per_element, calibration_duty_cycle}
    }};
    
    // PID parameters :
    double Kp = 5.0, Ki = 0.0005, Kd = 45.0; 
    double deadband = 0.5, baseline = 0.0, sampleTime = 1.0; 
    double alphaTemp = 0.9, alphaDeriv = 0.5, maxPower = 40.0;   
    
    PIDController pidZones[number_of_heating_zones] = {
        PIDController(Kp, Ki, Kd, deadband, baseline, sampleTime, alphaTemp, alphaDeriv, maxPower),
        PIDController(Kp, Ki, Kd, deadband, baseline, sampleTime, alphaTemp, alphaDeriv, maxPower),
        PIDController(Kp, Ki, Kd, deadband, baseline, sampleTime, alphaTemp, alphaDeriv, maxPower),
        PIDController(Kp, Ki, Kd, deadband, baseline, sampleTime, alphaTemp, alphaDeriv, maxPower)
    };
    
    // bool s = false;

    printf("First calibration\n");
    for (int i = 0; i < number_of_heating_zones; i++)
    {
        printf("%.2fohm\t", heating_elements[i].calibrate_impedance());
    }
    printf("\n");


    
    double setpoint_mc = 30000; // 30k mCº
    // float desired_power_mw = 150;
    int calibration_cycle_counter = 0;
    int calibration_after_cycles = 10;
    int cycle_duration_ms = 1000;

    bool s = true;
    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, s);
        s = !s;
        sleep_ms(1000);
    }
    return 0;
    while (true)
    {
        // gpio_put(PICO_DEFAULT_LED_PIN, s);
        // s = !s;
            //bat.update_soc(float powerdata_voltage, float powerdata_current); // Every few seconds: Update battery SoC estimate, needs some I2C magic measured voltage [mV] and current [mA]
            //bat.estimate_life(float pwr_usage_now);                         // Every few seconds: Update estimated battery hours left, needs estimated power usage from feedback model
        printf("Cycle start\n");

        if (calibration_cycle_counter >= calibration_after_cycles) {
            printf("Calibrating\n");
            for (int i = 0; i < number_of_heating_zones; i++){
                printf("%.2fohm\t", heating_elements[i].calibrate_impedance());
            }
            printf("\n");
            calibration_cycle_counter = 0;
        }
        calibration_cycle_counter++;

        std::array<float, number_of_heating_zones> zone_temperatures_data_mc = zone_temperatures.get_temperatures();

        for (int i = 0; i < number_of_heating_zones; i++)
        {
            double desired_power_mw = 1000 * pidZones[i].update(
                0.001 * (DELTA_MILLIKELVIN_MILLICELSIUS + setpoint_mc),
                0.001 * (DELTA_MILLIKELVIN_MILLICELSIUS + zone_temperatures_data_mc[i])
            );
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
