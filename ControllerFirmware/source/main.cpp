#include <common.hpp>

#include <hw/PWM.hpp>
#include <hw/SyncI2CMaster.hpp>
#include <hw/UART.hpp>
#include <drivers/fram.hpp>
#include <drivers/current_sensor.hpp>
#include <drivers/fram.hpp>
#include <drivers/temp_sensor.hpp>
#include <drivers/heating_element.hpp>
#include <control/PIDController.h>
#include <control/WeightedTemperaturePoints.hpp>

#include <functions/powerdata.hpp>
#include <functions/safetycheck.hpp>

constexpr uint power_cutoff_switch_gpio = 0;
constexpr uint buzzer_gpio = 26;
constexpr uint heatingzone1_gpio= 6;
constexpr uint heatingzone2_gpio = 7;
constexpr uint heatingzone3_gpio = 8;
constexpr uint heatingzone4_gpio = 9;

// See: https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for core interaction
void core1_entry() {
    while (1)
        ;
}

constexpr float DELTA_MILLIKELVIN_MILLICELSIUS = 273150.0;

void enable_heating_power(bool enable) {
    gpio_put(power_cutoff_switch_gpio, enable);
}

int main() {
    stdio_init_all();
    sleep_ms(5000);
    if (info) printf("Controller startup.\n");

    if (debug) printf("Init LED pin.\n");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    if (debug) printf("Init power cutoff switch.\n");
    gpio_init(power_cutoff_switch_gpio);
    gpio_set_dir(power_cutoff_switch_gpio, GPIO_OUT);
    gpio_pull_down(power_cutoff_switch_gpio);
    enable_heating_power(true); // Disable cutoff switch

    if (debug) printf("Init multicore.\n");
    multicore_launch_core1(core1_entry);

    const uint8_t number_of_heating_zones = 4;

    if (debug) printf("Init I2C bus.\n");
    SyncI2CMaster bus0{0, 20, 21, true};

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
        {0.05, 0.34, 0.00, 0.00},
        {0.05, 0.00, 0.45, 0.00},
        {0.00, 0.23, 0.00, 0.45},
        {0.00, 0.00, 0.15, 0.55},
        {0.00, 0.43, 0.40, 0.00}
    }}};

    if (debug) printf("Init heating power sensors.\n");
    std::array<INA219, number_of_heating_zones> heating_power_sensors = {{
        {&bus0, 0x45, 1, 0.101}, // Channel 1
        {&bus0, 0x44, 1, 0.1012}, // Channel 2
        {&bus0, 0x41, 1, 0.1005}, // Channel 3
        {&bus0, 0x40, 1, 0.1006}  // Channel 4
    }};

    if (debug) printf("Init PWM channels heating zones.\n");
    float max_duty_cycle_element1 = 0.5;
    float max_duty_cycle_element2 = 0.5;
    float max_duty_cycle_element3 = 0.7;
    float max_duty_cycle_element4 = 0.45;
    std::array<PWM, number_of_heating_zones> heating_pwm_channels = {{
        {heatingzone1_gpio, max_duty_cycle_element1},
        {heatingzone2_gpio, max_duty_cycle_element2},
        {heatingzone3_gpio, max_duty_cycle_element3},
        {heatingzone4_gpio, max_duty_cycle_element4}
    }};
    for (int i = 0; i < 4; i++) {
        heating_pwm_channels[i].enable();
    }

    if (debug) printf("Init PWM for buzzer.\n");
    PWM buzzer = {buzzer_gpio, 0.5};
    buzzer.enable();
    buzzer.set_duty_cycle_safe(0.5);
    sleep_ms(100);
    buzzer.set_duty_cycle_safe(0.0);
    sleep_ms(100);
    buzzer.set_duty_cycle_safe(0.5);
    sleep_ms(100);
    buzzer.set_duty_cycle_safe(0.0);
    if (info) printf("Init battery voltage processing.\n");
    Battery bat;
    // bat.startup()

    if (info) printf("Init safety controller.\n");
    // SafetyControl safety{*uart_bus, buzzer};
    // safety.check_startup(bool i2c_status_power, bool i2c_status_sensorsheating, bool uart_status_ui);

    if (info) printf("Init heating zones (PWM + sensors).\n");
    float calibration_duty_cycle = 0.3;
    float max_power_mw_element1 = 7000.0;
    float max_power_mw_element2 = 11000.0;
    float max_power_mw_element3 = 20000.0;
    float max_power_mw_element4 = 13000.0;
    std::array<HeatingElement, number_of_heating_zones> heating_elements = {{
        {0, &heating_pwm_channels[0], &heating_power_sensors[0], max_power_mw_element1, calibration_duty_cycle},
        {1, &heating_pwm_channels[1], &heating_power_sensors[1], max_power_mw_element2, calibration_duty_cycle},
        {2, &heating_pwm_channels[2], &heating_power_sensors[2], max_power_mw_element3, calibration_duty_cycle},
        {3, &heating_pwm_channels[3], &heating_power_sensors[3], max_power_mw_element4, calibration_duty_cycle}
    }};
    
    if (info) printf("Init PID controller.\n");
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
    
    if (info) printf("First calibration\n");
    if (csv_output) printf("FCAL: ");
    for (int i = 0; i < number_of_heating_zones; i++){
        float new_impedance = heating_elements[i].calibrate_impedance();
        if (info) printf("%.2fohm\t", new_impedance);
        if (csv_output) printf("%.2f,", new_impedance);
    }
    if (csv_output) printf("\n");
    if (info) printf("\n\n");

    double setpoint_mc = 30000; // 30k mCº
    int calibration_cycle_counter = 0;
    int calibration_after_cycles = 10;
    int cycle_duration_ms = 1000;

    if (info) printf("Starting main loop\n");
    while (true)
    {
        if (info) printf("New cycle: ");
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        
        //bat.update_soc(float powerdata_voltage, float powerdata_current); // Every few seconds: Update battery SoC estimate, needs some I2C magic measured voltage [mV] and current [mA]
        //bat.estimate_life(float pwr_usage_now);                         // Every few seconds: Update estimated battery hours left, needs estimated power usage from feedback model

        if (calibration_cycle_counter >= calibration_after_cycles) {
            if (info) printf("Calibrating\n");
            // "CAL: imp1,imp2,imp3,imp4\n" 
            if (csv_output) printf("CAL: ");
            for (int i = 0; i < number_of_heating_zones; i++){
                float new_impedance = heating_elements[i].calibrate_impedance();
                if (info) printf("%.2fohm\t", new_impedance);
                if (csv_output) printf("%.2f,", new_impedance);
            }
            if (info || csv_output) printf("\n");
            calibration_cycle_counter = 0;
        }
        calibration_cycle_counter++;

        // "NORM: despow1,actpow1,zonetemp1,despow2,actpow2,zonetemp2,despow3,actpow3,zonetemp3,despow4,actpow4,zonetemp4\n"
        if (csv_output) printf("NORM: ");
        std::array<float, number_of_heating_zones> zone_temperatures_data_mc = zone_temperatures.get_temperatures();

        //!!!!!!!UNCOMMENT WHEN UI IS DONE!!!!!!!!!!!!!
        /*setpoint_mc = uart_bus.get_current_set_temp();

        {
            float avg = 0;
            for(auto temp : zone_temperatures_data_mc)
                avg += temp;
            avg /= number_of_heating_zones;

            uart_bus.send_ui_update(bat.soc, avg, setpoint_mc);
        }*/
       //!!!!!! END OF UNCOMMENT WHEN UI IS DONE!!!!!!!!
        

        // Every few seconds: Checks for safety concerns, uses basically all data available
        // Every few seconds: Check for safety concerns
        //safety.check_safety(bool i2c_power, float powerdata_current, float powerdata_voltage, bool i2c_sensorsheating, const std::array<std::array<int32_t, 2>, 6>& temps, const std::array<float, 4>& currents, float pwr_usage_now, const std::array<float, 4>& pwm_heating, int bat_soc, bool uart_ui);

        if (debug) {
            std::array<std::array<int32_t, 2>, 6> raw_temps = sensing();
            for (int j = 0; j < 6; j++) {
                for (int k = 0; k < 2; k++) {
                    printf("%.2f\t", static_cast<float>(raw_temps[j][k])*0.001);
                }
            }
            printf("\n");
        }

        for (int i = 0; i < number_of_heating_zones; i++)
        {
            double desired_power_mw = 1000 * pidZones[i].update(
                                                     0.001 * (DELTA_MILLIKELVIN_MILLICELSIUS + setpoint_mc),
                                                     0.001 * (DELTA_MILLIKELVIN_MILLICELSIUS + zone_temperatures_data_mc[i]));
            desired_power_mw = 5000;
            heating_elements[i].set_power_safe(desired_power_mw);
            sleep_ms(200);
            float actual_power = heating_elements[i].get_current_power();
            if (csv_output) printf("%.3f,%.3f,%.3f,", desired_power_mw, actual_power, zone_temperatures_data_mc[i]);
            if (info) printf("%.0fmW[desired:%.0fmW][zone_temp:%.0f]\t", actual_power, desired_power_mw, zone_temperatures_data_mc[i]);
        }

        if (info || csv_output) printf("\n");
        sleep_ms(cycle_duration_ms);
    }
}
