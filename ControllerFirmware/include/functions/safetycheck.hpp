/*Safety Controller Checklist (all warnings, severe if stated):
Startup
    Handle bus+sensor / SPI initialization failures: severe
Checking battery
    Track response time
    Track delta between readings of voltage and current
    Compare current with sent PWM signals previously: severe
    Sudden drop in current could be blown fuse: severe
    Sanity check the read values of voltage and current
    Compare (roughly) battery output power with heating power
    Alerting low battery life remaining
Checking UI
    Check SPI / webapp communication
Checking temperature sensor data
    Track response time
    Sanity check the read values
    Track delta between readings: severe
    Compare delta with previous PWM signals?
Checking heating behaviour (current sensors)
    Track response time
    Track delta between readings: severe
    Sanity check the read values
    Compare with previous sent PWM’s

Implementation:
Function or class every loop, basically a checklist
Probably gonna be object-oriented way of retrieving the data, global is possible but not nice
All checks, with sending a warning(... some text…) to UI or the severe(...with some text…) to UI / battery.

WIP!
- Might need some work before using
- Correctly get all the data
- Have a second thought about the implemented bounds for throwing errors
*/

#pragma once

#include <common.hpp>

#include <cmath>        // for using floor and stuff
#include <array>        // for using arrays

class SafetyControl {
    public:
        const char* ui_msg;         // String message sent to UI
        bool severe_error = false;          // Flag for severe error, which should disable PWM + Cut heating activation line + Ring buzzer
        int cnt;                    // #, counter for tracking history

        // Checks for issues on startup, needs the status of the 2 I2C lines and the UART line as a boolean
        void check_startup(bool i2c_status_power, bool i2c_status_sensorsheating, bool uart_status_ui){
            cnt = 0;                // set counter to 0 
            
            // Startup/connection errors
            if (i2c_status_power == false) {
                ui_msg = "No battery data connection";
                severe_error = true;
            }
            if (i2c_status_sensorsheating == false) {
                ui_msg = "No sensor and/or heater data connection";
                severe_error = true;
            }
            if (uart_status_ui == false) {
                ui_msg = "No UI connection";
                severe_error = true;
            }
        }

        // Continously monitors device for safety issues
        void check_safety(bool i2c_power, float powerdata_current, float powerdata_voltage, bool i2c_sensorsheating, const std::array<std::array<int32_t, 2>, 6>& temps, const std::array<float, 4>& currents, float pwr_usage_now, const std::array<float, 4>& pwm_heating, int bat_soc, bool uart_ui) {
            /* Variables coming in:
                    bool i2c_power;                   // status of power I2C line
                    float powerdata_current;          // mA, battery current measured over I2C line
                    float powerdata_voltage;          // mV, battery voltage measured over I2C line

                    bool i2c_sensorsheating;                                // status of sensors and heating I2C line
                    const std::array<std::array<int32_t, 2>, 6>& temps;     // pointer array [degrees C?], data of the 6 sensing clusters of sensing, each cluster having 2 sensor values
                    const std::array<float, 4>& currents[4];                // pointer array [mA], data of the 4 current sensors of heating  
                    
                    float pwr_usage_now;                        // mW, total power usage for heating from model
                    const std::array<float, 4>& pwm_heating;    // array with [ratio], PWM signals to heating from model
                    int bat_soc;                                // %, battery SOC as determined from power data 

                    bool uart_ui;                   // status of UART connection to UI
            */

            // constants
            constexpr int tracking_size = 2;      // number of data values to be tracked

            // CONNECTION ERRORS
            if (i2c_power == false) {
                ui_msg = "Lost battery data connection";
                severe_error = true;
            }
            if (i2c_sensorsheating == false) {
                ui_msg = "Lost sensor and/or heater data connection";
                severe_error = true;
            }
            if (uart_ui == false) {
                ui_msg = "Lost UI connection";
                severe_error = true;
            }

            // CHECKING BATTERY
            // Tracking battery data
            std::array<float, tracking_size> track_power_voltage{}; //mV, tracking history of battery voltage
            std::array<float, tracking_size> track_power_current{}; //mA, tracking history of battery current
            track_power_voltage[cnt] = powerdata_voltage; // put current data in history array
            track_power_current[cnt] = powerdata_current; // put current data in history array
            for (int i = 0; i<tracking_size; i++) {     // go through every history array element of voltage
                float value = track_power_voltage[i];
                // Check absence of reading / connection / response time (NaN/inf)
                if (isnan(value) || isinf(value)) {
                    ui_msg = "No battery voltage data";
                }
                else {
                    // Sanity check values
                    if (value < 13600 || value > 16800) {
                        ui_msg = "Invalid battery voltage data"; 
                    }
                    else if (i+1<tracking_size) {
                        // Check delta between readings (with next in history array, not for last element)
                        if (fabs(value-track_power_voltage[i+1]) > 1000) {
                            ui_msg = "Sudden battery voltage difference";
                            severe_error = true; 
                        }
                    }   
                }  
            }
            for (int i = 0; i<tracking_size; i++) {     // go through every history array element of current
                float value = track_power_current[i];
                // Check absence of reading / connection / response time (NaN/inf)
                if (isnan(value) || isinf(value)) {
                    ui_msg = "No battery current data";
                }
                else {
                    // Sanity check values
                    if (value < 0 || value > 8000) {
                        ui_msg = "Invalid battery current data"; 
                    }
                    else if (i+1<tracking_size) {
                        // Check delta between readings (with next in history array, not for last element)
                        if (fabs(value-track_power_current[i+1]) > 4000) {
                            ui_msg = "Sudden battery current difference";
                            severe_error = true; 
                        }
                    }   
                }  
            }
            // Compare battery output power with model heating power
            if (fabs(pwr_usage_now - (powerdata_current*powerdata_voltage/1000)) >= 5000) {   // difference in mW
                ui_msg = "Different power consumption than expected";
            }
            // Compare battery current with current sensors of heating
            float heating_current = currents[0]+currents[1]+currents[2]+currents[3];    // mA, total current measured by heating
            if (abs(powerdata_current - heating_current) >= 500) {      // difference in mA
                ui_msg = "Power leakage";
                severe_error = true;
            }
            // Low battery life
            if (bat_soc <= 25) {
                ui_msg = "Low battery";
            }
            if (bat_soc <= 10) {
                ui_msg = "Battery empty";
                severe_error = true;
            }

            // CHECKING SENSORS
            // Tracking sensor data
            std::array<std::array<std::array<int32_t, 2>, 6>, tracking_size> track_temps{};     //degrees C, tracking history of temperature sensors (3D: tracking 6 clusters, each containing 2 sensors)
            for (int c = 0; c < 6; c++) {           // do for every single cluster
                for (int s = 0; s < 2; s++) {       // do for every sensor of cluster
                    track_temps[cnt][c][s] = temps[s];      // put every sensor data in history array
                    for (int i = 0; i<tracking_size; i++) {     // go through every history array element of sensors
                        int32_t value = track_temps[i][c][s];
                        // Check absence of reading / connection / response time (NaN/inf)
                        if (isnan(value) || isinf(value)) {
                            ui_msg = "No temperature sensor data";
                        }
                        else {
                            // Sanity check values
                            if (value < 0 || value > 50) {
                                ui_msg = "Invalid temperature sensor data"; 
                            }
                            else if (value < 20) {
                                ui_msg = "Cold warning"; 
                            }
                            else if (value > 40) {
                                ui_msg = "Heat warning"; 
                                severe_error = true;
                            }
                            else if (i+1<tracking_size) {
                                // Check delta between readings (with next in history array, not for last element)
                                if (fabs(value-track_temps[i+1][c][s]) > 2) {
                                    ui_msg = "Sudden temperature sensor difference";
                                    severe_error = true; 
                                }
                            }   
                        }  
                    }
                }
            }

            // CHECKING HEATING BEHAVIOUR
            // Tracking heating current sensors
            std::array<std::array<float, 4>, tracking_size> track_currents{};   //mA, tracking history of heating element currents (2D array with history of 4 sensors)
            
            for (int s = 0; s < 4; s++) {       // do for every single sensor
                track_currents[cnt][s] = currents[s]; // put every sensor data in history array
                for (int i = 0; i<tracking_size; i++) {     // go through every history array element of sensors
                    float value = track_currents[i][s];
                    // Check absence of reading / connection / response time (NaN/inf)
                    if (isnan(value) || isinf(value)) {
                        ui_msg = "No heating current sensor data";
                    }
                    else {
                        // Sanity check values
                        if (value < 0 || value > 2000) {
                            ui_msg = "Invalid heating current sensor data"; 
                        } 
                    }  
                }
                // Compare current with sent PWM signals
                float heating_pwm_current = 10 * pwm_heating[s];    // WIP! mA, converting PWM to A of a single PWM channel, magic?
                if (fabs(track_currents[cnt][s]-heating_pwm_current) > 500) {
                    ui_msg = "(conversion is WIP!) Heating element current misaligns with sent PWM";
                    severe_error = true; 
                }
            }

            // Update tracker
            cnt++;                                   // increment history counter
            if (cnt == tracking_size) { cnt=0; };    // loop counter back to 0 if the history array is full
        }

        // Raises alarm when a severe error is processed, needs the boolean for severe error
        void alarm(bool severe_error) {
            if (severe_error == true) {
                // Disable PWM
                // Cut heating activation line
                // Ring buzzer
            }
        }
};
