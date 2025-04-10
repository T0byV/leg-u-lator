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
#include <string>       // for using strings
using namespace std;    // get rid of the std:: shit

class SafetyControl {
    public:
        string ui_msg;              // String message sent to UI
        bool severe_error;          // Flag for severe error, which should disable PWM + Cut heating activation line + Ring buzzer
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
        void check_safety(bool i2c_power, int powerdata_current, int powerdata_voltage, bool i2c_sensorsheating, auto temps, auto currents, int pwr_usage_now, auto pwm_heating, int bat_soc, bool uart_ui) {
            /* Variables coming in:
                    bool i2c_power;                 // status of power I2C line
                    int powerdata_current;          // mA, battery current measured over I2C line
                    int powerdata_voltage;          // mV, battery voltage measured over I2C line

                    bool i2c_sensorsheating;        // status of sensors and heating I2C line
                    float temps[6];                 // arrary [degrees C?], data of the 6 sensing clusters of sensing
                    int currents[4];                // array [mA], data of the 4 current sensors of heating  
                    
                    int pwr_usage_now;              // mW, total power usage for heating from model
                    float pwm_heating[4];           // array with [ratio], PWM signals to heating from model
                    int bat_soc;                    // %, battery SOC as determined from power data 

                    bool uart_ui;                   // status of UART connection to UI
            */

            // constants
            const int tracking_size = 2;      // number of data values to be tracked

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
            int track_power_voltage[tracking_size] = { 0 }; //mV, tracking history of battery voltage
            int track_power_current[tracking_size] = { 0 }; //mA, tracking history of battery current
            track_power_voltage[cnt] = powerdata_voltage; // put current data in history array
            track_power_current[cnt] = powerdata_current; // put current data in history array
            for (int i = 0; i<tracking_size; i++) {     // go through every history array element of voltage
                int value = track_power_voltage[i];
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
                int value = track_power_current[i];
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
            int heating_current = currents[0]+currents[1]+currents[2]+currents[3];    // mA, total current measured by heating
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
            float track_temps[6][tracking_size] = { 0.0 }; //degrees C, tracking history of temperature sensors (row for each sensor)
            for (int s = 0; s < 6; s++) {       // do for every single sensor
                track_temps[s][cnt] = temps[s]; // put every sensor data in history array
                for (int i = 0; i<tracking_size; i++) {     // go through every history array element of sensors
                    float value = track_temps[s][i];
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
                            if (fabs(value-track_temps[s][i+1]) > 2) {
                                ui_msg = "Sudden temperature sensor difference";
                                severe_error = true; 
                            }
                        }   
                    }  
                }
            }

            // CHECKING HEATING BEHAVIOUR
            // Tracking heating current sensors
            int track_currents[4][tracking_size] = { 0 }; //mA, tracking history of heating element currents (row for each sensor)
            for (int s = 0; s < 4; s++) {       // do for every single sensor
                track_currents[s][cnt] = currents[s]; // put every sensor data in history array
                for (int i = 0; i<tracking_size; i++) {     // go through every history array element of sensors
                    int value = track_currents[s][i];
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
                int heating_current = 10 * pwm_heating[s];    // WIP! mA, converting PWM to A of a single PWM channel, magic?
                if (fabs(track_currents[s][cnt]-heating_current) > 500) {
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