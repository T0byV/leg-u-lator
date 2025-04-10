/* Battery data processing
    (I2C controller)
    Power data processing -> battery current & voltage
    Determine + store battery voltage and percentage
    Estimate battery life
*/

#pragma once

#include <common.hpp>

#include <map>    // for mapping battery voltage to SOC
#include <cmath>    // for using floor and stuff
using namespace std;    // get rid of the std:: shit

class Battery {
    public:
        int soc;                // % of battery SoC remaining
        int voltage;            // mV, voltage of battery
        int life;               // h, remaining battery life
        uint64_t last_update;   // us, time since last soc update
        int cnt;                // #, counter for tracking history

        // Initializes all attributes, estimates SoC [%] using Voltage [mV] from battery
        void startup(int powerdata_voltage){
            // initialize most attributes
            voltage = powerdata_voltage;    // save measured voltage to battery attribute
            life = 0;                       // initialize battery life to 0h
            last_update = time_us_64();     // sets battery startup time
            cnt = 0;                        // Initialize tracking counter

            // Lookup table voltage[mV]-soc[%] of LiPo 4S battery
            map<int, int> VtoSOC = {
                {0, 0},
                {136, 1},
                {137, 7},
                {138, 10},
                {139, 12},
                {140, 13},
                {141, 14},
                {142, 15},
                {143, 17},
                {144, 18},
                {145, 19},
                {146, 20},
                {147, 23},
                {148, 25},
                {149, 27},
                {150, 32},
                {151, 36},
                {152, 40},
                {153, 47},
                {154, 64},
                {155, 68},
                {156, 73},
                {157, 77},
                {158, 83},
                {159, 85},
                {160, 87},
                {161, 89},
                {162, 91},
                {163, 93},
                {164, 94},
                {165, 96},
                {166, 98},
                {167, 99},
                {168, 100}
            };    

            // map voltage to SoC
            if (powerdata_voltage >= 13600 && powerdata_voltage <= 16800) {
                int v_lookup = round(powerdata_voltage/100);    // round measured voltage to voltage with .1V accuravy
                soc = VtoSOC[v_lookup]; // %, estimated battery percentage, based on lookup table
            }
            else {
                soc = 100;    // try to estimate battery percentage, if this fails default to full battery
            }
        }

        // Updates estimated SoC [%] using coulomb counting, needs measured voltage [mV] and current [mA] from battery
        void update_soc(int powerdata_voltage, int powerdata_current){
            voltage = powerdata_voltage;    // save measured voltage to battery attribute
            const int capacity = 5000;      // mAh, capacity of the used 4S LiPo battery

            // calculate time delta between SoC updates
            uint64_t current_time = time_us_64();   // us, current time
            float delta_time = (current_time - last_update) / 3600000000; // h, time delta since last soc estimate, [h] = ([us]-[us]) / 10^6 / 3600 
            last_update = current_time;    //us, update last update time to this one

            // subtract used energy from SoC estimate
            soc = soc - ceil((powerdata_current * delta_time) / capacity); // %, updated soc estimate, [%] = [%] - ([mA]*[h])/[mAh]
            if (soc < 0) { soc = 0;}        // limit soc to 0 and 100%
            if (soc > 100) { soc = 100;}    // limit soc to 0 and 100%

            // ! necessary to store voltage/soc in fram? Thomas magic?
        }

        // Updates estimated remaining battery life [h], needs current expected power usage [mW] magic from control and uses stored voltage
        void estimate_life(int pwr_usage_now){
            const int tracking_size = 10;               // number of data values to be tracked

            // track power history
            int pwr_usage[tracking_size] = { 0 };       // mW, tracking history array of estimated power usage by heating from control loop
            pwr_usage[cnt] = pwr_usage_now;             // put current power usage in history array
            cnt++;                                      // increment history counter
            if (cnt == tracking_size) { cnt = 0; };     // loop counter back to 0 if the history array is full

            // calculate average power usage
            int pwr_sum = 0;                            // mW, init sum of power values
            for (int i=0; i < tracking_size; i++) {     // loop through every power data value
                pwr_sum += pwr_usage[i];                // add each value of the power to the sum
            }
            int avg_pwr_usage = round(pwr_sum / tracking_size);  // mW, average of the tracked power plan

            // estimate battery life 
            int mah_left = floor(soc*capacity/100);                 // mAh left in battery, [%/100*[mAh]]
            float mah_perh = ceil(avg_pwr_usage / voltage) * 1000;  // mAh/h consuming, [mAh/h] = [mA] = [mW] / [mV] * 1000
            life = floor(mah_left / mah_perh);                      // battery life in [h] = [mAh] / [mAh/h]
        }
};