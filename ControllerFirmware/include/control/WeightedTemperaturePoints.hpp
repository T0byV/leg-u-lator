#pragma once

#include <common.hpp>
#include <drivers/temp_sensor.hpp>

template<int SensN, int SensM, int WeightN, int WeightM>
class WeightedTemperaturePoints {
    public:
        WeightedTemperaturePoints(Sensor<SensN,SensM> *sensors, const std::array<std::array<float, WeightM>, WeightN>& weight_matrix) : sensors{sensors}, weight_matrix{weight_matrix} {}

        std::array<float, WeightM> get_temperatures() {
            std::array<float, WeightM> weighted_sensor_data = {};
            std::array<std::array<int32_t, SensM>, SensN> sensor_data = (*sensors)();

            for (int i = 0; i < WeightM; i++)
            {
                float avg_zone_temperature = 0;
                for (int j = 0; j < WeightN; j++) {
                    float avg_cluster_temperature = 0;
                    for (int k = 0; k < SensM; k++)
                        avg_cluster_temperature += static_cast<float>(sensor_data[j][k]);
    
                    avg_zone_temperature += weight_matrix[j][i] * (avg_cluster_temperature / SensM);
                }
                weighted_sensor_data[i] = avg_zone_temperature / WeightN;
            }

            return weighted_sensor_data;
        }

    private:
        Sensor<SensN,SensM> *sensors;
        std::array<std::array<float, WeightM>, WeightN> weight_matrix;
};