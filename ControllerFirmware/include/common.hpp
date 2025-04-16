#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include <cmath>

#define ASSERT assert

constexpr bool csv_output = false;
constexpr bool error = true;
constexpr bool info = true && !csv_output;
constexpr bool debug = false && !csv_output;

void enable_heating_power(bool enable);
