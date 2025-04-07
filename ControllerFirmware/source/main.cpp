#include <common.hpp>

#include <i2c/SyncI2CMaster.hpp>
#include <drivers/fram.hpp>

// See: https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for core interaction
void core1_entry() {
    while (1)
        ;
}

int main() {
    stdio_init_all();

    multicore_launch_core1(core1_entry);

    SyncI2CMaster bus0{0, 4, 5, true};

    MB85RC64TA fram{bus0, 0x65};

    fram.write<uint32_t>(4, 324324);

    auto v = fram.read<uint32_t>(4);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
