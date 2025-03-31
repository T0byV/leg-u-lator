#include <common.hpp>

#include <i2c/SyncI2CMaster.hpp>

int main() {
    stdio_init_all();

    SyncI2CMaster bus0{0, 4, 5, true};

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
