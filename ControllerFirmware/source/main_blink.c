#include <common.hpp>

int main() {
    stdio_init_all();
    printf("Init LED pin.\n");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    bool s = true;
    while (true)
    {
        printf("Test\n");

        gpio_put(PICO_DEFAULT_LED_PIN, s);
        s = !s;
        sleep_ms(1000);
    }
    return 0;
}
