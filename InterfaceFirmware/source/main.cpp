#include <common.hpp>

#include "lwip/tcp.h"
#include "lwip/apps/httpd.h"

#include "hw/UART.hpp"

// See https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for intercore interaction
void core1_entry() {
    while(1)
        ;
}

int main()
{
    stdio_init_all();
    // sleep_ms(1000);
    printf("Hello world!\n");

    multicore_launch_core1(core1_entry);

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Enable wifi station
    cyw43_arch_enable_sta_mode();

    UART uart_bus{uart0, 16, 17};
    
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("Your Wi-Fi SSID", "Your Wi-Fi Password", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    // Start lwip http server
    httpd_init();

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
        // gpio_put(PICO_DEFAULT_LED_PIN, s);
        // s = !s;
        printf("Cycle start\n");

        // uart_bus.write("b47#");
        // uart_bus.write("c25452#");
        // uart_bus.write("d25000#");
        uart_bus.write("u26000#");
        // uart_bus.write("e3#");
        // uart_bus.write("w5#");

        sleep_ms(3000);
    }
}
