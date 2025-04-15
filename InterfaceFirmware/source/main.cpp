#include <common.hpp>
#include <iostream>
#include <string.h>
#include <array>
#include "lwip/tcp.h"
#include "lwip/apps/httpd.h"

// Global is more fun
const int max_tag_length = 8;
const int max_value_length = 128;
const int num_tags = 6;
const char* tag_list[num_tags] = {
    "b",            // Current battery percentage
    "c",            // Currently measured temperature
    "d",            // Set temperature according to the controller
    "e",            // Error
    "w",            // Warning
    "n",            // Notice
    // "TIMSTA",       // Status of the timer (on/off)
    // "TIMREM",       // Remaining time of the timer
    // "POWSTA",       // Power status of the heating (on/off)
    // "",          // Remaining time for the battery
};

std::array<char[max_value_length], num_tags> value_list = { // Initialise to example values for testing
    "100",
    "20.0",
    "25.0",
    "",
    "",
    "Connected.",
    // "2h12m",
    // "1"
};

#include "hw/UART.hpp"

// See https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#detailed-description-8 for intercore interaction
void core1_entry() {
    while(1)
        ;
}

bool connect_to_wifi() {
    if (cyw43_arch_wifi_connect_timeout_ms("Your Wi-Fi SSID", "Your Wi-Fi Password", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
            printf("failed to connect.\n");
            return false;
        } else {
            printf("Connected.\n");
            // Read the ip address in a human readable way
            uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
            printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
            return true;
        }
}

u16_t handle_tags(int iIndex, char *pcInsert, int iInsertLen){
    strncpy(pcInsert, value_list[iIndex], max_value_length);
    return strlen(value_list[iIndex]);
}

int main() {
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

    printf("Attempting to establish a Wi-Fi connection...\n");
    int attempts = 10;
    for(int attempt = 0; attempt < attempts; attempt++){
        if (connect_to_wifi()){
            std::cout << "Succeeded on attempt " << attempt << std::endl;
            break;
        }
        else {
            std::cout << "Attempt " << attempt << " failed." << std::endl;
        }
    }

    // Start lwip http server
    httpd_init();

    http_set_ssi_handler(handle_tags, tag_list, num_tags);

    while (true) {
        // gpio_put(PICO_DEFAULT_LED_PIN, s);
        // s = !s;
        printf("Cycle start\n");

        // uart_bus.write("b47#");
        // uart_bus.write("c25452#");
        // uart_bus.write("d25000#");
        uart_bus.write("u26000#");
        // uart_bus.write("e3#");
        // uart_bus.write("w5#");
        sleep_ms(1000);
        // for (int i = 0; i < num_tags; i++){
        // }
    }
}
