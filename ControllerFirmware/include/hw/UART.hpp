#pragma once

#include <common.hpp>
#include "hardware/uart.h"
#include "hardware/irq.h"

#include <vector>
#include <cctype>
#include <cstdlib>

extern class UART* irq_userptrs[2];

void on_uart0_irq();
void on_uart1_irq();

constexpr int RX_BUFFER_LEN = 32;
class UART
{
public:
    static inline char rx_buffer[RX_BUFFER_LEN] = {};
    static inline UART* irq_userptrs[2] = { nullptr, nullptr };


    UART(uart_inst_t *instance = uart0, uint8_t tx_pin = 0, uint8_t rx_pin = 1, uint baudrate = 115200) : instance{instance} 
    {
        gpio_set_function(tx_pin, UART_FUNCSEL_NUM(instance, tx_pin));
        gpio_set_function(rx_pin, UART_FUNCSEL_NUM(instance, rx_pin));

        uart_init(instance, baudrate);

        setup_interrupts();

        if (debug) printf("UART bus initialised\n");
        uart_puts(instance, "UART bus initialised");
    }

    void tx_error_or_warning(bool error, int idx) {
        auto tx = [&](char cmd, int v) {char buf[16]; snprintf(buf, 16, "%c%d", cmd, v); write(buf); };
        
        if(error) tx('e', idx);
        else tx('w', idx);
    }

    void send_ui_update(int battery_percentage, int current_average_meas_temp, int current_set_temp) {
        auto tx = [&](char cmd, int v) {char buf[16]; snprintf(buf, 16, "%c%d", cmd, v); write(buf); };

        tx('b', battery_percentage);
        tx('c', current_average_meas_temp);
        tx('d', current_set_temp);
    }

    void write(const char *data) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        uart_puts(instance, data);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    void on_uart_rx() {
        while (uart_is_readable(instance))
        {
            char ch = uart_getc(instance);

            if (rx_buffer_idx > RX_BUFFER_LEN) {
                rx_buffer_idx = 0;
                printf("INFO: RX Buffer overflow, resetting\n");
            }

            // Make sure to also change this in the parse_buffer() function
            if ( ch == 'b' || ch == 'c' || ch == 'd' || ch == 'u' || ch == 'e' || ch == 'w') {
                rx_buffer[0] = ch;
                rx_buffer_idx = 1;
            }
            else if ( isdigit(ch) && rx_buffer_idx > 0) {
                rx_buffer[rx_buffer_idx++] = ch;
            }
            else if (ch == '#')
            {
                rx_buffer[rx_buffer_idx] = '\0';
                parse_buffer();
                rx_buffer_idx = 0;
            }
            else {
                rx_buffer_idx = 0;
            }
        }
    }

    void parse_buffer() {
        char key = rx_buffer[0];
        char *p;
        int value = strtol(&rx_buffer[1], &p, 10);

        // b: battery percentage [%]
        // c: current average measured leg temperature [mC]
        // d: controller current set temperature [mC]
        // u: update leg temperature (send new temp) [mC]
        // e: error
        // w: warning
        switch (key) {
            case 'b':
                if (debug) printf("uart_rx: BatPerc: %d\n", value);
                break;
            case 'c':
                if (debug) printf("uart_rx: CurAvgMeasTemp: %d\n", value);
                break;
            case 'd':
                if (debug) printf("uart_rx: CurSetTempControlMCU: %d\n", value);
                break;
            case 'u':
                if (debug) printf("uart_rx: UpdateLegSetTemp: %d\n", value);
                current_set_temp = value;
                break;
            case 'e':
                if (debug) printf("uart_rx: ErrorMsg: %d\n", value);
                break;
            case 'w':
                if (debug) printf("uart_rx: WarningMsg: %d\n", value);
                break;
            default:
                printf("ERR: UART_RX: Unknown: {%c - %d}\n", key, value);
                break;
            }
    }

    int get_current_set_temp() const { return current_set_temp; }

    private:
        uart_inst_t *instance;
        int rx_buffer_idx = 0;
        int current_set_temp = 20000;

        void setup_interrupts() {
            int irq_num = (instance == uart0) ? 0 : 1;
            int irq_enum = (instance == uart0) ? UART0_IRQ : UART1_IRQ;

            irq_userptrs[irq_num] = this;

            uart_set_hw_flow(instance, false, false);
            uart_set_fifo_enabled(instance, false);

            if (irq_num == 0)
                irq_set_exclusive_handler(irq_enum, []() {
                    if(auto* self = irq_userptrs[0]; self) self->on_uart_rx();
                });
            else {
                irq_set_exclusive_handler(irq_enum, []() {
                    if(auto* self = irq_userptrs[1]; self) self->on_uart_rx();
                });
            }

            irq_set_enabled(irq_enum, true);
            uart_set_irqs_enabled(instance, true, false);
        }
};