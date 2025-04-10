#pragma once

#include <common.hpp>
#include "hardware/uart.h"

extern class UART* irq_userptrs[2];

void on_uart0_irq();
void on_uart1_irq();

class UART {
    public:
        UART(uint baudrate = 115200, uart_inst_t *instance = uart0, uint8_t tx_pin = 0, uint8_t rx_pin = 1) : instance{instance} {
            gpio_set_function(0, UART_FUNCSEL_NUM(instance, tx_pin));
            gpio_set_function(1, UART_FUNCSEL_NUM(instance, rx_pin));

            uart_init(instance, baudrate);

            setup_interrupts();

            printf("UART bus initialised");
            uart_puts(instance, "UART bus initialised");
        }

        void write(const char *data) {
            uart_puts(instance, data);
        }
    
        void on_uart_rx() {
            while (uart_is_readable(instance)) {
                char ch = uart_getc(instance);
                printf("%c", ch);
            }
        }

    private:
        uart_inst_t *instance;

        void setup_interrupts() {
            int irq_num = (instance == uart0) ? 0 : 1;
            int irq_enum = instance == uart0 ? UART0_IRQ : UART1_IRQ;

            irq_userptrs[irq_num] = this;

            if (irq_num == 0)
                irq_set_exclusive_handler(irq_enum, on_uart0_irq);
            else
                irq_set_exclusive_handler(irq_enum, on_uart1_irq);

            irq_set_enabled(irq_enum, true);
            uart_set_irqs_enabled(instance, true, false);
        }
};