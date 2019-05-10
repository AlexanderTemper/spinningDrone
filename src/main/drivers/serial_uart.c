#include <stdbool.h>
#include <stdint.h>

#include "drivers/serial_uart.h"
#include "common/utils.h"


uartDevice_t uartDevice;

serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartPort_t *s = serialUART((UARTDevice_e)device,baudRate, mode, options);
    if (!s)
    return (serialPort_t *)s;

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    /*uartReconfigure(s);

    // Receive DMA or IRQ
    if (mode & MODE_RX) {
    }

    // Transmit DMA or IRQ
    if (mode & MODE_TX) {
    }*/
    return (serialPort_t *)s;
}
