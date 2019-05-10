#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_bmf.h"
#include "common/utils.h"

#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"
#include "common/crc.h"

#include "usart_support.h"
#include "debug.h"


const struct serialPortVTable uartVTable[];


static void uartWrite(serialPort_t *instance, uint8_t ch)
{
	uartPort_t *s = (uartPort_t *)instance;
	s->port.txBuffer[s->port.txBufferHead] = ch;

	if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
		s->port.txBufferHead = 0;
	} else {
		s->port.txBufferHead++;
	}

	usart_callback_transmit(&usart_instance);
}


static uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;
    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
           return s->port.rxBufferHead - s->port.rxBufferTail;
       } else {
           return s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
   }
}

static uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
	uartPort_t *s = (uartPort_t*)instance;

	uint32_t bytesUsed;

	if (s->port.txBufferHead >= s->port.txBufferTail) {
		bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
	} else {
		bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
	}


	return (s->port.txBufferSize - 1) - bytesUsed;
}

static uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *s = (uartPort_t *)instance;


    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }

    //printf("Lese:%c int:%d\n",ch,ch);
    return ch;
}
static void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
}
static bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    return true;
}

/*static uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len) {
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}*/

static void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
}
const char *devName;

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uartdev = &uartDevice; ///dev/pts/2"
    uartPort_t *s = &uartdev->port;
    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate;
    s->port.rxBuffer = uartdev->rxBuffer;
    s->port.txBuffer = uartdev->txBuffer;
    s->port.rxBufferSize = ARRAYLEN(uartdev->rxBuffer);
    s->port.txBufferSize = ARRAYLEN(uartdev->txBuffer);
    // NIX ZU TUN DA SCHON WO ANDERS
    return s;
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};

