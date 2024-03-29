/**
 * @file       main.cpp
 * @author     A. Petrosino - G. Sciddurlo
 * @version    v0.01
 * @date       Nov, 2022
 * @brief
 *
 */

/*================================ include ==================================*/

#include <string>
#include <vector>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <BoardImplementation.hpp>
#include "Callback.hpp"
#include "Scheduler.hpp"
#include "Semaphore.hpp"
#include "Serial.hpp"
#include "Task.hpp"

/*================================ define ===================================*/

#define RADIO_MODE_RX (0)
#define RADIO_MODE_TX (1)
#define RADIO_MODE (RADIO_MODE_RX)
#define RADIO_CHANNEL (26)

#define PAYLOAD_LENGTH (125)
#define EUI48_LENGTH (6)

#define UART_BAUDRATE (115200)
#define SPI_BAUDRATE (8000000)
#define SERIAL_BUFFER_LENGTH (1024)

#define GREEN_LED_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define RADIO_RX_TASK_PRIORITY (tskIDLE_PRIORITY + 0)
#define RADIO_TX_TASK_PRIORITY (tskIDLE_PRIORITY + 0)

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

extern "C" void vApplicationTickHook(void);
extern "C" void vApplicationIdleHook(void);

static void prvGreenLedTask(void *pvParameters);
static void prvRadioRxTask(void *pvParameters);
static void prvRadioTxTask(void *pvParameters);

static void rxInit(void);
static void rxDone(void);
static void txInit(void);
static void txDone(void);

/*=============================== variables =================================*/

static SemaphoreBinary rxSemaphore, txSemaphore;

static PlainCallback rxInitCallback(&rxInit);
static PlainCallback rxDoneCallback(&rxDone);
static PlainCallback txInitCallback(&txInit);
static PlainCallback txDoneCallback(&txDone);

static uint8_t radio_buffer[PAYLOAD_LENGTH];
static uint8_t *radio_ptr = radio_buffer;
static uint8_t radio_len = sizeof(radio_buffer);
static int8_t rssi;
static uint8_t lqi;
static bool crc;

static uint8_t uartBuffer[1024];
static uint8_t serial_buffer[SERIAL_BUFFER_LENGTH];
static uint8_t rssi_buffer[SERIAL_BUFFER_LENGTH];

static Serial serial(uart0);
/*================================= public ==================================*/

int main(void)
{
    // Initialize board
    board.init();

    // Enable the IEEE 802.15.4 radio
    radio.setTxCallbacks(&txInitCallback, &txDoneCallback);
    radio.setRxCallbacks(&rxInitCallback, &rxDoneCallback);
    radio.enable();
    radio.enableInterrupts();
    radio.setChannel(RADIO_CHANNEL);

    // Create the blink task
    xTaskCreate(prvGreenLedTask, (const char *)"Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);

#if (RADIO_MODE == RADIO_MODE_RX)
    // Enable the UART driver
    uart0.enable(UART_BAUDRATE);
    /* Enable the SPI interface */
    spi0.enable(SPI_BAUDRATE);
    /* Initialize Serial interface */
    serial.init();
    /* Initialize the DMA */
    dma.init();

    // Create the radio receive task
    xTaskCreate(prvRadioRxTask, (const char *)"RadioRx", 128, NULL, RADIO_RX_TASK_PRIORITY, NULL);
#elif (RADIO_MODE == RADIO_MODE_TX)
    // Create the radio transmit task
    xTaskCreate(prvRadioTxTask, (const char *)"RadioTx", 128, NULL, RADIO_TX_TASK_PRIORITY, NULL);
#endif
    // Start the scheduler
    Scheduler::run();
}

/*================================ private ==================================*/

static void prvGreenLedTask(void *pvParameters)
{
    // Forever
    while (true)
    {
        // Turn off the green LED and keep it for 950 ms
        led_green.off();
        vTaskDelay(950 / portTICK_RATE_MS);

        // Turn on the green LED and keep it for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void prvRadioRxTask(void *pvParameters)
{
    static RadioResult result;
    size_t len = 0;
    // Forever
    while (true)
    {
        // Turn on the radio transceiver
        radio.on();

        // Put the radio transceiver in receive mode
        radio.receive();

        // Turn the yellow LED on when a the radio is receiving
        led_yellow.on();

        // Take the rxSemaphre, block until available
        if (rxSemaphore.take())
        {
            // Get a packet from the radio buffer
            radio_ptr = radio_buffer;
            radio_len = sizeof(radio_buffer);
            result = radio.getPacket(radio_ptr, &radio_len, &rssi, &lqi, &crc);

            len = sprintf((char *)uartBuffer, "%s\r\n", radio_ptr);
            if (result == RadioResult_Success && crc)
            {
                if (len > 0)
                {                    
                    /* Prepare serial buffer */
                    /* Copy radio packet payload */
                    // https://www.asciitable.it/
                    // uartBuffer[0] = 73;
                    // uartBuffer[len++] = 03;
                    // uartBuffer[len++] = 105;
                    dma.memcpy(&serial_buffer[1], &uartBuffer[0], len);

                    /* Send packet via Serial */
                    serial.write(serial_buffer, len, true);

                    /* Send packet via Serial */                                
                    len = sprintf((char*) rssi_buffer, "RSSI:\t%d\t\r\n", rssi);
                    serial.write(rssi_buffer, len, true);       
                    len = 0;    

                }
            }

            len = 0;
            // Turn the yellow LED off when a packet is received
            led_yellow.off();
            // Turn off the radio until the next packet
            radio.off();
        }
    }
}

static void prvRadioTxTask(void *pvParameters)
{
    static RadioResult result;

    // Get the EUI64 address of the board
    board.getEUI48(radio_buffer);

    // Forever
    while (true)
    {
        // Take the txSemaphre, block until available
        if (txSemaphore.take())
        {
            // Turn on the radio transceiver
            radio.on();

            // Turn the yellow LED on when the packet is being loaded
            led_yellow.on();

            // Load the EUI64 address to the transmit buffer
            radio_ptr = radio_buffer;
            radio_len = EUI48_LENGTH;
            result = radio.loadPacket(radio_ptr, radio_len);

            if (result == RadioResult_Success)
            {
                // Put the radio transceiver in transmit mode
                radio.transmit();

                // Turn the yellow LED off when the packet has beed loaded
                led_yellow.off();
            }

            // Delay the transmission of the next packet 250 ms
            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
}

static void rxInit(void)
{
    // Turn on the radio LED as the radio is now receiving a packet
    led_red.on();
}

static void rxDone(void)
{
    // Turn off the radio LED as the packet is received
    led_red.off();

    // Let the task run once a packet is received
    rxSemaphore.giveFromInterrupt();
}

static void txInit(void)
{
    // Turn on the radio LED as the packet is now transmitting
    led_red.on();
}

static void txDone(void)
{
    // Turn off the radio LED as the packet is transmitted
    led_red.off();

    // Let the task run once a packet is transmitted
    txSemaphore.giveFromInterrupt();
}
