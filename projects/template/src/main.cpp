/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include <task.h>
#include "semphr.h"

#include "Gpio.hpp"
#include "I2c.hpp"
#include "Spi.hpp"


#include <OneWire.hpp>
#include <platform_types.hpp>

#include <BoardImplementation.hpp>

#include <Callback.hpp>
#include <Scheduler.hpp>
#include <Semaphore.hpp>
#include <Serial.hpp>
#include <Task.hpp>

#include "bme280/Bme280.hpp"

/*================================ define ===================================*/

#define RADIO_MODE_RX                       ( 0 )
#define RADIO_MODE_TX                       ( 1 )
#define RADIO_MODE                          ( RADIO_MODE_TX )
#define RADIO_CHANNEL                       ( 26 )

#define PAYLOAD_LENGTH                      ( 125 )
#define EUI48_LENGTH                        ( 6 )

#define UART_BAUDRATE                       ( 115200 )

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 2 )
#define RADIO_RX_TASK_PRIORITY              ( tskIDLE_PRIORITY + 0 )
#define RADIO_TX_TASK_PRIORITY              ( tskIDLE_PRIORITY + 0 )


#define SENSORS_CTRL_PORT           		( GPIO_A_BASE )
#define SENSORS_CTRL_PIN            		( GPIO_PIN_7 )
#define BME280_I2C_ADDRESS          		( BME280_I2C_ADDR_PRIM )

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
static uint8_t* radio_ptr = radio_buffer;
static uint8_t  radio_len = sizeof(radio_buffer);
static int8_t rssi;
static uint8_t lqi;
//static uint8_t crc;
static bool crc;


static GpioConfig sensors_pwr_cfg = {SENSORS_CTRL_PORT, SENSORS_CTRL_PIN, 0, 0, 0};
static GpioOut sensors_pwr_ctrl(sensors_pwr_cfg);
static Serial serial(uart0);
static Bme280 bme280(i2c, BME280_I2C_ADDRESS);

static uint8_t uartBuffer[500];

typedef struct {
  uint16_t temperature;
  uint16_t humidity;
  uint16_t pressure;
} SensorData;

/*================================= public ==================================*/

int main (void)
{
    // Initialize board
    board.init();

    // Enable the IEEE 802.15.4 radio
    radio.setTxCallbacks(&txInitCallback, &txDoneCallback);
    radio.setRxCallbacks(&rxInitCallback, &rxDoneCallback);
    radio.enable();
    radio.enableInterrupts();
    radio.setChannel(RADIO_CHANNEL);
	
	
	/* Enable UART */
    uart0.enable(UART_BAUDRATE);
	
	led_orange.on();

    // Create the blink task
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);

#if (RADIO_MODE == RADIO_MODE_RX)
    // Enable the UART driver
    //uart0.enable(UART_BAUDRATE);

    // Create the radio receive task
    xTaskCreate(prvRadioRxTask, (const char *) "RadioRx", 128, NULL, RADIO_RX_TASK_PRIORITY, NULL);
#elif (RADIO_MODE == RADIO_MODE_TX)
	
    /* Turn on the sensors board. Set PA7 to high. */
	sensors_pwr_ctrl.high();

	/* Enable i2c */
	i2c.enable();	
		
	
	// Create the radio transmit task
    xTaskCreate(prvRadioTxTask, (const char *) "RadioTx", 128, NULL, RADIO_TX_TASK_PRIORITY, NULL);
	
	
	
#endif

    // Start the scheduler
    Scheduler::run();
	
	return 0;
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
    while(true)
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
            // Turn the yellow LED off when a packet is received
            led_yellow.off();

            // Get a packet from the radio buffer
            radio_ptr = radio_buffer;
            radio_len = sizeof(radio_buffer);
			
            result = radio.getPacket(radio_ptr, &radio_len, &rssi, &lqi, &crc);
			//result = radio.getPacket(radio_ptr, &radio_len, &rssi, &lqi, 0);

			
			len = sprintf((char*) uartBuffer, "RSSI: %i, RADIO_LEN: %u, LQI: %u\r\n", rssi, radio_len, lqi);
			
            if (result == RadioResult_Success && crc)
            {
            	//uart0.writeByte(rssi);
				//uart0.writeByte(radio_len);
				//uart0.writeByte(lqi);
                //uart0.writeByte(crc >> 7);
				if (len > 0)
				{
				uart0.writeBytes(uartBuffer, len);
				len = 0;
				}				
            }
			
/*			len = sprintf((char*) uartBuffer, "Radio PTR1: %u \r\n", &radio_ptr);
			if (result == RadioResult_Success && crc)
            {
            	//uart0.writeByte(rssi);
				//uart0.writeByte(radio_len);
				//uart0.writeByte(lqi);
                //uart0.writeByte(crc >> 7);
				if (len > 0)
				{
				uart0.writeBytes(uartBuffer, len);
				len = 0;
				}				
            }
			
			len = sprintf((char*) uartBuffer, "Radio PTR2: %u \r\n", radio_ptr);
			if (result == RadioResult_Success && crc)
            {
            	//uart0.writeByte(rssi);
				//uart0.writeByte(radio_len);
				//uart0.writeByte(lqi);
                //uart0.writeByte(crc >> 7);
				if (len > 0)
				{
				uart0.writeBytes(uartBuffer, len);
				len = 0;
				}				
            }
*/			
			len = sprintf((char*) uartBuffer, "Radio vector read: %s \r\n", radio_ptr );
			if (result == RadioResult_Success && crc)
            {
            	//uart0.writeByte(rssi);
				//uart0.writeByte(radio_len);
				//uart0.writeByte(lqi);
                //uart0.writeByte(crc >> 7);
				if (len > 0)
				{
				uart0.writeBytes(uartBuffer, len);
				len = 0;
				}				
            }
			
            
            // Turn off the radio until the next packet
            radio.off();
			Scheduler::delay_ms(1000);
        }
    }
}

static void prvRadioTxTask(void *pvParameters)
{
	
    static RadioResult result;
	
    SensorData sensorData;
    Bme280Data bme280_data;
    size_t len = 0;
	bool status;



    // Get the EUI64 address of the board
    board.getEUI48(radio_buffer);

	//memcpy(&radio_buffer[3], &temp[5], 3);	
	//board.getEUI64(radio_buffer);
	led_orange.off();
	/* Initialize the bme280 */
	bme280.init();
	
	
    // Forever
    while(true)
    {
		led_orange.on();
		
		/* Check whether we can read data from bme280 */
		status = bme280.read(&bme280_data);
		
		if (status)
		{
			sensorData.temperature = (uint16_t)(bme280_data.temperature * 1.0f);
			sensorData.humidity = (uint16_t)(bme280_data.humidity * 1.0f);
			sensorData.pressure = (uint16_t)(bme280_data.pressure * 1.0f);
		}

	
		len = sprintf((char*)uartBuffer, "Temperature: %u °C, Humidity: %u, Pressure: %u Bar ----> IoT-Handler\r\n", sensorData.temperature, sensorData.humidity, sensorData.pressure);	
		
		if (len > 0)
		{
		  /* Write bytes to Serial */
		  uart0.writeBytes(uartBuffer, len);
		  len = 0;
		}
		
		uint8_t temp[124];		
		
		len = sprintf((char*)temp, "Temperature: %u °C, Humidity: %u, Pressure: %u Bar ----> IoT-Handler\r\n", sensorData.temperature, sensorData.humidity, sensorData.pressure);
		
		memcpy(&radio_buffer[0], &temp[0], len);
		
		//len = 0;
		
        // Take the txSemaphre, block until available
        if (txSemaphore.take())
        {
            // Turn on the radio transceiver
            radio.on();

            // Turn the yellow LED on when the packet is being loaded
            led_yellow.on();

            // Load the EUI64 address to the transmit buffer
            radio_ptr = radio_buffer;
            //radio_len = EUI48_LENGTH;
			radio_len = sizeof(radio_buffer);
				
			
            result = radio.loadPacket(radio_ptr, radio_len);

            if (result == RadioResult_Success)
            {
                // Put the radio transceiver in transmit mode
                radio.transmit();
				Scheduler::delay_ms(100);
                // Turn the yellow LED off when the packet has beed loaded
                led_yellow.off();
            }

            // Delay the transmission of the next packet 250 ms
            vTaskDelay(10 / portTICK_RATE_MS);			
			
        }
		led_orange.off();
		
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
