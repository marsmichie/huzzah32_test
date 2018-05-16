#include <sstream>
#include <cstring>
#include "sdkconfig.h"
#include "esp_log.h"
#include "Pins_huzzah32.h"
#include "BMM_reg.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"



/* SPI stuff goes here */
#define MOSI_ESP1  GPIO_NUM_18
#define MISO_ESP1  GPIO_NUM_19
#define SCK_ESP1   GPIO_NUM_5

#define NOP_                __asm__ ("nop\n\t");
// ~4ns for 240 MHz

#define SPI_READ    (0x80)     //read command
#define SPI_WRITE   (0x7f)   // write command


spi_device_handle_t spiSetup () {
								esp_err_t ret;
								spi_device_handle_t spi;
								spi_bus_config_t buscfg;

								buscfg.miso_io_num = MISO_ESP1;
								buscfg.mosi_io_num = MOSI_ESP1;
								buscfg.sclk_io_num = SCK_ESP1;
								buscfg.quadwp_io_num = -1;
								buscfg.quadhd_io_num = -1;
								buscfg.max_transfer_sz = 4096; // maximum transfer size in bytes

								spi_device_interface_config_t devcfg;
								devcfg.clock_speed_hz = 1500000;
								devcfg.mode = 3;
								devcfg.queue_size = 1;
								devcfg.dummy_bits = 0;
								devcfg.address_bits = 8;
								devcfg.cs_ena_pretrans = 0;
								devcfg.cs_ena_posttrans = 0;
								devcfg.spics_io_num = -1;
								devcfg.command_bits = 0;
								devcfg.duty_cycle_pos = 128; // default 128 = 50%/50% duty
								devcfg.flags = 0; // 0 not used
								devcfg.pre_cb = NULL;
								devcfg.post_cb = NULL;

								// initialize SPI
								/* last parameter -> DMA channel? allows to transmit more than 32 bytes at once */
								ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1); // 0 -> DMA not used
								assert(ret == ESP_OK);
								// Attach the slave(s) to the SPI bus
								ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
								assert(ret == ESP_OK);
								return spi;
}

/**
 * Reading
 */

esp_err_t readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) {
								if(length == 0) return ESP_ERR_INVALID_SIZE;
								spi_transaction_t transaction;
								memset(&transaction, 0, sizeof(transaction));
								transaction.flags = 0;//SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
								// command may be changed to 'cmd' if not compiling
								transaction.command = 0;
								// address may be changed to 'addr' if not compiling
								transaction.address = regAddr | SPI_READ;
								transaction.length = 8;
								transaction.rxlength = length * 8;
								transaction.user = (void*)0;
								//transaction.tx_data[0] = regAddr | SPI_READ;
								transaction.tx_buffer = NULL;
								transaction.rx_buffer = data;
								esp_err_t err = spi_device_transmit(handle, &transaction);

								return err;
//								assert(err == ESP_OK);
//								return transaction.rx_data[0];
}

/**
 * Writing
 */

uint8_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data) {
								spi_transaction_t transaction;
								memset(&transaction, 0, sizeof(spi_transaction_t));
								transaction.flags = 0;//SPI_TRANS_USE_TXDATA;
								// command may be changed to 'cmd' if not compiling
								transaction.command = 0;//(uint8_t)(*data);
								// address may be changed to 'addr' if not compiling
								transaction.address = regAddr & SPI_WRITE; // set highest bit of sending data to 0
								transaction.length = length * 8;
								//transaction.tx_data[0] = regAddr & SPI_WRITE;
								//transaction.tx_data[1] = (uint8_t)(*data);
								transaction.rxlength = 0;
								transaction.user = (void*)0;
//								transaction.rx_data[0] = *data;
								transaction.tx_buffer = data;
								transaction.rx_buffer = NULL;

								esp_err_t err = spi_device_transmit(handle, &transaction);
								assert(err == ESP_OK);
								return err;

}

/* end of SPI stuff */

/* assign some Macro functions PINS */
#define DFF_DATA_HIGH       (gpio_set_level(DFF_DATA, HIGH))
#define DFF_DATA_LOW        (gpio_set_level(DFF_DATA, LOW))

#define DFF_PRE_HIGH        (gpio_set_level(DFF_PRE, HIGH))
#define DFF_PRE_LOW         (gpio_set_level(DFF_PRE, LOW))

#define DFF_CLR_A_LOW       (gpio_set_level(DFF_CLR_A, LOW))
#define DFF_CLR_A_HIGH      (gpio_set_level(DFF_CLR_A, HIGH))

#define RED_LED      (GPIO_NUM_13)

void pinSetup () {
								/** DFF related pins **/

								// set DFF_PRE to HIGH (default)
								gpio_set_direction(DFF_PRE, GPIO_MODE_OUTPUT);
								DFF_PRE_LOW;
								vTaskDelay(10 / portTICK_RATE_MS);
								DFF_PRE_HIGH;

								// set DFF_CLR to LOW
								gpio_pad_select_gpio(DFF_CLR_A);
								gpio_set_direction(DFF_CLR_A, GPIO_MODE_OUTPUT);
								gpio_set_level(DFF_CLR_A, LOW);

								// clock input of DFF -> rising edge activates
								gpio_pad_select_gpio(DFF_CLK);
								gpio_set_direction(DFF_CLK, GPIO_MODE_OUTPUT); // DFF Clock
								gpio_set_level(DFF_CLK, LOW);

								// DFF data in
								gpio_pad_select_gpio(DFF_DATA);
								gpio_set_direction(DFF_DATA, GPIO_MODE_OUTPUT); // Data input DFF 1
								gpio_set_level(DFF_DATA, LOW);
								vTaskDelay(10 / portTICK_RATE_MS);
								gpio_set_level(DFF_DATA, HIGH);

								// out from last DFF
								gpio_pad_select_gpio(DFF_LAST);
								gpio_set_direction(DFF_LAST, GPIO_MODE_INPUT); // output from last DFF

								/** end DFF related pins **/

								// pin state: is 1 if all sensors are ready with measurement
								// NOTE: does not work if one of the sensors isn't working
								//gpio_pad_select_gpio(ALL_DATA_RDY);
								//gpio_set_direction(ALL_DATA_RDY, GPIO_MODE_INPUT); // checks if all sensors are ready with measurement

								vTaskDelay(10 / portTICK_RATE_MS);
								gpio_set_level(DFF_CLR_A, HIGH);

								gpio_pad_select_gpio(RED_LED);
								gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);

}

/* end of PIN assigments */

/*
 * some functions
 */

/**
 * manual positive edge for DFF clock
 */
inline void dff_low_high_trig() {
								gpio_set_level(DFF_CLK, LOW);
								NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; // wait 32 ns
								gpio_set_level(DFF_CLK, HIGH);
								NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; // wait 32 ns
}

/**
 * set all outputs (Q) of DFFs to LOW
 * deselect all sensors
 */
inline void clearDFF() {
								gpio_set_level(DFF_CLR_A, LOW);
								NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_;
								gpio_set_level(DFF_CLR_A, HIGH);
								NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_;
}

/**
 * set all Q_inv to LOW (chip select LOW for all sensors)
 * select all sensors
 */
inline void setAllDFF() {
								DFF_PRE_LOW;
								DFF_CLR_A_HIGH;
								NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_; NOP_;
}

/**
 * go to specific sensorNo -> only for testing single sensors
 * set individual chip select to low for sensor number <sensorNo>
 */
inline void jumpToSens(int sensorNo) {
								clearDFF();
								ets_delay_us(10);
								DFF_DATA_HIGH;
								//delayMicroseconds(1);
								dff_low_high_trig();
								DFF_DATA_LOW;
								for (uint16_t sens_i = 1; sens_i < sensorNo; sens_i++) {
																dff_low_high_trig();
								}
}

/* end of some functions */

void myTask(void *pvParameters) {
								spi_device_handle_t myspi = spiSetup();

								/** try to init sensor number <test_sensor> **/
								int test_sensor = 7;
								// select sensor via DFF
								jumpToSens(test_sensor);
								ets_delay_us(100);
								// set powerBit in order to read
								ESP_ERROR_CHECK(writeBytes(myspi, BMM150_POWER_CONTROL_ADDR, 1, &powerBit));
								ets_delay_us(100);
								// deselect all sensors
								clearDFF();


								while (1) {
																jumpToSens(test_sensor);
																NOP_;NOP_;NOP_;
																// set powerBit in order to read
																/* NOTE: change writeBytes function -> now only for powerBit */
																ESP_ERROR_CHECK(writeBytes(myspi, BMM150_POWER_CONTROL_ADDR, 1, &powerBit));
																ets_delay_us(1);
																// deselect all sensors
																clearDFF();
																// blinking blue LED
																gpio_set_level(RED_LED, HIGH);

																/** try to read deviceID from sensor **/
																jumpToSens(test_sensor);
																// read deviceID
																NOP_;
																uint8_t buffer[1];
																//ESP_ERROR_CHECK(readBytes(myspi, 0x40, 1, buffer));
																readBytes(myspi, 0x40, 2, buffer);
																vTaskDelay(1000/portTICK_RATE_MS);
																gpio_set_level(RED_LED, LOW);
																vTaskDelay(500/portTICK_RATE_MS);
																// deselect all sensors
																clearDFF();
																// printf
																printf("response should be 50: %i \n", buffer[0]);

								}

}

#ifdef __cplusplus
extern "C" {
#endif

void app_main() {
								vTaskDelay(1000 / portTICK_PERIOD_MS);

								// external pullups necessary
								gpio_set_pull_mode(MOSI_ESP1, GPIO_PULLUP_ONLY);
								gpio_set_pull_mode(MISO_ESP1, GPIO_PULLUP_ONLY);

								pinSetup();
								//xTaskCreatePinnedToCore(&myTask, "spi_test_task", 2096, NULL, 5, NULL, 0);
								myTask(NULL);
								ESP_LOGW("test", "task started...");
}
#ifdef __cplusplus
}
#endif
