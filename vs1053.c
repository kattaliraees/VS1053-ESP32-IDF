
#include "vs1053.h"
#include <driver/spi_master.h>
#include "driver/gpio.h"

//SPI & IO Connections
#define GPIO_SCLK	18
#define GPIO_MISO	19
#define GPIO_MOSI	23
#define GPIO_CS		33
#define GPIO_DCS	25
#define GPIO_DREQ	26
#define GPIO_RESET	27

spi_device_handle_t spi_handle;


void vs1053_init() {

	esp_err_t ret;

	//CS, DCS, RESET GPIOs
	gpio_pad_select_gpio(GPIO_CS);
	gpio_set_direction(GPIO_CS, GPIO_MODE_OUTPUT);

	gpio_pad_select_gpio(GPIO_DCS);
	gpio_set_direction(GPIO_DCS, GPIO_MODE_OUTPUT);
  
	gpio_pad_select_gpio(GPIO_RESET);
	gpio_set_direction(GPIO_RESET, GPIO_MODE_OUTPUT);

	//DREQ GPIO as input
	gpio_config_t dreq_gpio_conf;
	dreq_gpio_conf.mode = GPIO_MODE_INPUT;
	dreq_gpio_conf.pull_up_en =	GPIO_PULLUP_DISABLE;
	dreq_gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	dreq_gpio_conf.intr_type = GPIO_INTR_DISABLE;   
	dreq_gpio_conf.pin_bit_mask = ((uint64_t)(((uint64_t)1)<<CONFIG_GPIO_DREQ));
	ESP_ERROR_CHECK(gpio_config(&dreq_gpio_conf));


	spi_bus_config_t buscfg = {
		.sclk_io_num = GPIO_SCLK,
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_MISO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.flags = SPICOMMON_BUSFLAG_MASTER
	};

	int freq = 800000;
	spi_device_interface_config_t spicfg={
		.clock_speed_hz = freq,
        .command_bits = 8,
        .address_bits = 8,
        .dummy_bits = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 1,
		.flags = SPI_DEVICE_NO_DUMMY,
		.mode = 0,
		.spics_io_num = -1,
		.queue_size = 1
	};

	ret = spi_bus_initialize( HSPI_HOST, &buscfg, 1 );
	assert(ret==ESP_OK);

	ret = spi_bus_add_device( HSPI_HOST, &spicfg, &spi_handle);
	assert(ret==ESP_OK);
	SLEEP_MS(20);
	gpio_set_level(CONFIG_GPIO_CS, 1);
	gpio_set_level(CONFIG_GPIO_DCS, 1);
	gpio_set_level(CONFIG_GPIO_RESET, 0);
	SLEEP_MS(5);
	gpio_set_level(CONFIG_GPIO_RESET, 1);

	uint16_t reg_data = vs1053_read_sci(SCI_MODE);

	if(reg_data != (SM_LINE1 | SM_SDINEW)) {
		vs1053_write_sci(SCI_MODE, (SM_LINE1 | SM_SDINEW));
	}

	vs1053_write_sci(SCI_AUDATA, 44101);
	vs1053_write_sci(SCI_CLOCKF, 6 << 12);

	spicfg.clock_speed_hz = 6000000;
	spicfg.command_bits = 0;
	spicfg.address_bits = 0;
	ret = spi_bus_add_device( HSPI_HOST, &spicfg, &spi_handle);
	assert(ret==ESP_OK);
}

//For writing data to VS10xx registers at addr
void vs1053_write_sci(uint8_t addr, uint16_t data) {
    
	uint8_t d = 2;
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	while(!gpio_get_level(CONFIG_GPIO_DREQ)); //Wait until DREQ is high

	gpio_set_level(CONFIG_GPIO_CS, 0);

	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.flags |= SPI_TRANS_USE_TXDATA;
	SPITransaction.cmd = d;
	SPITransaction.addr = addr;
	SPITransaction.tx_data[0] = (data >> 8) & 0xFF;
	SPITransaction.tx_data[1] = (data & 0xFF);
	SPITransaction.length= 16;
	ret = spi_device_transmit(spi_handle, &SPITransaction );
	assert(ret==ESP_OK);

	gpio_set_level(CONFIG_GPIO_CS, 1);
}

//For reading VS10xx register at addr
uint16_t vs1053_read_sci(uint8_t addr) {

	uint16_t res;
	uint8_t d = 3;
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	while(!gpio_get_level(CONFIG_GPIO_DREQ)); //Wait until DREQ is high

	gpio_set_level(CONFIG_GPIO_CS, 0);
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length=16;
	SPITransaction.flags |= SPI_TRANS_USE_RXDATA	;
	SPITransaction.cmd = d;
	SPITransaction.addr = addr;
	ret = spi_device_transmit(spi_handle, &SPITransaction );
	assert(ret==ESP_OK);
	res = (((SPITransaction.rx_data[0]&0xFF)<<8) | ((SPITransaction.rx_data[1])&0xFF)) ;
	gpio_set_level(CONFIG_GPIO_CS, 1);

	return res;
}

//Sending MP3 raw bytes. Max 32 bytes at time without checking DREQ each time
void vs1053_write_sdi(uint8_t *data, uint8_t bytes) {
	if(bytes > 32) {
		return;//Error - too many bytes to transfer
	}

	while(!gpio_get_level(CONFIG_GPIO_DREQ)); //Wait until DREQ is high

	spi_transaction_t SPITransaction;
	esp_err_t ret;

	gpio_set_level(CONFIG_GPIO_DCS, 0);

	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = bytes * 8;
	SPITransaction.tx_buffer = data;
	ret = spi_device_transmit(spi_handle , &SPITransaction );
	assert(ret==ESP_OK);
	gpio_set_level(CONFIG_GPIO_DCS, 1);
}
