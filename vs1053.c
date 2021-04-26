
#include "vs1053.h"
#include <driver/spi_master.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include<string.h>

static const int GPIO_SCLK = 18;
static const int GPIO_MISO = 19;
static const int GPIO_MOSI = 23;

#define CONFIG_GPIO_CS 32
#define CONFIG_GPIO_DCS 33
#define CONFIG_GPIO_DREQ 35
#define CONFIG_GPIO_RESET 5

#define TAG "VS1053"
static const char* LOG_TAG = "VS1053";


spi_device_handle_t spi_handle_low_speed;
spi_device_handle_t spi_handle_high_speed;


void vs1053_init() {

	esp_err_t ret;

	//CS, DCS, RESET GPIOs
	gpio_pad_select_gpio( CONFIG_GPIO_CS );
	gpio_set_direction( CONFIG_GPIO_CS, GPIO_MODE_OUTPUT );
	gpio_pad_select_gpio( CONFIG_GPIO_DCS );
	gpio_set_direction( CONFIG_GPIO_DCS, GPIO_MODE_OUTPUT );
    gpio_pad_select_gpio( CONFIG_GPIO_RESET );
    gpio_set_direction( CONFIG_GPIO_RESET, GPIO_MODE_OUTPUT );
	gpio_pad_select_gpio( CONFIG_GPIO_DREQ );
    gpio_set_direction( CONFIG_GPIO_DREQ, GPIO_MODE_INPUT );

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
	spi_device_interface_config_t devcfg={
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

	ret = spi_bus_add_device( HSPI_HOST, &devcfg, &spi_handle_low_speed);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	SLEEP_MS(20);
    gpio_set_level(CONFIG_GPIO_CS, 1);
	gpio_set_level(CONFIG_GPIO_DCS, 1);
	gpio_set_level(CONFIG_GPIO_RESET, 0);
	SLEEP_MS(5);
	gpio_set_level(CONFIG_GPIO_RESET, 1);

    uint16_t reg_data = vs1053_read_sci(SCI_MODE);

	if(reg_data != (SM_LINE1 | SM_SDINEW)) {

        LOG_INFO("%d", reg_data);
		ESP_LOGI(TAG, "Setting mode");
		vs1053_write_sci(SCI_MODE, (SM_LINE1 | SM_SDINEW));
	}

	reg_data = 0;
	reg_data = vs1053_read_sci(SCI_MODE);


	if(reg_data != (SM_LINE1 | SM_SDINEW)) {

		ESP_LOGI(TAG, "Setting mode");
		vs1053_write_sci(SCI_MODE, (SM_LINE1 | SM_SDINEW));
	}

	reg_data = vs1053_read_sci(SCI_AUDATA);
	LOG_INFO("SCI_AUDATA - %d", reg_data);

	reg_data = vs1053_read_sci(SCI_CLOCKF);
	LOG_INFO("SCI_CLOCKF - %d", reg_data);
	vs1053_write_sci(SCI_AUDATA, 44101); // 44.1kHz stereo
	// The next clocksetting allows SPI clocking at 5 MHz, 4 MHz is safe then.
	vs1053_write_sci(SCI_CLOCKF, 6 << 12); // Normal clock settings multiplyer 3.0 = 12.2 MHz

	//vs1053_set_volume(100);
	//vs1053_write_sci(0x0B, 0x0000);

	reg_data = vs1053_read_sci(SCI_AUDATA);
	LOG_INFO("SCI_AUDATA - %d", reg_data);

	reg_data = vs1053_read_sci(SCI_CLOCKF);
	LOG_INFO("SCI_CLOCKF - %d", reg_data);

	//reg_data = vs1053_read_sci(0x0B);
	//LOG_INFO("SCI_VOL - %d", reg_data);


	spi_device_interface_config_t devcfg_hs = devcfg;

	devcfg_hs.clock_speed_hz = 6000000;
	devcfg_hs.command_bits = 0;
	devcfg_hs.address_bits = 0;
	ret = spi_bus_add_device( HSPI_HOST, &devcfg_hs, &spi_handle_high_speed);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	
	reg_data = vs1053_read_sci(SCI_MODE);

	if(reg_data != (SM_LINE1 | SM_SDINEW)) {

        LOG_INFO("%d", reg_data);
		ESP_LOGI(TAG, "High speed - Setting mode");
		vs1053_write_sci(SCI_MODE, (SM_LINE1 | SM_SDINEW));
	}
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
	ret = spi_device_transmit(spi_handle_low_speed, &SPITransaction );
	assert(ret==ESP_OK);

    gpio_set_level(CONFIG_GPIO_CS, 1);
}

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
	ret = spi_device_transmit(spi_handle_low_speed, &SPITransaction );
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
    ret = spi_device_transmit(spi_handle_high_speed , &SPITransaction );
    assert(ret==ESP_OK);
    gpio_set_level(CONFIG_GPIO_DCS, 1);
}

void vs1053_set_volume(uint8_t vol) {
	
	// Set volume.	Both left and right.
	// Input value is 0..100.  100 is the loudest.
	uint16_t value; // Value to send to SCI_VOL

	value = map(vol, 0, 100, 0x20, 0x00); // 0..100% to one channel
	if (value == 0x20)
	{
		value = 0xFE;
	}
	value = (value << 8) | value;
	vs1053_write_sci(SCI_VOL, value); // Volume left and right

	uint16_t reg_data = vs1053_read_sci(SCI_VOL);
	LOG_INFO("SCI_VOL - %d", reg_data);
}
