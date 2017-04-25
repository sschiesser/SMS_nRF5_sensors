#include "sms_pressure.h"
#include "ms58.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "PRESSURE"
#include "nrf_log.h"
#include "app_error.h"

extern const nrf_drv_spi_t spi_master_instance;
extern volatile bool spi_xfer_done;
extern uint8_t m_tx_buf[];
extern uint8_t m_rx_buf[];

ms58_output_s ms58_output;
ms58_config_s ms58_config;
ms58_interrupt_s ms58_interrupt;

static void ms58_reset(void)
{
	NRF_LOG_INFO("Resetting MS58\n\r");
	uint8_t tx_len = 1;
	uint8_t rx_len = 0;
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_RESET;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done);
	nrf_delay_ms(4);
}

static void ms58_read_prom(void)
{
	uint8_t tx_len = 1;
	uint8_t rx_len = 3;
	
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_1;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[1] = (m_rx_buf[1] << 8 | m_rx_buf[2]);
		
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_2;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[2] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_3;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[3] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_4;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[4] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_5;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[5] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_6;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[6] = (m_rx_buf[1] << 8 | m_rx_buf[2]);


	// Send the first conversion command
	tx_len = 1;
	rx_len = 0;
	m_tx_buf[0] = MS58_CONV_D1_4096;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done);
}


void ms58_startup(void)
{
	// Initialize all ms58 struct values
	ms58_config.init_ok = true;
	ms58_config.dev_enabled = false;
	ms58_output.complete = false;
	ms58_output.pressure = 0;
	ms58_output.temperature = 0;
	for(uint8_t i = 0; i < MS58_PROM_VAL_MAX; i++) {
		ms58_output.prom_values[i] = 0;
	}
	for(uint8_t i = 0; i < MS58_ADC_VAL_MAX; i++) {
		ms58_output.adc_values[i] = 0;
	}
	ms58_interrupt.enabled = false;
	ms58_interrupt.new_value = false;
	ms58_interrupt.rts = false;
	
	// Reset device & read PROM factory settings
	ms58_reset();
	ms58_read_prom();
	
	// Roughly test PROM values
	for(uint8_t i = 1; i < 7; i++) {
		if((ms58_output.prom_values[i] == 0) || (ms58_output.prom_values[i] == 0xff)) {
			NRF_LOG_INFO("MS58 PROM suspect value: 0x%#x\r\n", ms58_output.prom_values[i]);
			ms58_config.init_ok = false;
		}
	}
	
	ms58_config.dev_enabled = ms58_config.init_ok;
}
