
#include <stdio.h>
#include "veml7700.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "boards.h"
#include "app_util_platform.h"

//#define NRF_LOG_MODULE_NAME "VEML7700"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief TWI events handler.
 */
void twi_veml7700_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //NRF_LOG_INFO("twi_veml7700_handler p_event->type %d\r\n", p_event->type);
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            //NRF_LOG_INFO("NRF_DRV_TWI_EVT_DONE\r\n");
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void veml770_twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_veml7700_config = {
       .scl                = VEML700_SDL,
       .sda                = VEML700_SDA,
       .frequency          = TWI_FREQUENCY_FREQUENCY_K400,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_veml7700_config, twi_veml7700_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

int veml_config(uint8_t reg, uint8_t reg_lsb, uint8_t reg_msb)
{
    NRF_LOG_INFO("veml_config\r\n");
    ret_code_t err_code;
    uint8_t reg_data[3] = {0x00, 0x00, 0x00};
 
    err_code = nrf_drv_twi_tx(&m_twi, VEML7700_ADDR, reg_data, sizeof(reg_data), false);
    APP_ERROR_CHECK(err_code);

    while (m_xfer_done == false);
    NRF_LOG_INFO("veml_config done\r\n");
    return 0;
}

uint16_t veml_read_luminosity(uint8_t reg)
{
    // send slave addr/Wr + command code + no_stop
    ret_code_t err_code;
    uint8_t reg_data = reg;
    uint8_t luminosity[2];
    uint16_t result = 0;
    
    err_code = nrf_drv_twi_tx(&m_twi, VEML7700_ADDR, &reg_data, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    // Rx 2 bytes 
    m_xfer_done = false;
    // Read 2 byte from the specified address
    nrf_delay_ms(5);
    err_code = nrf_drv_twi_rx(&m_twi, VEML7700_ADDR, luminosity, sizeof(luminosity));
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;
    nrf_delay_ms(5); // don't know why need delay here to get correct value
    result = luminosity[1];
    result =  (result << 8) + luminosity[0];
    //NRF_LOG_INFO("veml_read_luminosity result %d \r\n", result);
    return result;
}
