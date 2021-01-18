/* Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef VEML7700_H__
#define VEML7700_H__

#include <stdint.h>

/* TWI instance ID. */
#define TWI_INSTANCE_ID                 1
#define VEML700_SDL			NRF_GPIO_PIN_MAP(0,7)//
#define VEML700_SDA			NRF_GPIO_PIN_MAP(0,8)//
#define VEML7700_ADDR                   0x29U


#define VEML7700_REG_ALS_CONFIG      0x00U
#define VEML7700_REG_ALS_WH          0x01U
#define VEML7700_REG_ALS_WL          0x02U
#define VEML7700_REG_ALS             0x04U
/**@brief Function for initializing the battery voltage module.
 */
int veml_config(uint8_t reg, uint8_t reg_lsb, uint8_t reg_msb);
uint16_t veml_read_luminosity(uint8_t reg);
void veml770_twi_init (void);


#endif
