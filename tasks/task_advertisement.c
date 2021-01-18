/**
 * Ruuvi Firmware 3.x advertisement tasks.
 *
 * License: BSD-3
 * Author: Otso Jousimaa <otso@ojousima.net>
 **/

#include "application_config.h"
#include "ruuvi_boards.h"
#include "ruuvi_driver_error.h"
#include "ruuvi_endpoint_3.h"
#include "ruuvi_endpoint_5.h"
#include "ruuvi_interface_acceleration.h"
#include "ruuvi_interface_adc.h"
#include "ruuvi_interface_communication_ble4_advertising.h"
#include "ruuvi_interface_communication_radio.h"
#include "ruuvi_interface_environmental.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"
#include "ruuvi_interface_watchdog.h"
#include "task_adc.h"
#include "task_advertisement.h"
#include "task_acceleration.h"
#include "task_environmental.h"
#include "veml7700.h"
#include "nrf_log.h"
RUUVI_PLATFORM_TIMER_ID_DEF(advertisement_timer);
ruuvi_interface_communication_t channel;
int8_t Is_Adv_Over = 0;
int8_t cnt_adv = 0;
//handler for scheduled advertisement event
static void task_advertisement_scheduler_task(void *p_event_data, uint16_t event_size)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  // Update BLE data
  
  if(cnt_adv++ < 2)
  {
    //nrf_gpio_pin_write(RESERVATION_GPIO_1,1);
    //nrf_gpio_pin_write(RESERVATION_GPIO_2,1);
    if(APPLICATION_DATA_FORMAT == 3) { err_code |= task_advertisement_send_3(); }
    NRF_LOG_INFO("advertisement = %d \r\n", cnt_adv);
  }
  else
  {
    ruuvi_platform_timer_stop(advertisement_timer);
    ruuvi_interface_communication_ble4_advertising_uninit(&channel);
    
    Is_Adv_Over = 1;
    cnt_adv = 0x0F;
  }
}

// Timer callback, schedule advertisement event here.
static void task_advertisement_timer_cb(void* p_context)
{
  ruuvi_platform_scheduler_event_put(NULL, 0, task_advertisement_scheduler_task);
}

ruuvi_driver_status_t task_advertisement_init(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  err_code |= ruuvi_interface_communication_ble4_advertising_init(&channel);
  err_code |= ruuvi_interface_communication_ble4_advertising_tx_interval_set(APPLICATION_ADVERTISING_INTERVAL);
  int8_t target_power = APPLICATION_ADVERTISING_POWER;
  err_code |= ruuvi_interface_communication_ble4_advertising_tx_power_set(&target_power);
  err_code |= ruuvi_interface_communication_ble4_advertising_manufacturer_id_set(RUUVI_BOARD_BLE_MANUFACTURER_ID);
  err_code |= ruuvi_platform_timer_create(&advertisement_timer, RUUVI_INTERFACE_TIMER_MODE_REPEATED, task_advertisement_timer_cb);
  err_code |= ruuvi_platform_timer_start(advertisement_timer, APPLICATION_ADVERTISING_INTERVAL);
  return err_code;
}

ruuvi_driver_status_t task_advertisement_send_3(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_acceleration_data_t acclereration;
  ruuvi_interface_adc_data_t battery;
  ruuvi_interface_environmental_data_t environmental;
  int light = 0;
  // Get data from sensors
  err_code |= task_acceleration_data_get(&acclereration);
  #if(!ADVERTISE_WITH_DUMMY_DATA)
  #if(ADVERTISE_WITH_BME280)
  err_code |= task_environmental_data_get(&environmental);
  #endif
  #endif
  err_code |= task_adc_battery_get(&battery);
  
  #if(!ADVERTISE_WITH_DUMMY_DATA)
  #if(ADVERTISE_WITH_VEML6035)
  light = veml_read_luminosity(VEML7700_REG_ALS);
  //NRF_LOG_INFO("light = %d \r\n", light);
  #endif
  #endif
  // end 
  ruuvi_endpoint_3_data_t data;
  data.accelerationx_g = acclereration.x_g;
  data.accelerationy_g = acclereration.y_g;
  data.accelerationz_g = acclereration.z_g;
  data.humidity_rh = environmental.humidity_rh;
  data.temperature_c = environmental.temperature_c;
  data.pressure_pa = environmental.pressure_pa;
  data.battery_v = battery.adc_v;
  data.light = light; 
  ruuvi_interface_communication_message_t message;
  message.data_length = RUUVI_ENDPOINT_3_DATA_LENGTH;
  ruuvi_endpoint_3_encode(message.data, &data, RUUVI_DRIVER_FLOAT_INVALID);
  err_code |= channel.send(&message);

  return err_code;
}

ruuvi_driver_status_t task_advertisement_send_5(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  static uint16_t sequence = 0;
  uint8_t movement_counter = 0;
  sequence++;

  // Data format 5 considers sequence with all bits set as invalid
  // TODO: Remove hardcoding, add sequence number getter
  if(0xFFFF == sequence) { sequence = 0; }
  ruuvi_interface_acceleration_data_t acclereration;
  ruuvi_interface_adc_data_t battery;
  ruuvi_interface_environmental_data_t environmental;

  // Get data from sensors
  err_code |= task_acceleration_data_get(&acclereration);
  err_code |= task_acceleration_movement_count_get(&movement_counter);

  // Data format 5 considers sequence with all bits set as invalid
  // TODO: Remove hardcoding,
  if(0xFF == movement_counter) { movement_counter = 0; }
  err_code |= task_environmental_data_get(&environmental);
  err_code |= task_adc_battery_get(&battery);

  ruuvi_endpoint_5_data_t data;
  data.accelerationx_g = acclereration.x_g;
  data.accelerationy_g = acclereration.y_g;
  data.accelerationz_g = acclereration.z_g;
  data.humidity_rh = environmental.humidity_rh;
  data.temperature_c = environmental.temperature_c;
  data.pressure_pa = environmental.pressure_pa;
  data.battery_v = battery.adc_v;
  data.tx_power = APPLICATION_ADVERTISING_POWER;
  data.measurement_count = sequence;
  data.movement_count = movement_counter;
  ruuvi_interface_communication_radio_address_get(&(data.address));


  ruuvi_interface_communication_message_t message;
  message.data_length = RUUVI_ENDPOINT_5_DATA_LENGTH;
  ruuvi_endpoint_5_encode(message.data, &data, RUUVI_DRIVER_FLOAT_INVALID);
  err_code |= channel.send(&message);

  return err_code;
}
