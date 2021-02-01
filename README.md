WaveBoost Sensor Board

1. Overview: 
This project is a part of WaveBoost power harvesting system. 
WaveBoost Sensor Board is including : 
  - Bluetooth low energy chip : nrf52840
  - Humidity + Pressure + Temperature sensor : BME280
  - Light sensors : (update later)
 
 The nrf52840 collect data from above sensors and advertise them periodically (the interval of advertising is configurable).
 The Bluetooth system works on Scan Active mode.
 
 2 Compile the project:
  - Step 1 : Download nordic sdk 	nRF5_SDK_15.2.0_9412b96.zip from the link https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/
  - Step 2 : Check out or download this project.
  - Step 3 : unzip the downloaded files (nordic sdk and project), copy them and put them in same folder.
    NOTE : normally, unzip process will create one more level of folder. For example : abc.zip, after unzip, the folder path like : abc/abc/....
           so just copy the root folder.
  - Step 4 : Open the project by double click on the file : nr52840_Rx/targets/ruuvitag_b/ses/ruuvi.firmware.c.emProject , then compile the project.
  
  3 GPIO Connection: 
   - BME280 : 
   SCLK <--> 13
   
   MOSI <--> 15  
   
   MISO <--> 17
   
   SS   <--> 20
              
   - VEML6305 : 
   
   SCL <--> 7
   
   SDA <--> 8
  
  4 Change the interval advertising: 
   - go to the file application_config.h, and search for the macro : APPLICATION_ADVERTISING_INTERVAL and change its value
   (#define APPLICATION_ADVERTISING_INTERVAL              1010)



*** active scanning : 
go to the file ble_advertising.c in SDK.
search for the function : ble_advertising_start
add line :  p_advertising->adv_params.scan_req_notification = 1; under if condition : if (p_advertising->adv_mode_current != BLE_ADV_MODE_IDLE)


***<error> app: ASSERT FAILED at..... ***
open the file : nRF5_SDK_15.2.0_9412b96\components\libraries\pwr_mgmt\nrf_pwr_mgmt.c
comment line 125 : ASSERT((original_fpscr & 0x7) == 0); ==> //ASSERT((original_fpscr & 0x7) == 0)

this is not a bug in application code, it is because checking of SDK. 
refer : https://devzone.nordicsemi.com/f/nordic-q-a/29419/pwr_mgmt_fpu_sleep_prepare-fails-with-assert