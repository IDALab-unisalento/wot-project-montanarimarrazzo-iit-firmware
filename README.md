# wot-project-part2-iotproject-firmwareiit
## IoT project 2021 - Firmware for signals acquisition and trasmission over Bluetooth Low Energy
This project contains acquisition from ADC code and trasmission of it over Bluetooth Low Energy using peripheral uart example of Nordic.

## Requirements
<ul>
  <li>nRF52832 Evalution Kit</li>
  <li>nRF Connect SDK v1.5.1</li>
</ul>

This project was created by working on the board nrf52832, to use it correctly select the board contained in the folder boards/arm/ebshsnzwz_nrf52832.

## About this project
This project was created in two parts to separate functionality, the files are as follows:
<ul>
  <li><b>adc.c</b> : to perform the acquisition of input signals are used two of the adc channels among those available, the first part of the file consists
  of channels configuration, setting the values of: num of channels, gain, voltage reference and acquisition interval. There are two main function: 
  <b>adc_init()</b> that execute the initialization of adc and of channels, and <b>adc_sample()</b> that perform the acquisition of samples on channel input.</li>
  
  <li><b>main.c</b> : this file contains all operation to manage bluetooth, contained in peripheral_uart example, and implementation of acquisition signals performed
  by adc, and transmission of array of samples over Bluetooth Low Energy.</li>
</ul>

This application is built to be used with nRF Connect, that can be downloaded from https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-desktop.

For any questions visit https://devzone.nordicsemi.com/.
