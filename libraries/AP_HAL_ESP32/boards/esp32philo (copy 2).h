/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This board that does not contain any sensors (but pins are active), it is a great help for a novice user,
 * by flashing an empty board, you can connect via Mavlink (Mission Planner - MP) and gradually add sensors.
 * If you had some sensor configured and it doesn't work then the MP connection does not work and then you may not know what to do next.
*/

/////////////////////////////////////////////////////
//Pins
/*
 * I2c0 13,15
 * I2c1 23,19  >>> SPI instead as below:
 * SPI SDI 23
 *     CLK 19
 *     SDA(ADO/SD0) 18
 *     NCS 26
 * 
 * RMT 4
 * RCOUT 25,27, 33,32           // nope: , 22, 21
 *  UART 0:   3 , 1   (Con 115200)
 *  UART 1:   34 in only has to Rx , 18  
 *  UART 2:   16 , 17  (GPS     #define AP_SERIALMANAGER_GPS_BAUD               230400)

*/
/////////////////////////////////////////////////////





#pragma once

#define HAL_ESP32_BOARD_NAME "esp32philo" //phil TODO chg to unique name
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_PHILO

#define TRUE  1
#define FALSE 0

                                                            // //Protocols
                                                            // // list of protocols/enum:  ardupilot/libraries/AP_SerialManager/AP_SerialManager.h
                                                            // // default protocols:    ardupilot/libraries/AP_SerialManager/AP_SerialManager.cpp
                                                            // // ESP32 serials:    AP_HAL_ESP32/HAL_ESP32_Class.cpp

                                                            // //phil Note This is enabled despite commented out
                                                            // //#define DEFAULT_SERIAL0_PROTOCOL        SerialProtocol_MAVLink2   //A  UART0: Always: Console, MAVLink2
                                                            // //#define DEFAULT_SERIAL0_BAUD            AP_SERIALMANAGER_CONSOLE_BAUD/1000  //115200

                                                            // //#define DEFAULT_SERIAL1_PROTOCOL        SerialProtocol_MAVLink2   //C  WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
                                                            // //#define DEFAULT_SERIAL1_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600

                                                            // #define DEFAULT_SERIAL2_PROTOCOL        SerialProtocol_MAVLink2   //D  UART2
                                                            // #define DEFAULT_SERIAL2_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600

                                                            // #define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_GPS        //B  UART1: GPS1
                                                            // #define DEFAULT_SERIAL3_BAUD            AP_SERIALMANAGER_GPS_BAUD/1000    //38400, Can not define default baudrate here (by config only)
                                                            // //#define DEFAULT_SERIAL3_PROTOCOL        SerialProtocol_None       //B
                                                            // //#define DEFAULT_SERIAL3_BAUD            (115200/1000)

                                                            // #define DEFAULT_SERIAL4_PROTOCOL        SerialProtocol_None       //E
                                                            // #define DEFAULT_SERIAL5_BAUD            (115200/1000)

                                                            // #define DEFAULT_SERIAL5_PROTOCOL        SerialProtocol_None       //F
                                                            // #define DEFAULT_SERIAL5_BAUD            (115200/1000)

                                                            // #define DEFAULT_SERIAL6_PROTOCOL        SerialProtocol_None       //G
                                                            // #define DEFAULT_SERIAL6_BAUD            (115200/1000)

                                                            // #define DEFAULT_SERIAL7_PROTOCOL        SerialProtocol_None       //H
                                                            // #define DEFAULT_SERIAL7_BAUD            (115200/1000)

                                                            // #define DEFAULT_SERIAL8_PROTOCOL        SerialProtocol_None       //I
                                                            // #define DEFAULT_SERIAL8_BAUD            (115200/1000)

                                                            // #define DEFAULT_SERIAL9_PROTOCOL        SerialProtocol_None       //J
                                                            // #define DEFAULT_SERIAL9_BAUD            (115200/1000)

//-------------------------------------UARTS-----------------------------------------
            // # Now the serial ordering. These map to the SERIALn_ parameter numbers
            // # If you use a shorter list then HAL_Empty::UARTDriver
            // # objects are substituted for later UARTs, or you can leave a gap by
            // # listing one or more of the uarts as EMPTY.
            //Devices other than MCUs having SPI interface tend to use SDI/SDO or DIN/DOUT convention 
            // (e.g. ADC/DAC chips, digital potentiometers, sensors etc). But MCUs always 
            // (maybe there are exceptions but I've never faced one) use MOSI/MISO convention 
            //         (MSP430 series MCUs have a slight difference at this point: 
            //         They have SIMO/SOMI pins which are totally the same as MOSI/MISO). 
            //         This is because the MCUs are assumed (which should be) to be master. 
            
// # The normal usage of this ordering is:
        // # 1) SERIAL0: console (primary mavlink, usually USB)
        // # 2) SERIAL1: telem1
        // # 3) SERIAL2: telem2
        // # 4) SERIAL3: primary GPS
        // # 5) SERIAL4: GPS2
        // # 6) SERIAL5: extra UART (usually RTOS debug console)

        // # order of UARTs (and USB)
        // SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 UART7




//phil Note This is enabled despite commented out
//#define DEFAULT_SERIAL0_PROTOCOL        SerialProtocol_MAVLink2   //A  UART0: Always: Console, MAVLink2
//#define DEFAULT_SERIAL0_BAUD            AP_SERIALMANAGER_CONSOLE_BAUD/1000  //115200

//#define DEFAULT_SERIAL1_PROTOCOL        SerialProtocol_MAVLink2   //C  WiFi:  TCP, UDP, or disable (depends on HAL_ESP32_WIFI)
//#define DEFAULT_SERIAL1_BAUD            AP_SERIALMANAGER_MAVLINK_BAUD/1000  //57600

#define DEFAULT_SERIAL2_PROTOCOL        SerialProtocol_GPS        //B  UART2: GPS1
#define DEFAULT_SERIAL2_BAUD            AP_SERIALMANAGER_GPS_BAUD/1000    //38400, Can not define default baudrate here (by config only)


// UART_NUM_0 and UART_NUM_2 are configured to use defaults  //34 is Read ONLY
#define HAL_ESP32_UART_DEVICES  {.port=UART_NUM_0, .rx=GPIO_NUM_3 , .tx=GPIO_NUM_1 },{.port=UART_NUM_1, .rx=GPIO_NUM_34, .tx=GPIO_NUM_18},{.port=UART_NUM_2, .rx=GPIO_NUM_16, .tx=GPIO_NUM_17}





//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//and go to libraries/ap_scheduler/ap_scheduler.cpp and set the loop rate as 200hz
//You can add dubug print in AP_InertialSensor_Backend _notify_new_gyro_raw_sample function to see if there some data from IMU
//extra caps on pwr line always good, extra caps on data lines always bad
//Buzz 3/2/24 I use a jumper/switch as, for me, with-pullup sd-works, but flasjing doesnt... so i have to remove jumper while flashinh
//Buzz 4/7/24 ..from memory, esp32 is 3.3v nominal, but 5v tolerant on gpios
//𝒟𝑜𝓂𝒾𝓃𝒾𝒸 4/9/24, schematic and esp32 pcb files
//Buzz 6/5/24 in theory the esp32 codebase should suport as many compasses as ardupilot supports, but its not going to do any magical detection of them, u need to have the right pins and expected hardware definitions created for the code. 
//Buzz7/20/24 I usually use --disable-scripting when building
//..... up to discord August and stampfly stuff
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


//https://discuss.ardupilot.org/t/trying-to-understand-internal-exernal-compasses/55058/2
//dkemxr Dave Apr 2020 I2C are external, SPI are internal. ???????? If your screen looks different than 
//this install the latest MP beta!
//Compass Ordering and Priority¶
//During boot, ArduPilot automatically detects the compasses present in the system, adds them to a list,
//and assigns the first three a priority (1-3) linked to their DEV ID (COMPASS_PRIOx_ID), according to the 
//order in which they are discovered. This priority determines which compass is used by the EKF lanes


//-------------------------------------B U S E S -----------------------------------------
//I2C Buses
// make sensor selection clearer
                    // #define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
                    // #define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
                    // #define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
                    // #define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
                    // #define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))


                            // # Now define the order that I2C buses are presented in the hal.i2c API # in ArduPilot.
                            // # order of I2C buses ???????
                            // I2C_ORDER I2C2 I2C1

#define HAL_ESP32_I2C_BUSES {.port=I2C_NUM_0, .sda=GPIO_NUM_15, .scl=GPIO_NUM_13, .speed=400*KHZ, .internal=true, .soft=true},\
                            // Nope use SPI so can get buzz debug {.port=I2C_NUM_1, .sda=GPIO_NUM_19, .scl=GPIO_NUM_23, .speed=100*KHZ, .internal=true, .soft=true}

//SPI Buses
// make sensor selection clearer
                   // #define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
                   // #define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))
                   // #define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))
                   // #define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
// see IMU           #define HAL_ESP32_SPI_BUSES {}
//                   #define HAL_ESP32_SPI_DEVICES {}
                                      

//-------------------------------------radio -----------------------------------------
#define AP_RCPROTOCOL_ENABLED 1
//RCOUT
//esp32 hard coded for ppm  AND!!!!!  now  sbus ibus // Phil TODO
//this:
#define HAL_ESP32_RMT_RX_PIN_NUMBER	4
//or this:
//use UART
#define HAL_ESP32_RCOUT {GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_33, GPIO_NUM_32}

//-------------------------------------AIRSPEED----------------------------------------------
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

//-------------------------------------BAROMETER------------------------------------------------
#define HAL_BARO_ALLOW_INIT_NO_BARO 1
// BARO choices:
//#define HAL_BARO_DEFAULT HAL_BARO_BMP280_I2C
//#define HAL_BARO_BMP280_NAME "BMP280"
// or one of these:
//#define HAL_BARO_DEFAULT HAL_BARO_MS5837_I2C
// or: GPIO 34
//#define HAL_BARO_ANALOG_PIN (6)
// BARO probing:
//#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 1, 0x76)


//-----------------------------IMU----------------------------------------------------------
        //INS choices:
        //#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
        //#define HAL_INS_MPU9250_NAME "MPU9250"

        // or this:
        //#define HAL_INS_DEFAULT HAL_INS_MPU6500_I2C
        //#define HAL_INS_MPU6500_NAME "mpu6500"

//#define HAL_INS_ICM20XXX_I2C_ADDR (0x68)


#define AP_INERTIALSENSOR_ENABLED 1
//this defined /libraries/AP_HAL/board/esp32.h     #define HAL_EXTERNAL_AHRS_ENABLED 0
// #define AP_INERTIALSENSOR_KILL_IMU_ENABLED 0  //'Allow IMUs to be disabled at runtime', 0, None)

//#define HAL_INS_DEFAULT HAL_INS_NONE
//#define HAL_INS_DEFAULT HAL_INS_MPU9250_I2C

//nope use SPI #define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
// IMU probing:
//#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 0, 0x68, ROTATION_YAW_270)
//nope use SPI #define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 0, 0x68, ROTATION_NONE)


// Devices other than MCUs having SPI interface tend to use SDI/SDO or DIN/DOUT convention 
// (e.g. ADC/DAC chips, digital potentiometers, sensors etc). But MCUs always 
// (maybe there are exceptions but I've never faced one) use MOSI/MISO convention 
//         (MSP430 series MCUs have a slight difference at this point: 
//         They have SIMO/SOMI pins which are totally the same as MOSI/MISO). 
//         This is because the MCUs are assumed (which should be) to be master. 
//phil: SDI connects MCU,MOSI   SDO connects to MISO

#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
#define HAL_INS_MPU9250_NAME "mpu9250"
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI( Invensense, HAL_INS_MPU9250_NAME, ROTATION_NONE)
#define HAL_ESP32_SPI_BUSES   {.host=VSPI_HOST, .dma_ch=1, .mosi=GPIO_NUM_23, .miso=GPIO_NUM_18, .sclk=GPIO_NUM_19}
// SPI per-device setup, including speeds, etc.
#define HAL_ESP32_SPI_DEVICES  {.name="mpu9250", .bus=0, .device=0, .cs=GPIO_NUM_26,  .mode = 0, .lspeed=2*MHZ, .hspeed=8*MHZ}
#define HAL_SPI_DEVICE_LIST HAL_ESP32_SPI_DEVICES   // , deliminated






//-------------------------------------compass -----------------------------------------
#define AP_COMPASS_ENABLE_DEFAULT 0

// MAG/COMPASS choices:
// or others:
// #define HAL_COMPASS_QMC5883L_I2C_ADDR 0x0D
// #define HAL_MAG_PROBE_LIST PROBE_MAG_I2C(QMC5883L, 0, HAL_COMPASS_QMC5883L_I2C_ADDR, true, ROTATION_NONE)
//#define HAL_COMPASS_ICM20948_I2C_ADDR (0xD)
//#define HAL_COMPASS_AK09916_I2C_BUS 0
//#define HAL_COMPASS_AK09916_I2C_ADDR (0x0C)
//#define HAL_COMPASS_MAX_SENSORS 3

//#define ALLOW_ARM_NO_COMPASS
// #define AP_COMPASS_ENABLE_DEFAULT 1
// #define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
// #define HAL_PROBE_EXTERNAL_I2C_COMPASSES 1
// #define AP_COMPASS_AK8963_ENABLED TRUE
                                    // # two compasses. First is in the LSM303D
                                    // COMPASS LSM303D SPI:lsm9ds0_ext_am ROTATION_YAW_270
                                    // # 2nd compass is part of the 2nd invensense IMU
                                    // COMPASS AK8963:probe_mpu9250 1 ROTATION_YAW_270

                                    // # compass as part of ICM20948 on newer cubes
                                    // COMPASS AK09916:probe_ICM20948 0 ROTATION_ROLL_180_YAW_90

                                    // # also probe for external compasses
                                    // define HAL_PROBE_EXTERNAL_I2C_COMPASSES
//#define AP_COMPASS_HMC5843_ENABLED 1
//                 QMC5883L

//HMC5883L (I2C 0x?? is diff to Q5883L) ******USES HMC 5843 DRIVER



// MAG/COMPASS probing:
//#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_ICM20948, AP_Compass_AK09916::probe_ICM20948_I2C(0, ROTATION_NONE));

#define ALLOW_ARM_NO_COMPASS 1		

// turn off all the compasses by default.. 

//#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 1


// #define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))

//#define AP_COMPASS_AK8963_ENABLED TRUE
//phil on SPI instance nos = 0 (bus=0 ???)
//#define HAL_MAG_PROBEphil1  ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(0, ROTATION_NONE))

//#define AP_COMPASS_HMC5843_ENABLED 1
//#define HAL_MAG_PROBEphil2 ADD_BACKEND(DRIVER_HMC5843, AP_Compass_HMC5843::probe(GET_I2C_DEVICE(0, 0x1e), false, ROTATION_NONE))

//#define HAL_MAG_PROBE_LIST HAL_MAG_PROBEphil1;HAL_MAG_PROBEphil2
//#define HAL_MAG_PROBE_LIST HAL_MAG_PROBEphil1

//#define HAL_PROBE_EXTERNAL_I2C_COMPASSES 1
//#define AP_COMPASS_HMC5883_ENABLED 1

// we run the AK8963 only on the 2nd MPU9250, which leaves the
        // first MPU9250 to run without disturbance at high rate
//      ADD_BACKEND(DRIVER_AK8963, AP_Compass_AK8963::probe_mpu9250(1, ROTATION_YAW_270));

//#define HAL_MAG_PROBE_LIST ADD_BACKEND(DRIVER_, AP_Compass_AK09916::probe_ICM20948_I2C(0, ROTATION_NONE));







//See boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

//WIFI
//#define HAL_ESP32_WIFI 1  //1-TCP, 2-UDP, comment this line = without wifi
#define WIFI_SSID "ardupilot-esp32"
#define WIFI_PWD "ardupilot-esp32"


//ADC
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

//LED
                    //hwdat # Define a LED, mapping it to GPIO(0). LOW will illuminate the LED // PE12 FMU_LED_AMBER OUTPUT HIGH OPENDRAIN GPIO(0)
//phil remember S3 LED  different  from classic                     
#define DEFAULT_NTF_LED_TYPES Notify_LED_None

//SD CARD
// Do u want to use mmc or spi mode for the sd card, this is board specific,
// as mmc uses specific pins but is quicker,
// and spi is more flexible pinouts....
// dont forget vspi/hspi should be selected to NOT conflict with HAL_ESP32_SPI_BUSES

//#define HAL_ESP32_SDCARD //after enabled, uncomment one of below
//#define HAL_ESP32_SDMMC
//#define HAL_ESP32_SDSPI {.host=VSPI_HOST, .dma_ch=2, .mosi=GPIO_NUM_2, .miso=GPIO_NUM_15, .sclk=GPIO_NUM_26, .cs=GPIO_NUM_21}

#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 0

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

#define HAL_LOGGING_BACKENDS_DEFAULT 1

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
USART uses both data signals and clock for its functioning. While UART entails data signals only for its functioning.


# hw definition file for processing by chibios_hwdef.py
# for FMUv3 hardware (ie. for Pixhawk1, Pixhawk2 cube, XUAV2.1 etc)

# This hwdef.dat file contains a lot of comments so it can act as a
# reference for developers adding new boards.

# The hwdef.dat file defines all the hardware peripherals and pins for
# a port of ArduPilot to a board using the ChibiOS HAL. You should be
# able to write the hwdef.dat file for a new board with just the
# schematic for the board.

# This file is processed by chibios_hwdef.py to create hwdef.h for
# this board. You may find it useful to run chibios_hwdef.py manually
# when building this file for a new board. The resulting hwdef.h file
# is formatted to make it quite readable. It is strongly suggested
# that you read the resulting hwdef.h file when porting to a new board
# to make sure it has resulted in what you want.

# You should read this file in conjunction with the schematic for your
# board, the datasheet for the MCU for your board and the python
# tables file that we have extracted from the datasheet for your
# MCU. The python tables file is particularly important, so if you
# haven't seen it before go and look at it now. For the STM32F427 it
# it called STM32F427xx.py and it is in the hwdef/script/ directory
# inside the HAL_ChibiOS directory. That file tells you what each pin
# can do (the alternate functions table) and what DMA channels can be
# used for each peripheral type. The alternative functions table is
# particularly useful when doing a new hwdef.dat file as you can work
# out peripheral numbers given a port/pin name.

# We need to start off by saying what main CPU is on the board. There
# are two CPU identifiers that you need to specify. The first is the
# ChibiOS MCU type. So far we only support STM32F4xx for all STM32F4
# board types. In the future we will add F7 and other MCU types
# The second string needs to match the name of a config file in the
# libraries/AP_HAL_ChibiOS/hwdef/script directory. In this case we are
# using a F427 MCU, so we select STM32F427xx to match the
# STM32F427xx.py file in the script directory. If you are supporting a
# board type that doesn't have a python hardware database file yet
# then you will need to create one. There are scripts in the scripts
# directory to help with that by parsing the STM32 datasheets to
# extract the required DMA and alternate function tables.

# MCU class and specific type
MCU STM32F4xx STM32F427xx

# We set a specific HAL_BOARD_SUBTYPE, allowing for custom config in
# drivers. For this to be used the subtype needs to be added to
# AP_HAL/AP_HAL_Boards.h as well.
define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3

# Now we need to specify the APJ_BOARD_ID. This is the ID that the
# bootloader presents to GCS software so it knows if this firmware is
# suitable for the board. Please see
# https://github.com/ArduPilot/Bootloader/blob/master/hw_config.h for
# a list of current board IDs. If you add a new board type then please
# get it added to that repository so we don't get conflicts.

# Note that APJ is "ArduPilot JSON Firmware Format".

# board ID. See Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID TARGET_HW_CUBE_F4

# Now you need to say what crystal frequency you have for this
# board. All of the clocks are scaled against this. Typical values are
# 24000000 or 8000000.

# crystal frequency
OSCILLATOR_HZ 24000000

# On some boards you will need to also set the various PLL values. See
# the defaults in common/mcuconf.h, and use the define mechanism
# explained later in this file to override values suitable for your
# board. Refer to your MCU datasheet or examples from supported boards
# in ChibiOS for the right values.

# Now define the voltage the MCU runs at. This is needed for ChibiOS
# to set various internal driver limits. It is in 0.01 volt units.


# This is the STM32 timer that ChibiOS will use for the low level
# driver. This must be a 32 bit timer. We currently only support
# timers 2, 3, 4, 5 and 21. See hal_st_lld.c in ChibiOS for details.

# ChibiOS system timer
STM32_ST_USE_TIMER 5

# Now the size of flash in kilobytes, for creating the ld.script.

# flash size
FLASH_SIZE_KB 2048

# Now define which UART is used for printf(). We rarely use printf()
# in ChibiOS, so this is really only for debugging very early startup
# in drivers.

# Serial port for stdout. This is optional. If you leave it out then
# output from printf() lines will go to the ArduPilot console, which is the
# first UART in the SERIAL_ORDER list.  But note that some startup code
# runs before USB is set up. 
# The value for STDOUT_SERIAL is a serial device name, and must be for a 
# serial device for which pins are defined in this file. For example, SD7
# is for UART7 (SD7 == "serial device 7" in ChibiOS).
#STDOUT_SERIAL SD7
#STDOUT_BAUDRATE 57600

# Now the USB setup, if you have USB. All of these settings are
# option, and the ones below are the defaults. It ends up creating a
# USB ID on Linux like this:
# /dev/serial/by-id/usb-ArduPilot_fmuv3_3E0031000B51353233343932-if00
# If creating a board for a RTF vehicle you may wish to customise these.

# USB setup
USB_STRING_MANUFACTURER "ArduPilot"

# Now define the order that I2C buses are presented in the hal.i2c API
# in ArduPilot. For historical reasons inherited from HAL_PX4 the
# 'external' I2C bus should be bus 1 in hal.i2c, and internal I2C bus
# should be bus 0. On fmuv3 the STM32 I2C1 is our external bus and
# I2C2 is our internal bus, so we need to setup the order as I2C2
# followed by I2C1 in order to achieve the conventional order that
# drivers expect.

# order of I2C buses
I2C_ORDER I2C2 I2C1

# Now the serial ordering. These map to the SERIALn_ parameter numbers
# If you use a shorter list then HAL_Empty::UARTDriver
# objects are substituted for later UARTs, or you can leave a gap by
# listing one or more of the uarts as EMPTY.

# The normal usage of this ordering is:
# 1) SERIAL0: console (primary mavlink, usually USB)
# 2) SERIAL1: telem1
# 3) SERIAL2: telem2
# 4) SERIAL3: primary GPS
# 5) SERIAL4: GPS2
# 6) SERIAL5: extra UART (usually RTOS debug console)

# order of UARTs (and USB)
SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 UART7

# If the board has an IOMCU connected via a UART then this defines the
# UART to talk to that MCU. Leave it out for boards with no IOMCU.

# UART for IOMCU
IOMCU_UART USART6

# Now we start on the pin definitions. Every pin used by ArduPilot
# needs to be in this file. The pins in this file can be defined in any order.

# The format is P+port+pin. So PC4 is portC pin4.
# For every pin the second column is the label. If this is a
# peripheral that has an alternate function defined in the STM32
# datasheet then the label must be the name of that alternative
# function. The names are looked up in the python database for this
# MCU. Please see STM32F427xx.py for the F427 database. That database
# is used to automatically fill in the alternative function (and later
# for the DMA channels).

# The third column is the peripheral type. This must be one of the
# following: UARTn, USARTn, OTGn, SPIn, I2Cn, ADCn, TIMn, SWD, SDIO,
# INPUT, OUTPUT, CS.

# The fourth and later columns are for modifiers on the pin. The
# possible modifiers are:
# pin speed: SPEED_VERYLOW, SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH
# pullup: PULLUP, PULLDOWN, FLOATING
# out type: OPENDRAIN, PUSHPULL
# default value: LOW, HIGH

# Additionally, each class of pin peripheral can have extra modifiers
# suitable for that pin type. For example, for an OUTPUT you can map
# it to a GPIO number in hal.gpio using the GPIO(n) modifier. For ADC
# inputs you can apply a scaling factor (to bring it to unit volts)
# using the SCALE(x) modifier. See the examples below for more
# modifiers, or read the python code in chibios_hwdef.py.

# Now we define UART4 which is for the GPS. Be careful
# of the difference between USART and UART. Check the STM32F427xx.py
# if unsure which it is. For a UART we need to specify at least TX and
# RX pins.

# UART4 serial GPSa
PA0 UART4_TX UART4
PA1 UART4_RX UART4
#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
//#define HAL_INS_MPU9250_NAME "MPU9250"

# Now define the primary battery connectors. The labels we choose here
# are used to create defines for pinUSART uses both data signals and clock for its functioning. While UART entails data signals only for its functioning.s in the various drivers, so
# choose names that match existing board setups where possible. Here
# we define two pins PA2 and PA3 for voltage and current sensing, with
# a scale factor of 1.0 and connected on ADC1. The pin number this
# maps to in hal.adc is automatically determined using the datasheet
# tables in STM32F427xx.py.

PA2 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PA3 BATT_CURRENT_SENS ADC1 SCALE(1)

# Now the VDD sense pin. This is used to sense primary board voltage.
PA4 VDD_5V_SENS ADC1 SCALE(2)

# Now the first SPI bus. At minimum you need SCK, MISO and MOSI pin
definitions. You can add speed modifiers if you want them, otherwise
the defaults for the peripheral class are used.

PA5 SPI1_SCK SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# This defines an output pin which will default to output LOW. It is a
# pin that enables peripheral #define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
//#define HAL_INS_MPU9250_NAME "MPU9250"
power on this board.

PA8 nVDD_5V_PERIPH_EN OUTPUT LOW

# This is the pin that senses USB being connected. It is an input pin
# setup as OPENDRAIN.
PA9 VBUS INPUT OPENDRAIN

# This is a commented out pin for talking to the debug UART on the
# IOMCU, not used yet, but left as a comment (with a '#' in front) for
# future reference
# PA10 IO-debug-console

# Now we define the pins that USB is connected on.
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# These are the pins for SWD debugging with a STlinkv2 or black-magic probe.
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# This defines the PWM pin for the buzzer (if there is one). It is
# also mapped to a GPIO output so you can play with the buzzer via
# MAVLink relay commands if you want to.

# PWM output for buzzer
PA15 TIM2_CH1 TIM2 GPIO(77) ALARM

# This defines a couple of general purpose outputs, mapped to GPIO
# numbers 1 and 2 for users.
PB0 EXTERN_GPIO1 OUTPUT GPIO(1)
PB1 EXTERN_GPIO2 OUTPUT GPIO(2)

# This defines some input pins, currently unused.HAL_PROBE_EXTERNAL_I2C_COMPASSES
PB2 BOOT1 INPUT
PB3 FMU_SW0 INPUT

# This defines the pins for the 2nd CAN interface, if available.
PB6 CAN2_TX CAN2
PB12 CAN2_RX CAN2

# Now the first I2C bus. The pin speeds are automatically setup
# correctly, but can be overridden here if needed.
PB8 I2C1_SCL I2C1
PB9 I2C1_SDA I2C1

# the 2nd I2C bus
PB10 I2C2_SCL I2C2
PB11 I2C2_SDA I2C2

# the 2nd SPI bus
PB13 SPI2_SCK SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# This input pin is used to detect that power is valid on USB.
PC0 VBUS_nVALID INPUT PULLUP

# This defines the CS pin for the magnetometer and first IMU. Note
# that CS pins are software controlled, and are not tied to a particular
# SPI bus.
PC1 MAG_CS CS
PC2 MPU_CS CS

# This defines more ADC inputs.
PC3 AUX_POWER ADC1 SCALE(1)
PC4 AUX_ADC2 ADC1 SCALE(1)

# And the analog input for airspeed (rarely used these days).
PC5 PRESSURE_SENS ADC1 SCALE(2)

# This sets up the UART for talking to the IOMCU. Note that it is
# vital that this UART has DMA available. See the DMA settings below
# for more information.

#  USART6 to IO
PC6 USART6_TX USART6
PC7 USART6_RX USART6

# Now setup the pins for the microSD card, if available.
PC8 SDIO_D0 SDIO
PC9 SDIO_D1 SDIO
PC10 SDIO_D2 SDIO
PC11 SDIO_D3 SDIO
PC12 SDIO_CK SDIO
PD2 SDIO_CMD SDIO

# More CS pins for more sensors. The labels for all CS pins need to
# match the SPI device table later in this file.
PC13 GYRO_EXT_CS CS
PC14 BARO_EXT_CS CS
PC15 ACCEL_EXT_CS CS
PD7 BARO_CS CS
PE4 MPU_EXT_CS CS

# the first CAN bus
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1

# Another USART, this one for telem1. This one has RTS and CTS lines.
# USART2 serial2 telem1
PD3 USART2_CTS USART2
PD4 USART2_RTS USART2
PD5 USART2_TX USART2
PD6 USART2_RX USART2

# The telem2 USART, also with RTS/CTS available.
# USART3 serial3 telem2
PD8 USART3_TX USART3
PD9 USART3_RX USART3
PD11 USART3_CTS USART3
PD12 USART3_RTS USART3

# The CS pin for FRAM (ramtron). This one is marked as using
# SPEED_VERYLOW, which matches the HAL_PX4 setup.
PD10 FRAM_CS CS SPEED_VERYLOW

# Now we start defining some PWM pins. We also map these pins to GPIO
# values, so users can set BRD_PWM_COUNT to choose how many of the PWM
# outputs on the primary MCU are setup as PWM and how many as
# GPIOs. To match HAL_PX4 we number the GPIOs for the PWM outputs
# starting at 50.
PE14 TIM1_CH4 TIM1 PWM(1) GPIO(50)
PE13 TIM1_CH3 TIM1 PWM(2) GPIO(51)
PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52)
PE9  TIM1_CH1 TIM1 PWM(4) GPIO(53)
PD13 TIM4_CH2 TIM4 PWM(5) GPIO(54)
PD14 TIM4_CH3 TIM4 PWM(6) GPIO(55)

# This is the invensense data-ready pin. We don't use it in the
# default driver.
PD15 MPU_DRDY INPUT

# the 2nd GPS UART
# UART8 serial4 GPS2
PE0 UART8_RX UART8
PE1 UART8_TX UART8

# Now setup SPI bus4.
PE2 SPI4_SCK  SPI4
PE5 SPI4_MISO SPI4
PE6 SPI4_MOSI SPI4

# This is the pin to enable the sensors rail. It can be used to power
# cycle sensors to recover them in case there are problems with power on
# timing affecting sensor stability. We pull it high by default.
PE3 VDD_3V3_SENSORS_EN OUTPUT HIGH

# UART7 maps to SERIAL5.
PE7 UART7_RX UART7
PE8 UART7_TX UART7

# Define a LED, mapping it to GPIO(0). LOW will illuminate the LED
PE12 FMU_LED_AMBER OUTPUT HIGH OPENDRAIN GPIO(0)

# Power flag pins: these tell the MCU the status of the various power
# supplies that are available. The pin names need to exactly match the
# names used in AnalogIn.cpp. 
PB5 VDD_BRICK_nVALID INPUT PULLUP
PB7 VDD_BRICK2_nVALID INPUT PULLUP
PE10 VDD_5V_HIPOWER_nOC INPUT PULLUP
PE15 VDD_5V_PERIPH_nOC INPUT PULLUP

# Now the SPI device table. This table creates all accessible SPI
# devices, giving the name of the device (which is used by device
# drivers to open the device), plus which SPI bus it it on, what
# device ID will be used (which controls the IDs used in parameters
# such as COMPASS_DEV_ID, so we can detect when the list of devices
# changes between reboots for calibration purposes), the SPI mode to
# use, and the low and high speed settings for the device.

# You can define more SPI devices than you actually have, to allow for
# flexibility in board setup, and the driver code can probe to see
# which are responding.

# The DEVID values and device names are chosen to match the PX4 port
# of ArduPilot so users don't need to re-do their accel and compass
# calibrations when moving to ChibiOS.

SPIDEV ms5611         SPI1 DEVID3  BARO_CS      MODE3 20*MHZ 20*MHZ
SPIDEV ms5611_ext     SPI4 DEVID2  BARO_EXT_CS  MODE3 20*MHZ 20*MHZ
SPIDEV mpu6000        SPI1 DEVID4  MPU_CS       MODE3  2*MHZ  8*MHZ
SPIDEV icm20608-am    SPI1 DEVID2  ACCEL_EXT_CS MODE3  4*MHZ  8*MHZ
SPIDEV mpu9250        SPI1 DEVID4  MPU_CS       MODE3  4*MHZ  8*MHZ
SPIDEV mpu9250_ext    SPI4 DEVID1  MPU_EXT_CS   MODE3  4*MHZ  8*MHZ
SPIDEV icm20948       SPI1 DEVID4  MPU_CS       MODE3  4*MHZ  8*MHZ
SPIDEV icm20948_ext   SPI4 DEVID1  MPU_EXT_CS   MODE3  4*MHZ  8*MHZ
SPIDEV hmc5843        SPI1 DEVID5  MAG_CS       MODE3 11*MHZ 11*MHZ
SPIDEV lsm9ds0_g      SPI1 DEVID1  GYRO_EXT_CS  MODE3 11*MHZ 11*MHZ 
SPIDEV lsm9ds0_am     SPI1 DEVID2  ACCEL_EXT_CS MODE3 11*MHZ 11*MHZ 
SPIDEV lsm9ds0_ext_g  SPI4 DEVID4  GYRO_EXT_CS  MODE3 11*MHZ 11*MHZ 
SPIDEV lsm9ds0_ext_am SPI4 DEVID3  ACCEL_EXT_CS MODE3 11*MHZ 11*MHZ 
SPIDEV icm20602_ext   SPI4 DEVID4  GYRO_EXT_CS  MODE3  4*MHZ  8*MHZ
SPIDEV ramtron        SPI2 DEVID10 FRAM_CS      MODE3  8*MHZ  8*MHZ
SPIDEV external0m0    SPI4 DEVID5  MPU_EXT_CS   MODE0  2*MHZ  2*MHZ
SPIDEV external0m1    SPI4 DEVID5  MPU_EXT_CS   MODE1  2*MHZ  2*MHZ
SPIDEV external0m2    SPI4 DEVID5  MPU_EXT_CS   MODE2  2*MHZ  2*MHZ
SPIDEV external0m3    SPI4 DEVID5  MPU_EXT_CS   MODE3  2*MHZ  2*MHZ
SPIDEV pixartPC15     SPI4 DEVID13 ACCEL_EXT_CS MODE3  2*MHZ  2*MHZ

# Now some commented out SPI device names which can be used by
# developers to test that the clock calculations are right for a
# bus. This is used in conjunction with the mavproxy devop module.

# for SPI clock testing
#SPIDEV clock500 SPI4 DEVID5  MPU_EXT_CS   MODE0  500*KHZ 500*KHZ # gives 329KHz
#SPIDEV clock1   SPI4 DEVID5  MPU_EXT_CS   MODE0  1*MHZ 1*MHZ     # gives 657kHz
#SPIDEV clock2   SPI4 DEVID5  MPU_EXT_CS   MODE0  2*MHZ 2*MHZ     # gives 1.3MHz
#SPIDEV clock4   SPI4 DEVID5  MPU_EXT_CS   MODE0  4*MHZ 4*MHZ     # gives 2.6MHz
#SPIDEV clock8   SPI4 DEVID5  MPU_EXT_CS   MODE0  8*MHZ 8*MHZ     # gives 5.5MHz
#SPIDEV clock16  SPI4 DEVID5  MPU_EXT_CS   MODE0  16*MHZ 16*MHZ   # gives 10.6MHz

# This adds a C define which sets up the ArduPilot architecture
# define. Any line starting with 'define' is copied literally as
# a #define in the hwdef.h header.
define HAL_CHIBIOS_ARCH_FMUV3 1

# We need to tell HAL_ChibiOS/Storage.cpp how much storage is
# available (in bytes).
define HAL_STORAGE_SIZE 16384

# allow to have have a dedicated safety switch pin
define HAL_HAVE_SAFETY_SWITCH 1

# This enables the use of a ramtron device for storage, if one is
# found on SPI. You must have a ramtron entry in the SPI device table.

# Enable RAMTROM parameter storage.
define HAL_WITH_RAMTRON 1

# Setup for the possibility of an IMU heater since the pixhawk2 cube has
# an IMU heater.
define HAL_HAVE_IMU_HEATER 1

# Enable FAT filesystem support (needs a microSD defined via SDIO).
define HAL_OS_FATFS_IO 1

# Now setup the default battery pins driver analog pins and default
# scaling for the power brick.
define HAL_BATT_VOLT_PIN 2
define HAL_BATT_CURR_PIN 3
define HAL_BATT_VOLT_SCALE 10.1
define HAL_BATT_CURR_SCALE 17.0

# This defines the default max#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
//#define HAL_INS_MPU9250_NAME "MPU9250"
imum clock on I2C devices.
define HAL_I2C_MAX_CLOCK 100000

# We can't share the IO UART (USART6).
DMA_NOSHARE USART6_TX ADC1
DMA_PRIORITY USART6* SPI*

# List of files to put in ROMFS. For fmuv3 we need an IO firmware so
# we can automatically update the IOMCU firmware on boot. The format
# is "ROMFS ROMFS-filename source-filename". Paths are relative to the
# ardupilot root.
ROMFS io_firmware.bin Tools/IO_Firmware/iofirmware_lowpolh.bin

# for users running fmuv3 on their Solo:
define AP_NOTIFY_OREOLED_ENABLED (BOARD_FLASH_SIZE > 1024)
define HAL_SOLO_GIMBAL_ENABLED (HAL_MOUNT_ENABLED && BOARD_FLASH_SIZE > 1024)

undef AP_BATTERY_SMBUS_SOLO_ENABLED
define AP_BATTERY_SMBUS_SOLO_ENABLED (AP_BATTERY_SMBUS_ENABLED && BOARD_FLASH_SIZE > 1024)

# produce this error if we are on a 1M board
define BOARD_CHECK_F427_USE_1M "ERROR: 1M flash use fmuv2"

*/

/*
BUILD_OPTIONS = [
    Feature('AHRS', 'EKF3', 'HAL_NAVEKF3_AVAILABLE', 'Enable EKF3', 1, None),
    Feature('AHRS', 'EKF2', 'HAL_NAVEKF2_AVAILABLE', 'Enable EKF2', 0, None),
    Feature('AHRS', 'AHRS_EXT', 'HAL_EXTERNAL_AHRS_ENABLED', 'Enable External AHRS', 0, None),
    Feature('AHRS', 'MicroStrain5', 'AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED', 'Enable MICROSTRAIN 5-series external AHRS', 0, "AHRS_EXT"),  # noqa: E501
    Feature('AHRS', 'MicroStrain7', 'AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED', 'Enable MICROSTRAIN 7-series external AHRS', 0, "AHRS_EXT"),  # noqa: E501
    Feature('AHRS', 'AHRS_EXT_VECTORNAV', 'AP_EXTERNAL_AHRS_VECTORNAV_ENABLED', 'Enable VectorNav external AHRS', 0, "AHRS_EXT"),  # noqa
    Feature('AHRS', 'InertialLabs', 'AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED', 'Enable InertialLabs external AHRS', 0, "AHRS_EXT"),  # noqa
    Feature('AHRS', 'VISUALODOM', 'HAL_VISUALODOM_ENABLED', 'Enable Visual Odometry', 0, None),
    Feature('AHRS', 'EKF3_EXTNAV', 'EK3_FEATURE_EXTERNAL_NAV', 'Enable External navigation for EKF3', 0, 'EKF3'),
    Feature('AHRS', 'EKF3_WINDEST', 'EK3_FEATURE_DRAG_FUSION', 'Enable Wind estimation for EKF3', 0, 'EKF3'),
    Feature('AHRS', 'EKF3_OPTFLOW', 'EK3_FEATURE_OPTFLOW_FUSION', 'Enable OpticalFlow fusion for EKF3', 0, 'EKF3,OPTICALFLOW'),
    Feature('AHRS', 'BARO_WIND_COMP', 'HAL_BARO_WIND_COMP_ENABLED', 'Enable Baro wind compensation', 0, None),

    Feature('Safety', 'PARACHUTE', 'HAL_PARACHUTE_ENABLED', 'Enable Parachute', 0, None),
    Feature('Safety', 'FENCE', 'AP_FENCE_ENABLED', 'Enable Geofences', 2, None),
    Feature('Safety', 'RALLY', 'HAL_RALLY_ENABLED', 'Enable Rally points', 0, None),  # noqa
    Feature('Safety', 'AC_AVOID', 'AP_AVOIDANCE_ENABLED', 'Enable Object Avoidance', 0, 'FENCE'),
    Feature('Safety', 'AC_OAPATHPLANNER', 'AP_OAPATHPLANNER_ENABLED', 'Enable Object Avoidance Path Planner', 0, 'FENCE'),

    Feature('Battery', 'BATTERY_FUELFLOW', 'AP_BATTERY_FUELFLOW_ENABLED', 'Enable Fuel flow battery monitor', 0, None),
    Feature('Battery', 'BATTERY_FUELLEVEL_PWM', 'AP_BATTERY_FUELLEVEL_PWM_ENABLED', 'Enable PWM Fuel level battery monitor', 0, None),  # noqa: E501
    Feature('Battery', 'BATTERY_FUELLEVEL_ANALOG', 'AP_BATTERY_FUELLEVEL_ANALOG_ENABLED', 'Enable Analog Fuel level battry monitor', 0, None),  # noqa: E501
    Feature('Battery', 'BATTERY_SMBUS', 'AP_BATTERY_SMBUS_ENABLED', 'Enable SMBUS battery monitor', 0, None),
    Feature('Battery', 'BATTERY_INA2XX', 'AP_BATTERY_INA2XX_ENABLED', 'Enable INA2XX battery monitor', 0, None),
    Feature('Battery', 'BATTERY_INA3221', 'AP_BATTERY_INA3221_ENABLED', 'Enable INA3221 battery monitor', 0, None),
    Feature('Battery', 'BATTERY_SYNTHETIC_CURRENT', 'AP_BATTERY_SYNTHETIC_CURRENT_ENABLED', 'Enable Synthetic Current monitor', 0, None), # noqa: E501
    Feature('Battery', 'BATTERY_ESC_TELEM_OUT', 'AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED', 'Enable Ability to put battery monitor data into ESC telem stream', 0, None), # noqa: E501
    Feature('Battery', 'BATTERY_SUM', 'AP_BATTERY_SUM_ENABLED', 'Enable Synthetic sum-of-other-batteries backend', 0, None), # noqa: E501
    Feature('Battery', 'BATTERY_WATT_MAX', 'AP_BATTERY_WATT_MAX_ENABLED', 'Enable BATT_WATT_MAX parameter', 0, None), # noqa: E501


    Feature('Ident', 'ADSB', 'HAL_ADSB_ENABLED', 'Enable ADSB', 0, None),
    Feature('Ident', 'ADSB_SAGETECH', 'HAL_ADSB_SAGETECH_ENABLED', 'Enable Sagetech ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_SAGETECH_MXS', 'HAL_ADSB_SAGETECH_MXS_ENABLED', 'Enable Sagetech MXS ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_UAVIONIX', 'HAL_ADSB_UAVIONIX_MAVLINK_ENABLED', 'Enable UAvionix ADSB', 0, 'ADSB'),
    Feature('Ident', 'ADSB_UAVIONX_UCP', 'HAL_ADSB_UCP_ENABLED', 'Enable uAvionix UCP ADSB', 0 , 'ADSB'),
    Feature('Ident', 'AIS', 'AP_AIS_ENABLED', 'Enable AIS', 0, None),
    Feature('Ident', 'OpenDroneID', 'AP_OPENDRONEID_ENABLED', 'Enable OpenDroneID (Remote ID)', 0, None),

    Feature('Telemetry', 'CRSF', 'HAL_CRSF_TELEM_ENABLED', 'Enable CRSF telemetry', 0, 'FrSky SPort PassThrough,FrSky,FrSky SPort,RC_CRSF'),  # noqa
    Feature('Telemetry', 'CRSFText', 'HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED', 'Enable CRSF text param selection', 0, 'CRSF,OSD_PARAM,FrSky SPort PassThrough,FrSky,FrSky SPort'),  # NOQA: E501
    Feature('Telemetry', 'HOTT', 'HAL_HOTT_TELEM_ENABLED', 'Enable HOTT telemetry', 0, None),
    Feature('Telemetry', 'SPEKTRUM', 'HAL_SPEKTRUM_TELEM_ENABLED', 'Enable Spektrum telemetry', 0, None),
    Feature('Telemetry', 'LTM', 'AP_LTM_TELEM_ENABLED', 'Enable LTM telemetry', 0, None),
    Feature('Telemetry', 'AUX_FUNCTION_STRINGS', 'AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED', 'Enable Auxilliary function activation text messages', 0, None),  # noqa
    Feature('Telemetry', 'FrSky', 'AP_FRSKY_TELEM_ENABLED', 'Enable FrSky telemetry', 0, None),
    Feature('Telemetry', 'FrSky D', 'AP_FRSKY_D_TELEM_ENABLED', 'Enable FrSkyD telemetry', 0, 'FrSky'),
    Feature('Telemetry', 'FrSky SPort', 'AP_FRSKY_SPORT_TELEM_ENABLED', 'Enable FrSkySPort telemetry', 0, 'FrSky'),  # noqa
    Feature('Telemetry', 'FrSky SPort PassThrough', 'AP_FRSKY_SPORT_PASSTHROUGH_ENABLED', 'Enable FrSkySPort pass-through telemetry', 0, 'FrSky SPort,FrSky'),  # noqa
    Feature('Telemetry', 'Bidirectional FrSky Telemetry', 'HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL', 'Enable bidirectional FrSky telemetry', 0, 'FrSky SPort'),  # noqa
    Feature('Telemetry', 'GHST', 'AP_GHST_TELEM_ENABLED', 'Enable Ghost telemetry', 0, "RC_GHST"), # noqa
    Feature('Telemetry', 'i-BUS', 'AP_IBUS_TELEM_ENABLED', 'Enable i-BUS telemetry', 0, None),

    Feature('Notify', 'PLAY_TUNE', 'AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED', 'Enable MAVLink Play Tune command', 0, None),  # noqa
    Feature('Notify', 'TONEALARM', 'AP_NOTIFY_TONEALARM_ENABLED', 'Enable PWM tone alarm', 0, None),  # noqa
    Feature('Notify', 'LED_CONTROL', 'AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED', 'Enable MAVLink LED control', 0, None),  # noqa
    Feature('Notify', 'NOTIFY_NCP5623', 'AP_NOTIFY_NCP5623_ENABLED', 'Enable NCP5623 LED', 0, None),  # noqa
    # Feature('Notify', 'NOTIFY_PCA9685', 'AP_NOTIFY_PCA9685_ENABLED', 'Enable PCA9685 LED', 0, None),  # noqa  linux-only
    Feature('Notify', 'NOTIFY_PROFILED', 'AP_NOTIFY_PROFILED_ENABLED', 'Enable ProfiLED', 0, None),  # noqa
    Feature('Notify', 'DISPLAY', 'HAL_DISPLAY_ENABLED', 'Enable I2C Displays', 0, None),
    Feature('Notify', 'NOTIFY_PROFILED_SPI', 'AP_NOTIFY_PROFILED_SPI_ENABLED', 'Enable ProfiLED (SPI)', 0, None),  # noqa
    Feature('Notify', 'NOTIFY_NEOPIXEL', 'AP_NOTIFY_NEOPIXEL_ENABLED', 'Enable NeoPixel LED strings', 0, None),  # noqa

    Feature('MSP', 'MSP', 'HAL_MSP_ENABLED', 'Enable MSP telemetry and MSP OSD', 0, 'OSD'),
    Feature('MSP', 'MSP_SENSORS', 'HAL_MSP_SENSORS_ENABLED', 'Enable MSP sensors', 0, 'MSP_GPS,MSP_BARO,MSP_COMPASS,MSP_AIRSPEED,MSP,MSP_OPTICALFLOW,MSP_RANGEFINDER,OSD'),   # NOQA: E501
    Feature('MSP', 'MSP_GPS', 'HAL_MSP_GPS_ENABLED', 'Enable MSP GPS', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_COMPASS', 'AP_COMPASS_MSP_ENABLED', 'Enable MSP compass', 0, 'MSP,OSD'),
    Feature('MSP', 'MSP_OPTICALFLOW', 'HAL_MSP_OPTICALFLOW_ENABLED', 'Enable MSP OpticalFlow', 0, 'MSP,OSD,OPTICALFLOW'), # also OPTFLOW dep   # NOQA: E501
    Feature('MSP', 'MSP_RANGEFINDER', 'HAL_MSP_RANGEFINDER_ENABLED', 'Enable MSP rangefinder', 0, 'MSP,OSD,RANGEFINDER'),
    Feature('MSP', 'MSP_DISPLAYPORT', 'HAL_WITH_MSP_DISPLAYPORT', 'Enable MSP DisplayPort OSD (aka CANVAS MODE)', 0, 'MSP,OSD'),   # NOQA: E501

    Feature('ICE', 'ICE Engine', 'AP_ICENGINE_ENABLED', 'Enable Internal combustion engine support', 0, 'RPM'),
    Feature('ICE', 'EFI', 'HAL_EFI_ENABLED', 'Enable EFI monitoring', 0, None),
    Feature('ICE', 'EFI_MegaSquirt', 'AP_EFI_SERIAL_MS_ENABLED', 'Enable MegaSquirt EFI', 0, 'EFI'),
    Feature('ICE', 'EFI_Lutan', 'AP_EFI_SERIAL_LUTAN_ENABLED', 'Enable Lutan EFI', 0, 'EFI'),
    Feature('ICE', 'EFI_NMPWU', 'AP_EFI_NWPWU_ENABLED', 'Enable NMPMU EFI', 0, 'EFI'),
    Feature('ICE', 'EFI_CURRAWONGECU', 'AP_EFI_CURRAWONG_ECU_ENABLED', 'Enable Currawong ECU', 0, 'EFI'),
    Feature('ICE', 'EFI_HIRTH', 'AP_EFI_SERIAL_HIRTH_ENABLED', 'Enable Hirth ECU', 0, 'EFI'),
    Feature('ICE', 'EFI_DRONECAN', 'AP_EFI_DRONECAN_ENABLED', 'Enable DroneCAN EFI', 0, 'EFI,DroneCAN'),
    Feature('ICE', 'EFI_MAV', 'AP_EFI_MAV_ENABLED', 'Enable MAVLink EFI', 0, 'EFI'),

    Feature('Generator', 'GENERATOR', 'HAL_GENERATOR_ENABLED', 'Enable Generator', 0, None),
    Feature('Generator', 'GENERATOR_RICHENPOWER', 'AP_GENERATOR_RICHENPOWER_ENABLED', 'Enable Richenpower generator', 0, "GENERATOR"),  # noqa
    Feature('Generator', 'GENERATOR_IE2400', 'AP_GENERATOR_IE_2400_ENABLED', 'Enable IntelligentEnergy 2400', 0, "GENERATOR"),  # noqa
    Feature('Generator', 'GENERATOR_IE650', 'AP_GENERATOR_IE_650_800_ENABLED', 'Enable IntelligentEnergy 650 and 800', 0, "GENERATOR"),  # noqa

    Feature('OSD', 'OSD', 'OSD_ENABLED', 'Enable OSD', 0, None),
    Feature('OSD', 'PLUSCODE', 'HAL_PLUSCODE_ENABLE', 'Enable PlusCode', 0, 'OSD'),
    Feature('OSD', 'OSD_PARAM', 'OSD_PARAM_ENABLED', 'Enable OSD param', 0, None),
    Feature('OSD', 'OSD_SIDEBARS', 'HAL_OSD_SIDEBAR_ENABLE', 'Enable Scrolling sidebars', 0, 'OSD'),
    Feature('OSD', 'OSD_EXTENDED_LINK_STATS', 'AP_OSD_LINK_STATS_EXTENSIONS_ENABLED', 'Enable OSD panels with extended link stats data', 0, "OSD,RC_CRSF,MSP"),  # noqa

    Feature('VTX', 'VIDEO_TX', 'AP_VIDEOTX_ENABLED', 'Enable VideoTX control', 0, None),
    Feature('VTX', 'SMARTAUDIO', 'AP_SMARTAUDIO_ENABLED', 'Enable SmartAudio VTX contol', 0, "VIDEO_TX"),
    Feature('VTX', 'TRAMP', 'AP_TRAMP_ENABLED', 'Enable IRC Tramp VTX control', 0, "VIDEO_TX"),

    Feature('ESC', 'PICCOLOCAN', 'HAL_PICCOLO_CAN_ENABLE', 'Enable PiccoloCAN', 0, 'DroneCAN'),
    Feature('ESC', 'TORQEEDO', 'HAL_TORQEEDO_ENABLED', 'Enable Torqeedo motors', 0, None),

    Feature('ESC', 'ESC_EXTENDED_TELM', 'AP_EXTENDED_ESC_TELEM_ENABLED', 'Enable Extended ESC telemetry', 0, 'DroneCAN'),

    Feature('AP_Periph', 'LONG_TEXT', 'HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF', 'Enable extended length text strings', 0, None),

    Feature('Camera', 'Camera', 'AP_CAMERA_ENABLED', 'Enable Camera trigger', 0, None),
    Feature('Camera', 'Camera_MAVLink', 'AP_CAMERA_MAVLINK_ENABLED', 'Enable MAVLink camera ', 0, 'Camera'),
    Feature('Camera', 'Camera_MAVLinkCamV2', 'AP_CAMERA_MAVLINKCAMV2_ENABLED', 'Enable MAVLink CameraV2', 0, 'Camera'),
    Feature('Camera', 'Camera_Mount', 'AP_CAMERA_MOUNT_ENABLED', 'Enable Camera-in-Mount ', 0, 'Camera,MOUNT'),
    Feature('Camera', 'Camera_Relay', 'AP_CAMERA_RELAY_ENABLED', 'Enable Relay camera trigger', 0, 'Camera,RELAY'),
    Feature('Camera', 'Camera_Servo', 'AP_CAMERA_SERVO_ENABLED', 'Enable Servo camera trigger', 0, 'Camera'),
    Feature('Camera', 'Camera_Solo', 'AP_CAMERA_SOLOGIMBAL_ENABLED', 'Enable Solo gimbal', 0, 'Camera'),
    Feature('Camera', 'Camera_FOV_Status', 'AP_CAMERA_SEND_FOV_STATUS_ENABLED', 'Enable GCS camera FOV status', 0, 'Camera,MOUNT'),  # noqa: E501
    Feature('Camera', 'Camera_ThermalRange', 'AP_CAMERA_SEND_THERMAL_RANGE_ENABLED', 'Enable GCS camera thermal range', 0, 'Camera,MOUNT'),  # noqa: E501
    Feature('Camera', 'Camera_Info_From_Script', 'AP_CAMERA_INFO_FROM_SCRIPT_ENABLED', 'Enable Camera information messages via Lua script', 0, 'Camera,SCRIPTING'), # noqa

    Feature('Camera', 'RUNCAM', 'AP_CAMERA_RUNCAM_ENABLED', 'Enable RunCam control', 0, 'Camera'),

    Feature('Copter', 'MODE_ZIGZAG', 'MODE_ZIGZAG_ENABLED', 'Enable Mode ZigZag', 0, None),
    Feature('Copter', 'MODE_SYSTEMID', 'MODE_SYSTEMID_ENABLED', 'Enable Mode SystemID', 0, 'Logging'),
    Feature('Copter', 'MODE_SPORT', 'MODE_SPORT_ENABLED', 'Enable Mode Sport', 0, None),
    Feature('Copter', 'MODE_FOLLOW', 'MODE_FOLLOW_ENABLED', 'Enable Mode Follow', 0, 'AC_AVOID'),
    Feature('Copter', 'MODE_TURTLE', 'MODE_TURTLE_ENABLED', 'Enable Mode Turtle', 0, None),
    Feature('Copter', 'MODE_GUIDED_NOGPS', 'MODE_GUIDED_NOGPS_ENABLED', 'Enable Mode Guided NoGPS', 0, None),
    Feature('Copter', 'MODE_FLOWHOLD', 'MODE_FLOWHOLD_ENABLED', 'Enable Mode Flowhold', 0, "OPTICALFLOW"),
    Feature('Copter', 'MODE_FLIP', 'MODE_FLIP_ENABLED', 'Enable Mode Flip', 0, None),
    Feature('Copter', 'MODE_BRAKE', 'MODE_BRAKE_ENABLED', 'Enable Mode Brake', 0, None),
    Feature('Copter', 'COPTER_ADVANCED_FAILSAFE', 'AP_COPTER_ADVANCED_FAILSAFE_ENABLED', 'Enable Advanced Failsafe', 0, "ADVANCED_FAILSAFE"),  # NOQA: 501
    Feature('Copter', 'COPTER_AHRS_AUTO_TRIM', 'AP_COPTER_AHRS_AUTO_TRIM_ENABLED', 'Enable Copter AHRS AutoTrim', 0, None),  # noqa

    Feature('Rover', 'ROVER_ADVANCED_FAILSAFE', 'AP_ROVER_ADVANCED_FAILSAFE_ENABLED', 'Enable Advanced Failsafe', 0, "ADVANCED_FAILSAFE"),  # NOQA: 501

    Feature('Mission', 'MISSION_NAV_PAYLOAD_PLACE', 'AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED', 'Enable NAV_PAYLOAD_PLACE', 0, None),  # noqa
    Feature('Copter', 'AC_PAYLOAD_PLACE_ENABLED', 'AC_PAYLOAD_PLACE_ENABLED', 'Enable Copter Payload Place', 0, 'MISSION_NAV_PAYLOAD_PLACE'),  # noqa

    Feature('Compass', 'AK09916', 'AP_COMPASS_AK09916_ENABLED', 'Enable AK09916 compasses', 1, None),
    Feature('Compass', 'AK8963', 'AP_COMPASS_AK8963_ENABLED', 'Enable AK8963 compasses', 1, None),
    Feature('Compass', 'BMM150', 'AP_COMPASS_BMM150_ENABLED', 'Enable BMM150 compasses', 1, None),
    Feature('Compass', 'BMM350', 'AP_COMPASS_BMM350_ENABLED', 'Enable BMM350 compasses', 1, None),
    Feature('Compass', 'EXTERNALAHRS_COMPASS', 'AP_COMPASS_EXTERNALAHRS_ENABLED', 'Enable ExternalAHRS compasses', 0, "AHRS_EXT"),  # noqa
    Feature('Compass', 'HMC5843', 'AP_COMPASS_HMC5843_ENABLED', 'Enable HMC5843 compasses', 1, None),
    Feature('Compass', 'ICM20948', 'AP_COMPASS_ICM20948_ENABLED', 'Enable AK09916 on ICM20948 compasses', 1, "AK09916"),
    Feature('Compass', 'IST8308', 'AP_COMPASS_IST8308_ENABLED', 'Enable IST8308 compasses', 1, None),
    Feature('Compass', 'IIS2MDC', 'AP_COMPASS_IIS2MDC_ENABLED', 'Enable IIS2MDC compasses', 0, None),
    Feature('Compass', 'IST8310', 'AP_COMPASS_IST8310_ENABLED', 'Enable IST8310 compasses', 1, None),
    Feature('Compass', 'LIS3MDL', 'AP_COMPASS_LIS3MDL_ENABLED', 'Enable LIS3MDL compasses', 1, None),
    Feature('Compass', 'LSM303D', 'AP_COMPASS_LSM303D_ENABLED', 'Enable LSM303D compasses', 1, None),
    Feature('Compass', 'LSM9DS1', 'AP_COMPASS_LSM9DS1_ENABLED', 'Enable LSM9DS1 compasses', 1, None),
    Feature('Compass', 'MAG3110', 'AP_COMPASS_MAG3110_ENABLED', 'Enable MAG3110 compasses', 1, None),
    Feature('Compass', 'MMC3416', 'AP_COMPASS_MMC3416_ENABLED', 'Enable MMC3416 compasses', 1, None),
    Feature('Compass', 'MMC5XX3', 'AP_COMPASS_MMC5XX3_ENABLED', 'Enable MMC5XX3 compasses', 1, None),
    Feature('Compass', 'QMC5883L', 'AP_COMPASS_QMC5883L_ENABLED', 'Enable QMC5883L compasses', 1, None),
    Feature('Compass', 'RM3100', 'AP_COMPASS_RM3100_ENABLED', 'Enable RM3100 compasses', 1, None),
    Feature('Compass', 'DRONECAN_COMPASS', 'AP_COMPASS_DRONECAN_ENABLED', 'Enable DroneCAN compasses', 0, "DroneCAN"),
    Feature('Compass', 'DRONECAN_COMPASS_HIRES', 'AP_COMPASS_DRONECAN_HIRES_ENABLED', 'Enable DroneCAN HiRes compasses for survey logging', 0, "DroneCAN,DRONECAN_COMPASS"), # noqa
    Feature('Compass', 'FixedYawCal', 'AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED', 'Enable Fixed-Yaw Compass Calibration', 1, None),  # noqa
    Feature('Compass', 'CompassLearn', 'COMPASS_LEARN_ENABLED', 'Enable In-Flight compass learning', 1, "FixedYawCal"),

    Feature('Gimbal', 'MOUNT', 'HAL_MOUNT_ENABLED', 'Enable Camera Mounts', 0, None),
    Feature('Gimbal', 'ALEXMOS', 'HAL_MOUNT_ALEXMOS_ENABLED', 'Enable Alexmos gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'CADDX', 'HAL_MOUNT_CADDX_ENABLED', 'Enable CADDX gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'GREMSY', 'HAL_MOUNT_GREMSY_ENABLED', 'Enable Gremsy gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SERVO', 'HAL_MOUNT_SERVO_ENABLED', 'Enable Servo gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SIYI', 'HAL_MOUNT_SIYI_ENABLED', 'Enable Siyi gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'SOLOGIMBAL', 'HAL_SOLO_GIMBAL_ENABLED', 'Enable Solo gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'STORM32_MAVLINK', 'HAL_MOUNT_STORM32MAVLINK_ENABLED', 'Enable SToRM32 MAVLink gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'STORM32_SERIAL', 'HAL_MOUNT_STORM32SERIAL_ENABLED', 'Enable SToRM32 Serial gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'TOPOTEK', 'HAL_MOUNT_TOPOTEK_ENABLED', 'Enable Topotek gimbal', 0, "MOUNT"),
    Feature('Gimbal', 'XACTI', 'HAL_MOUNT_XACTI_ENABLED', 'Enable Xacti gimbal', 0, "MOUNT,DroneCAN"),
    Feature('Gimbal', 'VIEWPRO', 'HAL_MOUNT_VIEWPRO_ENABLED', 'Enable Viewpro gimbal', 0, "MOUNT"),

    Feature('VTOL Frame', 'QUAD', 'AP_MOTORS_FRAME_QUAD_ENABLED', 'QUADS(BI,TRI also)', 1, None),
    Feature('VTOL Frame', 'HEXA', 'AP_MOTORS_FRAME_HEXA_ENABLED', 'HEXA', 0, None),
    Feature('VTOL Frame', 'OCTA', 'AP_MOTORS_FRAME_OCTA_ENABLED', 'OCTA', 0, None),
    Feature('VTOL Frame', 'DECA', 'AP_MOTORS_FRAME_DECA_ENABLED', 'DECA', 0, None),
    Feature('VTOL Frame', 'DODECAHEXA', 'AP_MOTORS_FRAME_DODECAHEXA_ENABLED', 'DODECAHEXA', 0, None),
    Feature('VTOL Frame', 'Y6', 'AP_MOTORS_FRAME_Y6_ENABLED', 'Y6', 0, None),
    Feature('VTOL Frame', 'OCTAQUAD', 'AP_MOTORS_FRAME_OCTAQUAD_ENABLED', 'OCTAQUAD', 0, None),

    Feature('Payload', 'GRIPPER', 'AP_GRIPPER_ENABLED', 'Enable Gripper', 0, None),
    Feature('Payload', 'SPRAYER', 'HAL_SPRAYER_ENABLED', 'Enable Sprayer', 0, None),
    Feature('Payload', 'LANDING_GEAR', 'AP_LANDINGGEAR_ENABLED', 'Enable Landing Gear', 0, None),
    Feature('Payload', 'WINCH', 'AP_WINCH_ENABLED', 'Enable Winch', 0, None),
    Feature('Payload', 'WINCH_DAIWA', 'AP_WINCH_DAIWA_ENABLED', 'Enable DAIWA Winch', 0, 'WINCH'),
    Feature('Payload', 'WINCH_PWM', 'AP_WINCH_PWM_ENABLED', 'Enable PWM Winch', 0, 'WINCH'),

    Feature('Payload', 'RELAY', 'AP_RELAY_ENABLED', 'Enable Relays', 0, None),
    Feature('Payload', 'SERVORELAY_EVENTS', 'AP_SERVORELAYEVENTS_ENABLED', 'Enable Servo/Relay Event', 0, None),

    Feature('Plane', 'ADVANCED_FAILSAFE', 'AP_ADVANCEDFAILSAFE_ENABLED', 'Enable Advanced Failsafe', 0, None),
    Feature('Plane', 'QUADPLANE', 'HAL_QUADPLANE_ENABLED', 'Enable QuadPlane', 0, None),
    Feature('Plane', 'SOARING', 'HAL_SOARING_ENABLED', 'Enable Soaring', 0, None),
    Feature('Plane', 'DEEPSTALL', 'HAL_LANDING_DEEPSTALL_ENABLED', 'Enable Deepstall landing', 0, None),
    Feature('Plane', 'QAUTOTUNE', 'QAUTOTUNE_ENABLED', 'Enable QuadPlane AUTOTUNE', 0, "QUADPLANE"),
    Feature('Plane', 'PLANE_BLACKBOX', 'AP_PLANE_BLACKBOX_LOGGING', 'Enable Blackbox logging', 0, None),
    Feature('Plane', 'AP_TX_TUNING', 'AP_TUNING_ENABLED', 'Enable TX-based tuning parameter adjustments', 0, None),
    Feature('Plane', 'PLANE_GUIDED_SLEW', 'AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED', 'Enable Offboard-guided slew commands', 0, None),  # noqa:401
    Feature('Plane', 'PLANE_GLIDER_PULLUP', 'AP_PLANE_GLIDER_PULLUP_ENABLED', 'Enable Glider pullup support', 0, None),
    Feature('Plane', 'QUICKTUNE', 'AP_QUICKTUNE_ENABLED', 'Enable VTOL quicktune', 0, None),
    Feature('Plane', 'AUTOLAND_MODE', 'MODE_AUTOLAND_ENABLED', 'Enable Fixed Wing Autolanding mode', 0, None),

    Feature('RC', 'RC_Protocol', 'AP_RCPROTOCOL_ENABLED', "Enable Serial RC Protocols", 0, None),   # NOQA: E501
    Feature('RC', 'RC_CRSF', 'AP_RCPROTOCOL_CRSF_ENABLED', "Enable CRSF", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_IBUS', 'AP_RCPROTOCOL_IBUS_ENABLED', "Enable IBus", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SBUS', 'AP_RCPROTOCOL_SBUS_ENABLED', "Enable SBUS", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_PPMSUM', 'AP_RCPROTOCOL_PPMSUM_ENABLED', "Enable PPMSum", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SRXL', 'AP_RCPROTOCOL_SRXL_ENABLED', "Enable SRXL", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SRXL2', 'AP_RCPROTOCOL_SRXL2_ENABLED', "Enable SRXL2", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_ST24', 'AP_RCPROTOCOL_ST24_ENABLED', "Enable ST24", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_SUMD', 'AP_RCPROTOCOL_SUMD_ENABLED', "Enable SUMD", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_GHST', 'AP_RCPROTOCOL_GHST_ENABLED', "Enable Ghost", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RC_MAVLINK_RADIO', 'AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED', "Enable MAVLink", 0, "RC_Protocol"),   # NOQA: E501
    Feature('RC', 'RSSI', 'AP_RSSI_ENABLED', 'RSSI', 0, None),

    Feature('Rangefinder', 'RANGEFINDER', 'AP_RANGEFINDER_ENABLED', "Enable Rangefinders", 0, None),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_ANALOG', 'AP_RANGEFINDER_ANALOG_ENABLED', "Enable Rangefinder - Analog", 0, "RANGEFINDER"),   # NOQA: E501
    # Feature('Rangefinder', 'RANGEFINDER_BBB_PRU', 'AP_RANGEFINDER_BBB_PRU_ENABLED', "Enable Rangefinder - BBB PRU", 0, "RANGEFINDER"),   # NOQA: E501
    # Feature('Rangefinder', 'RANGEFINDER_BEBOP', 'AP_RANGEFINDER_BEBOP_ENABLED', "Enable Rangefinder - Bebop", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_CAN', 'AP_RANGEFINDER_BENEWAKE_CAN_ENABLED', "Enable Rangefinder - Benewake (CAN)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_TF02', 'AP_RANGEFINDER_BENEWAKE_TF02_ENABLED', "Enable Rangefinder - Benewake -TF02", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BENEWAKE_TF03', 'AP_RANGEFINDER_BENEWAKE_TF03_ENABLED', "Enable Rangefinder - Benewake - TF03", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RFND_BENEWAKE_TFMINI', 'AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED', "Enable Rangefinder - Benewake - TFMini", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RFND_BENEWAKE_TFMINIPLUS', 'AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED', "Enable Rangefinder - Benewake - TFMiniPlus", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_BLPING', 'AP_RANGEFINDER_BLPING_ENABLED', "Enable Rangefinder - BLPing", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_GYUS42V2', 'AP_RANGEFINDER_GYUS42V2_ENABLED', "Enable Rangefinder - GYUS42V2", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_HC_SR04', 'AP_RANGEFINDER_HC_SR04_ENABLED', "Enable Rangefinder - HC_SR04", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_JRE_SERIAL', 'AP_RANGEFINDER_JRE_SERIAL_ENABLED', "Enable Rangefinder - JRE_SERIAL", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LANBAO', 'AP_RANGEFINDER_LANBAO_ENABLED', "Enable Rangefinder - Lanbao", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LEDDARONE', 'AP_RANGEFINDER_LEDDARONE_ENABLED', "Enable Rangefinder - LeddarOne", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LEDDARVU8', 'AP_RANGEFINDER_LEDDARVU8_ENABLED', "Enable Rangefinder - LeddarVU8", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LIGHTWARE_SERIAL', 'AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED', "Enable Rangefinder - Lightware (serial)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LUA', 'AP_RANGEFINDER_LUA_ENABLED', "Enable Rangefinder - Lua Scripting", 0, "RANGEFINDER,SCRIPTING"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_LWI2C', 'AP_RANGEFINDER_LWI2C_ENABLED', "Enable Rangefinder - Lightware (i2c)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAVLINK', 'AP_RANGEFINDER_MAVLINK_ENABLED', "Enable Rangefinder - MAVLink", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAXBOTIX_SERIAL', 'AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED', "Enable Rangefinder - MaxBotix (serial)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_MAXSONARI2CXL', 'AP_RANGEFINDER_MAXSONARI2CXL_ENABLED', "Enable Rangefinder - MaxSonarI2CXL", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_NMEA', 'AP_RANGEFINDER_NMEA_ENABLED', "Enable Rangefinder - NMEA", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_NOOPLOOP', 'AP_RANGEFINDER_NOOPLOOP_ENABLED', "Enable Rangefinder - Nooploop TOF P/F", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_NRA24_CAN', 'AP_RANGEFINDER_NRA24_CAN_ENABLED', "Enable Rangefinder - NRA24 CAN", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_PULSEDLIGHTLRF', 'AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED', "Enable Rangefinder - PulsedLightLRF", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_PWM', 'AP_RANGEFINDER_PWM_ENABLED', "Enable Rangefinder - PWM", 0, "RANGEFINDER"),   # NOQA: E501
    # Feature('Rangefinder', 'RANGEFINDER_SIM', 'AP_RANGEFINDER_SIM_ENABLED', "Enable Rangefinder - SIM", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TOFSF_I2C', 'AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED', "Enable Rangefinder - ToFSense-F I2C", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TOFSP_CAN', 'AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED', "Enable Rangefinder - ToFSense-P CAN", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TRI2C', 'AP_RANGEFINDER_TRI2C_ENABLED', "Enable Rangefinder - TeraRangerI2C", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_TR_SERIAL', 'AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED', "Enable Rangefinder - TeraRanger Serial", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_DRONECAN', 'AP_RANGEFINDER_DRONECAN_ENABLED', "Enable Rangefinder - DroneCAN", 0, "RANGEFINDER,DroneCAN"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_USD1_CAN', 'AP_RANGEFINDER_USD1_CAN_ENABLED', "Enable Rangefinder - USD1 (CAN)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_USD1_SERIAL', 'AP_RANGEFINDER_USD1_SERIAL_ENABLED', "Enable Rangefinder - USD1 (SERIAL)", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_VL53L0X', 'AP_RANGEFINDER_VL53L0X_ENABLED', "Enable Rangefinder - VL53L0X", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_VL53L1X', 'AP_RANGEFINDER_VL53L1X_ENABLED', "Enable Rangefinder - VL53L1X", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_WASP', 'AP_RANGEFINDER_WASP_ENABLED', "Enable Rangefinder - Wasp", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_RDS02UF', 'AP_RANGEFINDER_RDS02UF_ENABLED', "Enable Rangefinder - RDS02UF", 0, "RANGEFINDER"),   # NOQA: E501
    Feature('Rangefinder', 'RANGEFINDER_HEXSOONRADAR', 'AP_RANGEFINDER_HEXSOONRADAR_ENABLED', "Enable Rangefinder - Hexsoon Radar", 0, "RANGEFINDER"),   # NOQA: E501

    Feature('Sensors', 'OPTICALFLOW', 'AP_OPTICALFLOW_ENABLED', 'Enable Optical Flow', 0, None),
    Feature('Sensors', 'OPTICALFLOW_CXOF', 'AP_OPTICALFLOW_CXOF_ENABLED', 'Enable Optical flow CXOF Sensor', 0, "OPTICALFLOW"),
    Feature('Sensors', 'OPTICALFLOW_HEREFLOW', 'AP_OPTICALFLOW_HEREFLOW_ENABLED', 'Enable Optical flow HereFlow Sensor', 0, "OPTICALFLOW,DroneCAN"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_MAV', 'AP_OPTICALFLOW_MAV_ENABLED', 'Enable Optical flow MAVLink Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_ONBOARD', 'AP_OPTICALFLOW_ONBOARD_ENABLED', 'Enable Optical flow ONBOARD Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_PX4FLOW', 'AP_OPTICALFLOW_PX4FLOW_ENABLED', 'Enable Optical flow PX4FLOW Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_PIXART', 'AP_OPTICALFLOW_PIXART_ENABLED', 'Enable Optical flow PIXART Sensor', 0, "OPTICALFLOW"),   # NOQA: E501
    Feature('Sensors', 'OPTICALFLOW_UPFLOW', 'AP_OPTICALFLOW_UPFLOW_ENABLED', 'Enable Optical flow UPFLOW Sensor', 0, "OPTICALFLOW"),   # NOQA: E501

    Feature('Proximity', 'PROXIMITY', 'HAL_PROXIMITY_ENABLED', 'Enable Proximity', 0, None),
    Feature('Proximity', 'PROXIMITY_CYGBOT', 'AP_PROXIMITY_CYGBOT_ENABLED', 'Enable Cygbot D1 Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_DRONECAN', 'AP_PROXIMITY_DRONECAN_ENABLED', 'Enable DroneCAN Proximity Sensors', 0, "PROXIMITY,DroneCAN"),  # noqa
    Feature('Proximity', 'PROXIMITY_LIGHTWARE_SF40C', 'AP_PROXIMITY_LIGHTWARE_SF40C_ENABLED', 'Enable LightWare SF40C Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_LIGHTWARE_SF45B', 'AP_PROXIMITY_LIGHTWARE_SF45B_ENABLED', 'Enable LightWare SF45B Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_MAV', 'AP_PROXIMITY_MAV_ENABLED', 'Enable MAVLink Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_RANGEFINDER', 'AP_PROXIMITY_RANGEFINDER_ENABLED', 'Use RangeFinders as proximity sensors', 0, "PROXIMITY,RANGEFINDER"),  # noqa
    Feature('Proximity', 'PROXIMITY_RPLIDARA2', 'AP_PROXIMITY_RPLIDARA2_ENABLED', 'Enable RPLidarA2 Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_TERRARANGERTOWER', 'AP_PROXIMITY_TERARANGERTOWER_ENABLED', 'Enable TerraRangerTower Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_TERRARANGERTOWEREVO', 'AP_PROXIMITY_TERARANGERTOWEREVO_ENABLED', 'Enable TerraRangerTower Evo Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_MR72_ENABLED', 'AP_PROXIMITY_MR72_ENABLED', 'Enable NanoRadar MR72 Proximity Sensors', 0, "PROXIMITY"),  # noqa
    Feature('Proximity', 'PROXIMITY_HEXSOONRADAR_ENABLED', 'AP_PROXIMITY_HEXSOONRADAR_ENABLED', 'Enable Hexsoon Radar Proximity Sensors', 0, "PROXIMITY"),  # noqa

    Feature('Baro', 'BMP085', 'AP_BARO_BMP085_ENABLED', 'Enable BMP085 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP280', 'AP_BARO_BMP280_ENABLED', 'Enable BMP280 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP388', 'AP_BARO_BMP388_ENABLED', 'Enable BMP388 Barometric Sensor', 1, None),
    Feature('Baro', 'BMP581', 'AP_BARO_BMP581_ENABLED', 'Enable BMP581 Barometric Sensor', 1, None),
    Feature('Baro', 'DPS280', 'AP_BARO_DPS280_ENABLED', 'Enable DPS280/DPS310 Barometric Sensor', 1, None),
    # Feature('Baro', 'DUMMY', 'AP_BARO_DUMMY_ENABLED', 'Enable DUMMY Barometric Sensor', 0, None),
    Feature('Baro', 'EXTERNALAHRS', 'AP_BARO_EXTERNALAHRS_ENABLED', 'Enable EXTERNALAHRS Barometric Sensor', 0, 'AHRS_EXT'),
    Feature('Baro', 'FBM320', 'AP_BARO_FBM320_ENABLED', 'Enable FBM320 Barometric Sensor', 1, None),
    # Feature('Baro', 'ICM20789', 'AP_BARO_ICM20789_ENABLED', 'Enable ICM20789 Barometric Sensor', 1, None),
    Feature('Baro', 'KELLERLD', 'AP_BARO_KELLERLD_ENABLED', 'Enable KELLERLD Barometric Sensor', 1, None),
    Feature('Baro', 'LPS2XH', 'AP_BARO_LPS2XH_ENABLED', 'Enable LPS2XH Barometric Sensor', 1, None),
    Feature('Baro', 'MS56XX', 'AP_BARO_MS56XX_ENABLED', 'Enable MS56XX Barometric Sensor', 1, None),
    Feature('Baro', 'MSP_BARO', 'AP_BARO_MSP_ENABLED', 'Enable MSP Barometric Sensor', 0, 'MSP'),
    Feature('Baro', 'SPL06', 'AP_BARO_SPL06_ENABLED', 'Enable SPL06 Barometric Sensor', 1, None),
    Feature('Baro', 'DRONECAN_BARO', 'AP_BARO_DRONECAN_ENABLED', 'Enable DroneCAN Barometric Sensor', 0, "DroneCAN"),
    # Feature('Baro', 'ICP101XX', 'AP_BARO_ICP101XX_ENABLED', 'Enable ICP101XX Barometric Sensor', 0, None),
    # Feature('Baro', 'ICP201XX', 'AP_BARO_ICP201XX_ENABLED', 'Enable ICP201XX Barometric Sensor', 0, None),
    Feature('Baro', 'BARO_TEMPCAL', 'AP_TEMPCALIBRATION_ENABLED', 'Enable Baro Temperature Calibration', 0, None),
    Feature('Baro', 'BARO_PROBEXT', 'AP_BARO_PROBE_EXTERNAL_I2C_BUSES', 'Enable Probing of External i2c buses', 0, None),

    Feature('Sensors', 'RPM', 'AP_RPM_ENABLED', 'Enable RPM sensors', 0, None),
    Feature('Sensors', 'RPM_EFI', 'AP_RPM_EFI_ENABLED', 'Enable RPM EFI sensors', 0, 'RPM,EFI'),
    Feature('Sensors', 'RPM_ESC_TELEM', 'AP_RPM_ESC_TELEM_ENABLED', 'Enable RPM ESC Telemetry sensors', 0, 'RPM'),
    Feature('Sensors', 'RPM_HARMONIC_NOTCH', 'AP_RPM_HARMONICNOTCH_ENABLED', 'Enable RPM Harmonic Notch sensors', 0, 'RPM,HarmonicNotches'),  # noqa
    Feature('Sensors', 'RPM_PIN', 'AP_RPM_PIN_ENABLED', 'Enable RPM Pin-based sensors', 0, 'RPM'),
    Feature('Sensors', 'RPM_GENERATOR', 'AP_RPM_GENERATOR_ENABLED', 'Enable Generator RPM sensors', 0, 'RPM,GENERATOR'),
    Feature('Sensors', 'RPM_DRONECAN', 'AP_RPM_DRONECAN_ENABLED', 'Enable DroneCAN-based RPM sensors', 0, 'RPM,GENERATOR,DroneCAN'),  # noqa

    Feature('Sensors', 'TEMP', 'AP_TEMPERATURE_SENSOR_ENABLED', 'Enable Temperature Sensors', 0, None),
    Feature('Sensors', 'TEMP_TSYS01', 'AP_TEMPERATURE_SENSOR_TSYS01_ENABLED', 'Enable Temp Sensor - TSYS01', 0, "TEMP"),
    Feature('Sensors', 'TEMP_MCP9600', 'AP_TEMPERATURE_SENSOR_MCP9600_ENABLED', 'Enable Temp Sensor - MCP9600', 0, "TEMP"),
    Feature('Sensors', 'TEMP_TSYS03', 'AP_TEMPERATURE_SENSOR_TSYS03_ENABLED', 'Enable Temp Sensor - TSYS03', 0, "TEMP"),
    Feature('Sensors', 'TEMP_MLX90614', 'AP_TEMPERATURE_SENSOR_MLX90614_ENABLED', 'Enable Temp Sensor - MLX90614', 0, "TEMP"),
    Feature('Sensors', 'TEMP_SHT3X', 'AP_TEMPERATURE_SENSOR_SHT3X_ENABLED', 'Enable Temp Sensor - SHT3x', 0, "TEMP"),

    Feature('Sensors', 'AIRSPEED', 'AP_AIRSPEED_ENABLED', 'Enable Airspeed Sensors', 1, None),    # Default to enabled to not annoy Plane users   # NOQA: E501
    Feature('Sensors', 'BEACON', 'AP_BEACON_ENABLED', 'Enable Beacon', 0, None),
    Feature('Sensors', 'GPS_MOVING_BASELINE', 'GPS_MOVING_BASELINE', 'Enable GPS Moving Baseline', 0, None),
    Feature('Sensors', 'IMU_ON_UART', 'AP_SERIALMANAGER_IMUOUT_ENABLED', 'Enable Send raw IMU data on a serial port', 0, None), # NOQA: E501

    Feature('IMU', 'TEMPCAL', 'HAL_INS_TEMPERATURE_CAL_ENABLE', 'Enable IMU Temperature calibration', 0, None),
    Feature('IMU', 'HarmonicNotches', 'AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED', 'Enable InertialSensor harmonic notch filters', 0, None),  # noqa
    Feature('IMU', 'BatchSampler', 'AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED', 'Enable Batch sampler', 0, None),  # noqa

    Feature('Other', 'RateLoopThread', 'AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED', 'Enable Rate Loop Thread', 0, 'HarmonicNotches'),  # noqa
    Feature('Other', 'GyroFFT', 'HAL_GYROFFT_ENABLED', 'Enable In-Flight gyro FFT calculations', 0, None),
    Feature('Other', 'NMEA_OUTPUT', 'HAL_NMEA_OUTPUT_ENABLED', 'Enable NMEA output', 0, None),
    Feature('Other', 'SDCARD_FORMATTING', 'AP_FILESYSTEM_FORMAT_ENABLED', 'Enable Formatting of microSD cards', 0, None),
    Feature('Other', 'BOOTLOADER_FLASHING', 'AP_BOOTLOADER_FLASHING_ENABLED', 'Enable Bootloader flashing', 0, "FILESYSTEM_ROMFS"),  # noqa
    Feature('Other', 'SCRIPTING', 'AP_SCRIPTING_ENABLED', 'Enable Lua Scripting', 0, None),
    Feature('Other', 'SERIALDEVICE_REGISTER', 'AP_SERIALMANAGER_REGISTER_ENABLED', 'Enable Serial device registration', 0, None), # noqa
    Feature('Other', 'SCRIPTING_SERIALDEVICE', 'AP_SCRIPTING_SERIALDEVICE_ENABLED', 'Enable Lua serial device simulation', 0, "SCRIPTING,SERIALDEVICE_REGISTER"), # noqa
    Feature('Other', 'SLCAN', 'AP_CAN_SLCAN_ENABLED', 'Enable SLCAN serial protocol', 0, None),
    Feature('Other', 'SDCARD_MISSION', 'AP_SDCARD_STORAGE_ENABLED', 'Enable Storing mission on microSD cards', 0, None),
    Feature('Other', 'COMPASS_CAL', 'COMPASS_CAL_ENABLED', 'Enable "Tumble" compass calibration', 0, None),
    Feature('Other', 'DRONECAN_SERIAL', 'AP_DRONECAN_SERIAL_ENABLED', 'Enable DroneCAN virtual serial ports', 0, "DroneCAN,SERIALDEVICE_REGISTER"),  # NOQA: E501
    Feature('Other', 'Buttons', 'HAL_BUTTON_ENABLED', 'Enable Buttons', 0, None),
    Feature('Other', 'Logging', 'HAL_LOGGING_ENABLED', 'Enable Logging', 0, None),
    Feature('Other', 'CUSTOM_ROTATIONS', 'AP_CUSTOMROTATIONS_ENABLED', 'Enable Custom  sensor rotations', 0, None),
    Feature('Other', 'PID_FILTERING', 'AP_FILTER_ENABLED', 'Enable PID filtering', 0, None),

    # MAVLink section for mavlink features and/or message handling,
    # rather than for e.g. mavlink-based sensor drivers
    Feature('MAVLink', 'HIGHLAT2', 'HAL_HIGH_LATENCY2_ENABLED', 'Enable HighLatency2 Support', 0, None),
    Feature('MAVLink', 'FENCEPOINT_PROTOCOL', 'AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT', 'Enable Old MAVLink fence points protocol', 0, "FENCE"),  # noqa
    Feature('MAVLink', 'RALLYPOINT_PROTOCOL', 'AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED', 'Enable Old MAVLink rally points protocol', 0, "RALLY"),  # noqa
    Feature('MAVLink', 'MAVLINK_VERSION_REQUEST', 'AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED', 'Enable Old AUTOPILOT_VERSION_REQUEST mesage', 0, None),  # noqa
    Feature('MAVLink', 'REQUEST_AUTOPILOT_CAPA', 'AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED', 'Enable Old REQUEST_AUTOPILOT_CAPABILITIES command', 0, None),  # noqa
    Feature('MAVLink', 'MAV_MSG_RELAY_STATUS', 'AP_MAVLINK_MSG_RELAY_STATUS_ENABLED', 'Enable Send RELAY_STATUS message', 0, 'RELAY'),  # noqa
    Feature('MAVLink', 'MAV_DEVICE_OP', 'AP_MAVLINK_MSG_DEVICE_OP_ENABLED', 'Enable DeviceOp MAVLink messages', 0, None),  # noqa
    Feature('MAVLink', 'MAV_SERVO_RELAY', 'AP_MAVLINK_SERVO_RELAY_ENABLED', 'Enable ServoRelay MAVLink messages', 0, 'SERVORELAY_EVENTS'),  # noqa
    Feature('MAVLink', 'MAV_MSG_SERIAL_CONTROL', 'AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED', 'Enable Serial Control MAVLink messages', 0, None),  # noqa
    Feature('MAVLink', 'MAVLINK_MSG_MISSION_REQUEST', 'AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED', 'Enable MISSION_REQUEST MAVLink messages', 0, None),  # noqa
    Feature('MAVLink', 'MAVLINK_MSG_RC_CHANNELS_RAW', 'AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED', 'Enable RC_CHANNELS_RAW MAVLink messages', 0, None),  # noqa
    Feature('MAVLink', 'AP_MAVLINK_FTP_ENABLED', 'AP_MAVLINK_FTP_ENABLED', 'Enable MAVLink FTP protocol', 0, None),  # noqa
    Feature('MAVLink', 'MAV_CMD_SET_HAGL', 'AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED', 'Enable MAVLink HAGL command', 0, None),  # noqa
    Feature('MAVLink', 'VIDEO_STREAM_INFORMATION', 'AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED', 'Enable MAVLink VIDEO_STREAM_INFORMATION message', 0, "Camera"), # noqa

    Feature('Developer', 'KILL_IMU', 'AP_INERTIALSENSOR_KILL_IMU_ENABLED', 'Allow IMUs to be disabled at runtime', 0, None),
    Feature('Developer', 'CRASHCATCHER', 'AP_CRASHDUMP_ENABLED', 'Enable CrashCatcher', 0, None),

    Feature('GPS Drivers', 'UBLOX', 'AP_GPS_UBLOX_ENABLED', 'Enable U-blox GPS', 1, None),
    Feature('GPS Drivers', 'SBP2', 'AP_GPS_SBP2_ENABLED', 'Enable SBP2 GPS', 0, 'SBP'),
    Feature('GPS Drivers', 'SBP', 'AP_GPS_SBP_ENABLED', 'Enable SBP GPS', 0, None),
    Feature('GPS Drivers', 'ERB', 'AP_GPS_ERB_ENABLED', 'Enable ERB GPS', 0, None),
    Feature('GPS Drivers', 'GSOF', 'AP_GPS_GSOF_ENABLED', 'Enable GSOF GPS', 0, None),
    Feature('GPS Drivers', 'NMEA_GPS', 'AP_GPS_NMEA_ENABLED', 'Enable NMEA GPS', 0, None),
    Feature('GPS Drivers', 'NMEA_UNICORE', 'AP_GPS_NMEA_UNICORE_ENABLED', 'Enable NMEA Unicore GPS', 0, "NMEA_GPS"),
    Feature('GPS Drivers', 'MAV', 'AP_GPS_MAV_ENABLED', 'Enable MAVLink GPS', 0, None),
    Feature('GPS Drivers', 'NOVA', 'AP_GPS_NOVA_ENABLED', 'Enable NOVA GPS', 0, None),
    Feature('GPS Drivers', 'SBF', 'AP_GPS_SBF_ENABLED', 'Enable SBF GPS', 0, None),
    Feature('GPS Drivers', 'SIRF', 'AP_GPS_SIRF_ENABLED', 'Enable SiRF GPS', 0, None),
    Feature('GPS Drivers', 'DroneCAN_GPS_Out', 'AP_DRONECAN_SEND_GPS', 'Enable Sending GPS data from Autopilot', 0, "DroneCAN"),  # noqa:401
    Feature('GPS Drivers', 'GPS_Blending', 'AP_GPS_BLENDED_ENABLED', 'Enable GPS Blending', 0, None),


    Feature('Airspeed Drivers', 'Analog', 'AP_AIRSPEED_ANALOG_ENABLED', 'Enable Analog Airspeed', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'ASP5033', 'AP_AIRSPEED_ASP5033_ENABLED', 'Enable ASP5033 AIRSPEED', 0, 'AIRSPEED'),  # NOQA: E501
    Feature('Airspeed Drivers', 'DLVR', 'AP_AIRSPEED_DLVR_ENABLED', 'Enable DLVR AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MS4525', 'AP_AIRSPEED_MS4525_ENABLED', 'Enable MS4525 AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MS5525', 'AP_AIRSPEED_MS5525_ENABLED', 'Enable MS5525 AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'MSP_AIRSPEED', 'AP_AIRSPEED_MSP_ENABLED', 'Enable MSP AIRSPEED', 0, 'AIRSPEED,MSP,OSD'),
    Feature('Airspeed Drivers', 'NMEA_AIRSPEED', 'AP_AIRSPEED_NMEA_ENABLED', 'Enable NMEA AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'SDP3X', 'AP_AIRSPEED_SDP3X_ENABLED', 'Enable SDP3X AIRSPEED', 0, 'AIRSPEED'),
    Feature('Airspeed Drivers', 'DRONECAN_ASPD', 'AP_AIRSPEED_DRONECAN_ENABLED', 'Enable DroneCAN AIRSPEED', 0, 'AIRSPEED,DroneCAN'),   # NOQA: E501
    Feature('Airspeed Drivers', 'AUAV_AIRSPEED', 'AP_AIRSPEED_AUAV_ENABLED', 'ENABLE AUAV AIRSPEED', 0, 'AIRSPEED'),

    Feature('Actuators', 'ServoTelem', 'AP_SERVO_TELEM_ENABLED', 'Enable servo telemetry library', 0, None),
    Feature('Actuators', 'Volz', 'AP_VOLZ_ENABLED', 'Enable Volz Protocol', 0, None),
    Feature('Actuators', 'Volz_DroneCAN', 'AP_DRONECAN_VOLZ_FEEDBACK_ENABLED', 'Enable Volz DroneCAN Feedback', 0, "DroneCAN,Volz,ServoTelem"),  # noqa: E501
    Feature('Actuators', 'RobotisServo', 'AP_ROBOTISSERVO_ENABLED', 'Enable RobotisServo protocol', 0, None),
    Feature('Actuators', 'SBUS Output', 'AP_SBUSOUTPUT_ENABLED', 'Enable SBUS output on serial ports', 0, None),
    Feature('Actuators', 'FETTecOneWire', 'AP_FETTEC_ONEWIRE_ENABLED', 'Enable FETTec OneWire ESCs', 0, None),
    Feature('Actuators', 'KDECAN', 'AP_KDECAN_ENABLED', 'KDE Direct KDECAN ESC', 0, None),
    Feature('Actuators', 'HimarkServo', 'AP_DRONECAN_HIMARK_SERVO_SUPPORT', 'Enable Himark DroneCAN servos', 0, "DroneCAN"),
    Feature('Actuators', 'HobbywingESC', 'AP_DRONECAN_HOBBYWING_ESC_SUPPORT', 'Enable Hobbywing DroneCAN ESCs', 0, "DroneCAN"),

    Feature('Precision Landing', 'PrecLand', 'AC_PRECLAND_ENABLED', 'Enable Precision landing support', 0, None),
    Feature('Precision Landing', 'PrecLand - Companion', 'AC_PRECLAND_COMPANION_ENABLED', 'Enable Companion computer precision landing ', 0, "PrecLand"),  # noqa
    Feature('Precision Landing', 'PrecLand - IRLock', 'AC_PRECLAND_IRLOCK_ENABLED', 'Enable IRLock precision landing support', 0, "PrecLand"),  # noqa

    #    Feature('Filesystem', 'FILESYSTEM_ESP32_ENABLED', 'AP_FILESYSTEM_ESP32_ENABLED', 'Enable ESP32 Filesystem', 0, None),
    # Feature('Filesystem', 'FILESYSTEM_FATFS', 'AP_FILESYSTEM_FATFS_ENABLED', 'Enable FATFS Filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_MISSION', 'AP_FILESYSTEM_MISSION_ENABLED', 'Enable @MISSION/ filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_PARAM', 'AP_FILESYSTEM_PARAM_ENABLED', 'Enable @PARAM/ filesystem', 0, None),
    #    Feature('Filesystem', 'FILESYSTEM_POSIX', 'AP_FILESYSTEM_POSIX_ENABLED', 'Enable POSIX filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_ROMFS', 'AP_FILESYSTEM_ROMFS_ENABLED', 'Enable @ROMFS/ filesystem', 0, None),
    Feature('Filesystem', 'FILESYSTEM_SYS', 'AP_FILESYSTEM_SYS_ENABLED', 'Enable @SYS/ filesystem', 0, None),
    Feature('Filesystem', 'APJ_TOOL_PARAMETERS', 'FORCE_APJ_DEFAULT_PARAMETERS', 'Enable apj_tool parameter area', 0, None),

    Feature('Networking', 'PPP', 'AP_NETWORKING_BACKEND_PPP', 'Enable PPP networking', 0, None),
    # Feature('Networking', 'CAN MCAST', 'AP_NETWORKING_CAN_MCAST_ENABLED', 'Enable CAN multicast bridge', 0, None),

    Feature('CAN', 'DroneCAN', 'HAL_ENABLE_DRONECAN_DRIVERS', 'Enable DroneCAN support', 0, None),
    Feature('CAN', 'CAN Logging', 'AP_CAN_LOGGING_ENABLED', 'Enable CAN logging support', 0, 'Logging'),
]

*/

