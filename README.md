# iomon

Sensor monitor program for IO boards.


## Structure

`iomon/lib` contains modified, simplified versions of Atmel Software Framework
peripheral drivers. Insofar as possible, unused code and functionality has
been removed; particular remaining areas of bloat include the ADCIFA driver,
the various clock drivers, and the USB drivers.

`iomon/src` contains the main program code, divided into files mainly by
peripheral device or support function:
* `cobsr.c` is a serial framing protocol which encodes data packets to ensure
  null bytes are never found in the content, so they can be used as packet
  delimiters;
* `comms.c` is the primary serial/USB driver code for sensor monitor output
  and control input;
* `crc8.c` implements 8-bit, lookup-table-based CRC calculation;
* `gp.c` implements ADC and GPIO interfaces;
* `hmc5883.c` implements an I2C driver for the Honeywell HMC5883L 3-axis
  magnetometer;
* `i2cdevice.c` provides a framework for writing I2C device drivers, allowing
  initialization and read command sequences to be defined as data structures,
  and handling timeouts and power-down/reset logic;
* `main.c` contains the main entry point, initialization routine and event
  loop;
* `mpu6050.c` is an I2C driver for the Invensense MPU-6050 3-axis
  accelerometer/gyro;
* `ms5611.c` is an I2C driver for the Measurement Specialties MS5611
  barometric pressure/temperature sensor;
* `pwm.c` implements PWM management in response to packets received from the
  CPU interface;
* `twim_pdca.c` is used by `i2cdevice.c` and the various I2C drivers to handle
  I2C "transactions" (write/read sequences) and DMA-based I2C commands;
* `ui.c` implements flashing lights in response to various conditions;
* `venus638_gps.c` is a USART-based driver for the Venus638 binary message
  protocol.
* `ubx_gps.c` is a USART-based driver for the u-blox UBX binary protocol.


## Function

The `iomon` program is fundamentally an event loop which aggregates data
received from devices via I2C, UART, ADC and GPIO interfaces, and transmits
it in packets to a processing device. Aside from USB and PWM debug drivers, no
interrupts are used, and the per-packet processing time is guaranteed to be
less than 1ms.

Each sensor has a device driver responsible for initializing it (the
`*_init(void)` procedures) and reading its output (the `*_tick(void)`
procedures). Device driver tick procedures are called every millisecond, and
generally implement a state machine that handles:
* Detection of timeout/error conditions;
* Cycling the device power enable to ensure a hard reset;
* Start-up configuration;
* The primary read sequence and any conversions necessary to obtain usable
  data.

After sensor data is read, it is written to the sensor packet buffer via the
appropriate `comms_set_*` routine; at the end of the event loop, the packet
is sent via UART and serial interfaces. Different sizes of packet are sent,
depending on the data written during that iteration of the loop; in
particular, data from devices with a low refresh rate is not included in each
frame of the output: therefore, the magnetometer, the GPS PVT solution and the
GPS tracking status fields are not mandatory.


## Configuration

The primary configuration mechanism is the board header file, in
`iomon/src/boards`. These header files provide a mapping of all MCU pins to
the appropriate functions, as well as assignments between internal AVR32
peripherals and the external devices. The board file in use is determined by
`iomon/src/config/conf_board.h`.

Aside from those files, USB configuration data is stored in
`iomon/src/config/conf_usb.h` and `iomon/src/config/udi_cdc_conf.h`. These
configurations should not need to be changed, except perhaps for assigning
unique USB device serial numbers to each board.

Both the Venus638 and UBX drivers require that the GPS modules have been
configured appropriately, and the configuration has been saved to Flash. The
required settings are:

### Venus638

* Disable NMEA output (`\x09\x02\x01`)
* Set position rate to 10Hz (`\x0e\x0a\x01`)
* Set nav message interval to one message per position update (`\x11\x01\x01`)
* Switch to 115200 baud (`\x05\x00\x05\x01`)

### UBX

* `CFG MSG`: Enable NAV SOL and NAV LLH
* `CFG NAV5`: Set dynamics mode to `airborne <4g`
* `CFG PRT`: Set UART1 to UBX in, UBX out, 57600 baud (or 115200 if supported)
* `CFG RATE`: Set measurement period to 250ms (or 100ms if supported)


## Testing

TODO


## Building

Requires AVR Studio 6. Configure project for AT32UC3C; 1512 part is currently
used but could probably run on any 0* or 1* part.


## Software installation

Only via JTAG at this stage; just flash the MCU using whatever Atmel
programmer is convenient.

USB DFU bootloader might be added later depending on how frequently we need
to re-flash in the field.

GPS configuration requires u-blox [u-center](http://www.u-blox.com/en/evaluation-tools-a-software/u-center/u-center.html)
and/or the [Venus GPS configuration software](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/GPS/Venus/638/viewer/GPS%20Viewer%20-%20Customer%20Release_110613.zip).

The Venus638 also requires custom [high-dynamic-range firmware](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/GPS/Venus/638/firmware/STI_01.06.04-01.10.24_npse_HiDyn_AGPS_WAAS_LOG_9600_20111021.zip).


## Hardware installation

1. Plug UART into CPU board, or USB into computer;
2. Connect servos and ESC to PWM output lines;
3. Connect magnetometer to external I2C header;
4. Connect battery I/V sensor to P9 header;
5. Connect pitot to P7;
6. Connect ultrasonic distance sensor to P8;
7. Connect battery.
