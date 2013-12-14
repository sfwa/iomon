# iomon

Sensor monitor program for IO boards.


## Structure

`iomon/lib` contains modified, simplified versions of Atmel Software Framework
peripheral drivers. Insofar as possible, unused code and functionality has
been removed; particular remaining areas of bloat include the ADCIFA driver
and the various clock drivers.

`iomon/src` contains the main program code, divided into files mainly by
peripheral device or support function:
* `cobsr.c` is a serial framing protocol which encodes data packets to ensure
  null bytes are never found in the content, so they can be used as packet
  delimiters;
* `comms.c` is the primary serial driver code for sensor monitor output
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
* `ubx_gps.c` is a USART-based driver for the u-blox UBX binary protocol.


## Function

The `iomon` program is fundamentally an event loop which aggregates data
received from devices via I2C, UART, ADC and GPIO interfaces, and transmits
it in packets to a processing device. No interrupts are used, and the
per-packet processing time is guaranteed to be less than 1ms.

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

The UBX driver requires that the GPS module has been configured appropriately,
and the configuration has been saved to Flash. The required settings are:

### UBX

* `CFG MSG`: Enable NAV PVT on UART1
* `CFG NAV5`: Set dynamics mode to `airborne <4g`
* `CFG PRT`: Set UART1 to UBX in, UBX out, 57600 baud (or 115200 if supported)
* `CFG RATE`: Set measurement period to 250ms (or 100ms if supported)
* `CFG CFG`: save to flash


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


## Hardware installation

1. Plug UART into CPU board;
2. Connect servos and ESC to PWM output lines;
3. Connect magnetometer to external I2C header;
4. Connect battery I/V sensor to P9 header;
5. Connect pitot to P7;
6. Connect ultrasonic distance sensor to P8;
7. Connect battery.
