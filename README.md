# README
```
     ___ _____ _   ___ _  _____ ___  ___  ___ ___
    / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
    \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
    |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
    embedded.connectivity.solutions.==============
```

# Introduction

## Purpose

The STACKFORCE Serial MAC provides framing for serial interfaces.

On TX the STACKFORCE Serial MAC takes over the task to wrap up data in frames
before sending them over the serial interface.
On RX the STACKFORCE Serial MAC listens for incoming frames, verifies their
CRC and provides the payload to the upper layer.

## Serial MAC Protocol version

The Serial MAC Protocol version describes the structure of a Serial MAC frame.
Please note that this version is independent from the product version.

### Serial MAC Protocol V1

The Frame format is:

    +--------------------+-----------------+-- - - --+--------------+
    | SYNC BYTE [1 Byte] | LENGTH [2 Byte] | PAYLOAD | CRC [2 Byte] |
    +--------------------+-----------------+-- - - --+--------------+

  - SYNC BYTE: Fixed value 0xA5.
  - LENGTH: Transmitted payload size. HEADER and CRC are not counted.
  - MAC PAYLOAD Payload of the Serial MAC frame. Variable length which is described through the length field. The content of the MAC PAYLOAD is the serial protocol.
  - CRC: Cyclic redundancy check sum over the MAC PAYLOAD. The CRC polynomial is:


    x^16 + x^13 + x^12 + x^11 + x^10 + x^8 + x^6 + x^5 + x^2 + 1

  Example:
    - Payload: 0x01


    +----+-------+----+-------+
    | A5 | 00 01 | 01 | C2 9A |
    +----+-------+----+-------+


### Serial MAC Protocol V2

The Frame format is:

    +--------------------+-----------------+--------------------------+-- - - --+--------------+
    | SYNC BYTE [1 Byte] | LENGTH [2 Byte] | INVERTED LENGTH [2 Byte] | PAYLOAD | CRC [2 Byte] |
    +--------------------+-----------------+--------------------------+-- - - --+--------------+

  - INVERTED LENGTH: Bitwise inverted length field.
  - All other fields are described in Serial MAC Protocol V1.

  Example:
    - Payload: 0x01


    +----+-------+-------+----+-------+
    | A5 | 00 01 | FF FE | 01 | C2 9A |
    +----+-------+-------+----+-------+


## Features

The STACKFORCE Serial MAC is written with cross-platform portability in mind.
It should be usable within operating systems as well as bare metal devices.

* All API functions are non-blocking.
* The MAC has no direct dependencies (besides standard C libs and
STACKFORCE utilities that are hardware/OS independent, e.g. CRC module).
* The MAC is usable with any HAL library that provides non-blocking
functions to read from and write to the serial interface and a function
which returns the number of bytes waiting on input.
* Buffer allocation and management is completely left to the upper layer.

## Build instructions

The STACKFORCE Serial MAC uses CMake as build system. Note that, once built, the library can be used by other cmake projects with the `find_package()` command without requiring to install the library on the system. Therefore running a `make install` is not mandatory to be able to build other projects that link to the serialmac library.

Go to root, create build directory:

    cd serial-interface-mac
    mkdir build

and run:

    cmake ..
    make
    sudo make install

or to define a custom install directory e.g. devroot:

    cmake -DCMAKE_INSTALL_PREFIX=devroot ..
    make
    make install

To generate packages run:

    make package

This will generate a tar.gz archive, and installer shell script by default.
If run under Ubuntu, Debian or LinuxMint, a **deb** package will be generated.
Use **dpkg** as follows to install the package.

    dpkg -i package_name.deb

To generate the doxygen documentation run:

    cmake -DBUILD_DOC=on ..
    make doc

# Usage

## Initialization

To use the STACKFORCE Serial MAC you have to initialize it using
**sf_serialmac_init()**

## Reacting to events

The STACKFORCE Serial MAC is event driven. You can use the MAC by calling
**sf_serialmac_entry()** periodically.

Or you can add **sf_serialmac_hal_tx_callback()** and
**sf_serialmac_hal_rx_callback()** as callback function to the corresponding
serial port events.

## Receiving frames

Whenever the STACKFORCE Serial MAC receives the header of a frame it calls
the upper layers callback function registered as **SF_SERIALMAC_RX_EVENT
rx_buffer_event()** on Initialization. To receive the frame the upper layer has
to provide a memory location for the payload passed to the MAC by calling
**sf_serialmac_rx_frame()**. As soon as the frame has been completed,
the upper layer's callback function is called which has been registered
as **SF_SERIALMAC_RX_EVENT rx_event()** on initialization.

In case there is any error or problem (e.g. invalid CRC) while receiving a frame,
the upper layer's error callback function is called. This function has been registered as
**SF_SERIALMAC_EVENT_ERROR error_event** on initialization.

## Transmitting frames

Frames can be transmitted at once using **sf_serialmac_tx_frame()**. Or by
starting a frame with **sf_serialmac_tx_frame_start()** and successively
appending the payload using **sf_serialmac_tx_frame_append()** until the frame
is filled.

Whenever the MAC completed the transmission of a frame the upper layer's
callback called that has been registered as **SF_SERIALMAC_TX_EVENT tx_event()**
on initialization.

Whenever the MAC processed a buffer with payload the upper layer's callback
is called which has been registered as **SF_SERIALMAC_TX_EVENT tx_buf_event** on
initialization. The upper layer must not touch the buffer memory passed with
**sf_serialmac_tx_frame()** or **sf_serialmac_tx_frame_append()** before this
callback has been called. Also all calls to **sf_serialmac_tx_frame_append()**
are ignored until the previously provided buffer has been processed.
