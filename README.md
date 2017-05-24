# README                                                             {#mainpage}
    @code

     ___ _____ _   ___ _  _____ ___  ___  ___ ___
    / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
    \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
    |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
    embedded.connectivity.solutions.==============

    @endcode

# Introduction                                                   {#introduction}

## Purpose                                                            {#purpose}

The STACKFORCE Serial MAC provides framing for serial interfaces.

On TX the STACKFORCE Serial MAC takes over the task to wrap up data in frames
before sending them over the serial interface.
On RX the STACKFORCE Serial MAC listens for incoming frames, verifies their
CRC and provides the payload to the upper layer.

## Frame format                                                         {#frame}

The Frame format is:

    +--------------+--------+-- - - --+-----+
    | SYNC BYTE(S) | LENGTH | payload | CRC |
    +--------------+--------+-- - - --+-----+

## Features                                                           {#feature}

The STACKFORCE Serial MAC is written with cross-platform portability in mind.
It should be usable within operating systems as well as bare metal devices.

* All API functions are non-blocking.
* The MAC has no direct dependencies (besides standard C libs and
STACKFORCE utilities that are hardware/OS independent, e.g. CRC module).
* The MAC is usable with any HAL library that provides non-blocking
functions to read from and write to the serial interface and a function
which returns the number of bytes waiting on input.
* Buffer allocation and management is completely left to the upper layer.

## Build instructions                                                   {#build}

The STACKFORCE Serial MAC uses CMake as build system. However it can be 
integrated as sources or as library into your own project. How to build:
Go to root, create build directory:

    cd serial-interface-mac
    mkdir build

and run:

    cmake ..
    make
    sudo make install

or to define a custom install directory e.g. devroot:

    cmake .. -DCMAKE_INSTALL_PREFIX=devroot
    make
    make install

To generate the doxygen documentation run:

    make doc

# Usage                                                                 {#usage}

## Initialization                                              {#initialization}

To use the STACKFORCE Serial MAC you have to initialize it using
sf_serialmac_init()

## Reacting to events                                                  {#events}

The STACKFORCE Serial MAC is event driven. You can use the MAC by calling
sf_serialmac_entry() periodically.

Or you can add sf_serialmac_hal_tx_callback() and
sf_serialmac_hal_rx_callback() as callback function to the corresponding
serial port events.

## Receiving frames                                                        {#rx}

Whenever the STACKFORCE Serial MAC receives the header of a frame it calls
the upper layers callback function registered as SF_SERIALMAC_RX_EVENT
rx_buffer_event() on Initialization. To receive the frame the upper layer has
to provide a memory location for the payload passed to the MAC by calling
sf_serialmac_rx_frame(). As soon as the frame has been completed or rejected
due to CRC error or time out, the upper layer's callback function is called
which has been registered as SF_SERIALMAC_RX_EVENT rx_event() on
initialization.

## Transmitting frames                                                     {#tx}

Frames can be transmitted at once using sf_serialmac_tx_frame(). Or by
starting a frame with sf_serialmac_tx_frame_start() and successively
appending the payload using sf_serialmac_tx_frame_append() until the frame
is filled.

Whenever the MAC completed the transmission of a frame the upper layer's
callback called that has been registered as SF_SERIALMAC_TX_EVENT tx_event()
on initialization.

Whenever the MAC processed a buffer with payload the upper layer's callback
is called which has been registered as SF_SERIALMAC_TX_EVENT tx_buf_event on
initialization. The upper layer must not touch the buffer memory passed with
sf_serialmac_tx_frame() or sf_serialmac_tx_frame_append() before this
callback has been called. Also all calls to sf_serialmac_tx_frame_append()
are ignored until the previously provided buffer has been processed.
