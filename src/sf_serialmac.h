#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, www.stackforce.de
 * @author     STACKFORCE
 * @author     Lars Möllendorf
 * @brief      STACKFORCE Serial MAC Module
 *
 * @details Please consult the "README" for a general overview
 * of the STACKFORCE Serial MAC.
 *
 * This file is part of the STACKFORCE Serial MAC Library
 * (below "libserialmac").
 *
 * libserialmac is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libserialmac is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with libserialmac.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __STACKFORCE_SERIALMAC_H_
#define __STACKFORCE_SERIALMAC_H_

/* Definition of portable data types */
#include <stdint.h>
/* This API makes use of size_t */
#include <stddef.h>

/**
 * Return values.
 */
enum sf_serialmac_return
{
    /** No error occurred (always equals 0). */
    SF_SERIALMAC_SUCCESS = 0,
    /** Null pointer exception */
    SF_SERIALMAC_ERROR_NPE,
    /**
     * There is already an operation in progress. Retry later.
     */
    SF_SERIALMAC_ERROR_RW_PENDING,
    /**
     * A previously started frame is still being processed.
     * Wait for for the TX callback before starting a new frame.
     */
    SF_SERIALMAC_ERROR_FRM_PENDING,
    /** The HAL is busy (or you are too fast ;)) and did send nothing. */
    SF_SERIALMAC_ERROR_HAL_BUSY,
    /**
     * The HAL is slow (or you are too fast ;)) but did send at least one byte
     * (this is needed to work around slow serial handlers on Windows).
     */
    SF_SERIALMAC_ERROR_HAL_SLOW,
    /** The HAL reports an error. */
    SF_SERIALMAC_ERROR_HAL_ERROR,
    /** There was an error that should never have happened ;). */
    SF_SERIALMAC_ERROR_EXCEPTION,
    /** There was an error in buffer handling */
    SF_SERIALMAC_ERROR_BUFFER,
};


/**
 * Structure used by the MAC to store its context.
 */
struct sf_serialmac_ctx;

/**
 * Signature of HAL's read function to be used by the MAC for RX.
 * This has to be passed on initialization to @ref sf_serialmac_init().
 */
typedef size_t ( *SF_SERIALMAC_HAL_READ_FUNCTION ) ( void *port_handle,
        uint8_t *frame_buffer, size_t frame_buffer_length );
/**
 * Signature of HAL's function which returns the number of bytes waiting on
 * input to be used by the MAC for RX.
 * This has to be passed on initialization to @ref sf_serialmac_init().
 */
typedef size_t ( *SF_SERIALMAC_HAL_READ_WAIT_FUNCTION ) ( void *port_handle );

/**
 * Signature of HAL's write function to be used by the MAC for TX.
 * This has to be passed on initialization to @ref sf_serialmac_init().
 *
 * @param port_handle Points to the port handle that may be needed by the HAL.
 * @param frame_buffer Points to the buffer that has to be written.
 * @param frame_buffer_length Length of the buffer to be written in bytes.
 * @return Number of bytes successfully written.
 */
typedef size_t ( *SF_SERIALMAC_HAL_WRITE_FUNCTION ) ( void *port_handle,
        uint8_t *frame_buffer, size_t frame_buffer_length );

/**
 * Signature of upper layer's callback functions to be called by the MAC
 * on events.
 * These functions have to be passed on initialization to
 * @ref sf_serialmac_init().
 */
typedef void ( *SF_SERIALMAC_EVENT ) ( void *mac_context,
                                       uint8_t *frame_buffer,
                                       size_t frame_buffer_length );


/**
 * Returns the size of the MAC context structure.
 *
 * On initialization the upper layer has to provide memory the STACKFORCE
 * Serial MAC uses to store its context into. This is done by passing a pointer
 * memory allocated by the upper layer to @ref sf_serialmac_init(). This function
 * returns the size of memory that has to be provided.
 *
 * @return The size of the MAC context structure.
 */
size_t sf_serialmac_ctx_size ( void );


/**
 * Initialization of STACKFORCE Serial MAC.
 *
 * This function must be called once before the MAC can be used.
 *
 * @param ctx Points to the memory region the MAC can use for its context.
 * Please use sf_serialmac_ctx_size() to get the memory size needed by the
 * MAC context.
 * @param port_handle Points to the handle of the serial port which is passed to
 * the underlying HAL.
 * @param read Read function of the underlying HAL.
 * @param read_wait Function of the underlying HAL, that returns the number of
 * bytes waiting in the input buffer.
 * @param write Write function of the underlying HAL.
 * @param rx_event Callback function to be called by the MAC when a whole frame
 * has been received.
 * @param rx_buffer_event Callback function to be called by the MAC when an
 * ingoing buffer has to be provided. If this function is called a frame is
 * ready to be received using sf_serialmac_rxFrame().
 * @param rx_sync_event Callback function to be called by the MAC when a SYNC BYTE
 * @ref SF_SERIALMAC_PROTOCOL_SYNC_WORD has been received.
 * @param tx_event Callback function to be called by the MAC when a whole frame
 * has been sent.
 * @param tx_buffer_event Callback function to be called by the MAC when an
 * outgoing buffer has been processed.
 * @return Error state.
 */
enum sf_serialmac_return sf_serialmac_init ( struct sf_serialmac_ctx *ctx,
        void *port_handle, SF_SERIALMAC_HAL_READ_FUNCTION read,
        SF_SERIALMAC_HAL_READ_WAIT_FUNCTION read_wait,
        SF_SERIALMAC_HAL_WRITE_FUNCTION write, SF_SERIALMAC_EVENT rx_event,
        SF_SERIALMAC_EVENT rx_buffer_event, SF_SERIALMAC_EVENT rx_sync_event,
        SF_SERIALMAC_EVENT tx_event, SF_SERIALMAC_EVENT tx_buffer_event );

/**
 * Reset function of STACKFORCE Serial MAC.
 *
 * This function resets the variables and states in the ctx and can be
 * called whenever a reset of the state of the Serial MAC is needed
 * (e.g. after a timeout in the application).
 *
 * @param ctx Context to reset variables txFrame and rxFrame.
 * @return Error state.
 */
enum sf_serialmac_return sf_serialmac_reset ( struct sf_serialmac_ctx *ctx );

/**
 * This function can be passed to the HAL layer as callback function to be
 * called on TX events.
 *
 * This is an alternative to the usage of sf_serialmac_entry().
 *
 * @param ctx Points to the memory region the MAC uses to store its context.
 * @return Error state:
 *         - SF_SERIALMAC_ERROR_HAL_ERROR: The HAL reported an error.
 *         - SF_SERIALMAC_ERROR_HAL_BUSY: The HAL is busy.
 *         - SF_SERIALMAC_ERROR_HAL_SLOW: The HAL is busy, but you may retry
 *                                        immediately (Workaround for slow HALs).
 *         - SF_SERIALMAC_SUCCESS: The payload buffer has been processed.
 *         - SF_SERIALMAC_ERROR_EXCEPTION: You hit a bug.
 */
enum sf_serialmac_return sf_serialmac_hal_tx_callback ( struct sf_serialmac_ctx
        *ctx );


/**
 * This function can be passed to the HAL layer as callback function to be
 * called on RX events.
 *
 * This is an alternative to the usage of sf_serialmac_entry().
 *
 * @param ctx Points to the memory region the MAC uses for its context.
 * @return Error state.
 */
enum sf_serialmac_return sf_serialmac_hal_rx_callback ( struct sf_serialmac_ctx
        *ctx );


/**
 * This function has to be called periodically so the MAC can process incoming
 * data from the underlying serial interface and outgoing data from the upper
 * layer. Alternatively sf_serialmac_hal_tx_callback() and
 * sf_serialmac_hal_rx_callback() can be used.
 *
 * @param ctx Points to the memory region the MAC uses to store its context.
 */
void sf_serialmac_entry ( struct sf_serialmac_ctx *ctx );


/**
 * Start a MAC frame with given length, i.e. initialize frame buffers and send
 * the frame header.
 *
 * This is a non-blocking function.
 *
 * Use sf_serialmac_txFrameAppend() to append payload data.
 *
 * @param ctx Points to the memory region the MAC uses to store its context.
 * @param frame_length Length of the frame to send.
 * @return Error state:
 *         - SF_SERIALMAC_ERROR_NPE: ctx is NULL
 *         - SF_SERIALMAC_ERROR_FRM_PENDING: A frame has been started already
 *                                           and not completed yet.
 */
enum sf_serialmac_return sf_serialmac_tx_frame_start ( struct sf_serialmac_ctx
        *ctx, size_t frame_length );


/**
 * Append data to the current frame's payload.
 * Call sf_serialmac_tx_frame_start() first.
 *
 * This is a non-blocking function.
 *
 * As soon as all bytes in the buffer have been sent tx_buffer_event() is
 * called. The number of processed buffer bytes is passed to tx_buffer_event().
 *
 * As soon as the payload length specified in sf_serialmac_tx_frame_start()
 * has been reached the frame is completed with an CRC and tx_event()
 * is called.
 *
 * Remaining payload that did not fit into the frame is ignored. The number of
 * processed payload bytes is passed to tx_event().
 *
 * @param ctx Points to the memory region the MAC uses to store its context.
 * @param frame_buffer Points to the buffer containing the payload to be
 * appended to the frame.
 * @param frame_buffer_size Length of the buffer containing the payload to be
 * appended to the frame.
 * @return Error state:
 *         - SF_SERIALMAC_ERROR_NPE: ctx is NULL
 *         - SF_SERIALMAC_ERROR_RW_PENDING: There is still a payload buffer in
 *                                          progress.
 */
enum sf_serialmac_return sf_serialmac_tx_frame_append ( struct sf_serialmac_ctx
        *ctx, const uint8_t *frame_buffer, size_t frame_buffer_size );


/**
 * Start a frame if not already done and append given payload.
 * It is save to always use this function instead of
 * sf_serialmac_tx_frame_start() and sf_serialmac_tx_frame_append().
 * This function is a combination of sf_serialmac_tx_frame_start() and
 * sf_serialmac_tx_frame_append().
 *
 * This is a non-blocking function.
 *
 * @param ctx Points to the memory region the MAC uses to store its context.
 * @param frame_length Length of the frame to send.
 * @param frame_buffer Points to the buffer containing the payload to be
 * appended to the frame.
 * @param frame_buffer_size Length of the buffer containing the payload to be
 * appended to the frame.
 * @return Error state:
 *         - SF_SERIALMAC_ERROR_NPE: ctx is NULL
 *         - SF_SERIALMAC_ERROR_FRM_PENDING: A frame has been started already
 *                                           and not processed yet.
 *         - SF_SERIALMAC_ERROR_RW_PENDING: There is still a payload buffer in
 *                                          progress.
 */
enum sf_serialmac_return sf_serialmac_tx_frame ( struct sf_serialmac_ctx *ctx,
        size_t frame_length, const uint8_t *frame_buffer, size_t frame_buffer_size
                                               );


/**
 * The upper layer has to call this function whenever the MAC has notified it
 * about the reception of a frame header by calling the upper layers callback
 * function registered on initialization as
 * SF_SERIALMAC_RX_EVENT rx_buffer_event().
 * The MAC expects a frame_buffer with a frame_buffer_size greater or equal
 * the frame length.
 *
 * @param ctx Points to the memory region the MAC uses to store its context.
 * @param frame_buffer Points to the memory where the MAC shall store the
 * payload of the expected frame.
 * @param frame_buffer_size The size of the frame_buffer in bytes.
 */
enum sf_serialmac_return sf_serialmac_rx_frame ( struct sf_serialmac_ctx *ctx,
        uint8_t *frame_buffer, size_t frame_buffer_size );


/*
 * Internal types to enable static memory usage of libserialmac.
 * DO NOT ACCESS directly, call API function above for accessing.
 */

/** SYNC word of the STACKFORCE serial protocol */
#define SF_SERIALMAC_PROTOCOL_SYNC_WORD              0xA5U
/** Length of the STACKFORCE serial protocol SYNC word field. */
#define SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN          0x01U
/** Length of the STACKFORCE serial protocol length field */
#define SF_SERIALMAC_PROTOCOL_LENGTH_FIELD_LEN       0x02U
/** Length of the STACKFORCE serial protocol CRC field */
#define SF_SERIALMAC_PROTOCOL_CRC_FIELD_LEN          0x02U
/** Length of the serial MAC frame header */
#define SF_SERIALMAC_PROTOCOL_HEADER_LEN      \
 (SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN + SF_SERIALMAC_PROTOCOL_LENGTH_FIELD_LEN)

/**
 * A frame consists of the elements:
 * <ul>
 * <li>header (H)</li>
 * <li>payload</li>
 * <li>CRC (C)</li>
 * </ul>
 *
 * All three elements have their own serial MAC buffer which are held in the
 * serial MAC context.
 *
 * The MAC keeps track of the number of payload bytes that still needs to be
 * processed.
 *
 * Those 3 buffers then are processed sequentially in this order:
 *
 * |HHH|payload...|CC|
 *
 * 4 states have to be distinguished:
 */
enum rxTxState {
    /** MAC is idle. */
    SF_SERIALMAC_IDLE,
    /** HEADER is about to be processed. */
    SF_SERIALMAC_HEADER,
    /** PAYLOAD is about to be processed. */
    SF_SERIALMAC_PAYLOAD,
    /** CRC is about to be processed. */
    SF_SERIALMAC_CRC,
};

/**
 * Signature of APP's callback function to be called by the MAC
 * when a buffer has been processed.
 *
 * @param byteWritten Number of written byte.
 */
typedef void ( *SF_SERIALMAC_BUF_EVT ) ( struct sf_serialmac_ctx *ctx );

/**
 * Context of an internal serial MAC buffer.
 */
struct sf_serialmac_buffer {
    /** Memory for the bytes to be processed. */
    uint8_t *memory;
    /** length of the buffer memory in bytes. */
    size_t length;
    /** Bytes that still needs to be processed. */
    size_t remains;
    /** Function to be called when all bytes are processed. */
    SF_SERIALMAC_BUF_EVT callback;
};

/**
 * Context of an serial MAC frame.
 *
 * There is no memory for the payload, because this is handed over by the upper
 * layer.
 */
struct sf_serialmac_frame {
    enum rxTxState state;
    /** Payload bytes that still needs to be processed. */
    uint16_t remains;
    /** Memory for the MAC header: [SYNC] [Length field] */
    uint8_t headerMemory[SF_SERIALMAC_PROTOCOL_HEADER_LEN];
    /** Memory for the CRC. */
    uint8_t crcMemory[SF_SERIALMAC_PROTOCOL_CRC_FIELD_LEN];
    /** Buffer for the frame header to transmit. */
    struct sf_serialmac_buffer headerBuffer;
    /** Buffer for the frame payload to transmit. */
    struct sf_serialmac_buffer payloadBuffer;
    /** Buffer for the frame CRC to transmit. */
    struct sf_serialmac_buffer crcBuffer;
};

/**
 * Context of the serial MAC.
 */
struct sf_serialmac_ctx {
    /** Handle of the serial port that is passed through to the lower HAL. */
    void *portHandle;
    /** Read function of the lower HAL. */
    SF_SERIALMAC_HAL_READ_FUNCTION read;
    /**
     * Function of the lower HAL that returns number of byte waiting for
     * reading in HAL's buffer.
     */
    SF_SERIALMAC_HAL_READ_WAIT_FUNCTION readWait;
    /** Write function of the lower HAL. */
    SF_SERIALMAC_HAL_WRITE_FUNCTION write;
    /** Function to be called when a whole frame has been received. */
    SF_SERIALMAC_EVENT rx_frame_event;
    /** Function to be called when a RX buffer is needed to receive a frame. */
    SF_SERIALMAC_EVENT rx_buffer_event;
    /** Function to be called when a sync byte has been received. */
    SF_SERIALMAC_EVENT rx_sync_event;
    /** Function to be called when a whole frame has been sent. */
    SF_SERIALMAC_EVENT tx_frame_event;
    /** Function to be called when a TX buffer has been processed. */
    SF_SERIALMAC_EVENT tx_buffer_event;
    /** Context of the frame to send. */
    struct sf_serialmac_frame txFrame;
    /** Context of the frame to receive. */
    struct sf_serialmac_frame rxFrame;
};

#endif /* STACKFORCE_SERIALMAC_H_ */
#ifdef __cplusplus
}
#endif
