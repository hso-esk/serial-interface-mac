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
 * @brief      STACKFORCE serial command line client (sf)
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

/** definition of portable data types */
#include <stdint.h>
#include <stdbool.h>
/** Common definitions most notably NULL */
#include <stddef.h>
/** memset is used */
#include <string.h>

/** used to calculate crc */
#include "sf_crc.h"
#include "sf_serialmac.h"

#define UINT16_TO_UINT8(u8arr, u16var)         ((u8arr)[0] =\
( uint8_t ) ( ( uint8_t ) ( ( u16var ) >>8U ) & 0xFFU ) ); \
( ( u8arr ) [1] = ( uint8_t ) ( ( u16var ) & 0xFFU ) )
#define UINT8_TO_UINT16(u8arr) (((((uint16_t)((u8arr)[0]))<<8U) & 0xFF00U) | \
(((uint16_t)((u8arr)[1])) & 0xFFU ) )

static struct sf_serialmac_buffer* initBuffer (
    struct sf_serialmac_buffer *buffer, uint8_t *memory, size_t length,
    SF_SERIALMAC_BUF_EVT callback );
static void txInit ( struct sf_serialmac_ctx *ctx );
static enum sf_serialmac_return tx ( struct sf_serialmac_ctx *ctx,
                                     struct sf_serialmac_buffer
                                     *buffer, uint8_t *crc );
static void txProcHeaderCB ( struct sf_serialmac_ctx *ctx );
static void txProcPayloadCB ( struct sf_serialmac_ctx *ctx );
static void txProcCrcCB ( struct sf_serialmac_ctx *ctx );
static void rxInit ( struct sf_serialmac_ctx *ctx );
static enum sf_serialmac_return rx ( struct sf_serialmac_ctx *ctx,
                                     struct sf_serialmac_buffer *buffer,
                                     size_t bytesWaiting
                                   );
static void rxProcHeaderCB ( struct sf_serialmac_ctx *ctx );
static void rxProcPayloadCB ( struct sf_serialmac_ctx *ctx );
static void rxProcCrcCB ( struct sf_serialmac_ctx *ctx );


static struct sf_serialmac_buffer *initBuffer (
    struct sf_serialmac_buffer *buffer, uint8_t *memory, size_t length,
    SF_SERIALMAC_BUF_EVT callback )
{
    if ( buffer ) {
        buffer->memory = memory;
        buffer->length = length;
        buffer->remains = length;
        buffer->callback = callback;
    }
    return buffer;
}


static void initFrame ( struct sf_serialmac_frame *frame, uint8_t syncWord )
{
    frame->remains = 0;
    /** zero buffer */
    memset ( ( void * ) frame->headerMemory, 0, SF_SERIALMAC_PROTOCOL_HEADER_LEN
           );
    /** zero buffer */
    memset ( ( void * ) frame->crcMemory, 0, SF_SERIALMAC_PROTOCOL_CRC_FIELD_LEN
           );
    /** Write the sync word into the buffer */
    frame->headerMemory[0] = syncWord;
}


static void txInit ( struct sf_serialmac_ctx *ctx )
{
    initBuffer ( &ctx->txFrame.headerBuffer, ctx->txFrame.headerMemory,
                 ctx->headerLength, txProcHeaderCB );
    initBuffer ( &ctx->txFrame.payloadBuffer, NULL, 0, txProcPayloadCB );
    initBuffer ( &ctx->txFrame.crcBuffer, ctx->txFrame.crcMemory,
                 SF_SERIALMAC_PROTOCOL_CRC_FIELD_LEN, txProcCrcCB );
    initFrame ( &ctx->txFrame, SF_SERIALMAC_PROTOCOL_SYNC_WORD );
}


/**
 * Transmit up to sf_serialmac_buffer.length byte from given buffer.
 * Initialize buffer when everything is transmitted.
 *
 * @param *ctx the current context
 * @param *buffer the buffer to transmit
 * @param *crc place to store crc value to. Set to null if crc shall not be
 *             calculated.
 */
static enum sf_serialmac_return tx ( struct sf_serialmac_ctx *ctx,
                                     struct sf_serialmac_buffer
                                     *buffer, uint8_t *crc )
{
    /**
     * We need a signed integer to handle negative return values that
     * indicate an error
     */
    int byteSent = 0;
    uint16_t crcRead = 0;
    uint16_t crcCalc = 0;

    /** Check if we (still) have bytes to send */
    if ( buffer->remains ) {
        /** Send the bytes */
        if ( ( byteSent = ctx->write ( ctx->portHandle,
                                       buffer->memory
                                       + ( buffer->length - buffer->remains ),
                                       buffer->remains ) )  == 0 ) {
            /** No error, but nothing sent. */
            return SF_SERIALMAC_RETURN_ERROR_HAL_BUSY;
        } else if ( buffer->remains < byteSent ) {
            /** This should never happen, but if it does we can catch it. */
            return SF_SERIALMAC_RETURN_ERROR_EXCEPTION;
        }
        if ( crc ) {
            crcRead = UINT8_TO_UINT16 ( crc );
            crcCalc = crc_calc ( crcRead, ( uint8_t* ) buffer->memory
                                 + ( buffer->length - buffer->remains ),
                                 byteSent );
            UINT16_TO_UINT8 ( crc, crcCalc );
        }

        /** update to the number of byte already sent */
        buffer->remains -= byteSent;
    }

    /** Check if all bytes have been sent */
    if ( !buffer->remains ) {
        buffer->callback ( ctx );
        return SF_SERIALMAC_RETURN_SUCCESS;
    } else {
        /**
         * A special return value has been added here to work-around
         * the Windows specific serial interface API problems.
         * The Windows I/O model is based on async requests so one can do a
         * write with a pointer and a count and have it execute in the
         * background.
         * But the caller needs to keep that buffer untouched until it's
         * done and the write will take as large as a request as you like, no
         * relation whether the device is actually ready for new data.
         * Libserialport (the lib used on Windows) tries to offer the
         * same nonblocking semantics as on UNIX. To do so it just sends one
         * byte at a time and immediately checks whether the byte has been send
         * or not using HasOverlappedIoCompleted().
         * Actually this HasOverlappedIoCompleted() always returned false in
         * my test.
         * However, an immediate retry to send succeeds in most cases. So if
         * any byte has been send on Windows it is worth to immediately retry
         * and not hand the control back to main().
         * If Libserialport can't hand the single byte over to the serial
         * interface it returns 0 which indicates that the serial interface
         * buffer is full.
         * See Libserialport's sp_nonblocking_write() function for details.
         */
        return SF_SERIALMAC_RETURN_ERROR_HAL_SLOW;
    }
}


static void txProcHeaderCB ( struct sf_serialmac_ctx *ctx )
{
    ctx->txFrame.state = SF_SERIALMAC_PAYLOAD;
}


static void txProcPayloadCB ( struct sf_serialmac_ctx *ctx )
{
    /**
     * Save a pointer to the buffer_memory for the callback function.
     * This pointer is passed to the upper layer so it can free the memory.
     */
    uint8_t *buffer_memory = ctx->txFrame.payloadBuffer.memory;
    uint16_t crcRead = 0;
    uint16_t crcCalc = 0;
    size_t processed = ctx->txFrame.payloadBuffer.length -
                       ctx->txFrame.payloadBuffer.remains;
    ctx->txFrame.remains -= processed;
    if ( ctx->txFrame.remains <= 0 ) {
        crcRead = UINT8_TO_UINT16 ( ctx->txFrame.crcMemory );
        crcCalc = crc_finalize ( crcRead );
        UINT16_TO_UINT8 ( ctx->txFrame.crcMemory, crcCalc );
        ctx->txFrame.state = SF_SERIALMAC_CRC;
    }
    /**
     * Clear the buffer, so the write event won't be called again
     * and again for this buffer.
     */
    initBuffer ( &ctx->txFrame.payloadBuffer, NULL, 0, txProcPayloadCB );
    /** inform upper layer that the buffer has been processed */
    ctx->tx_buffer_event ( ctx, buffer_memory, processed );
}


static void txProcCrcCB ( struct sf_serialmac_ctx *ctx )
{
    size_t length = UINT8_TO_UINT16 ( ctx->txFrame.headerMemory +
                                      SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN );
    txInit ( ctx );
    ctx->txFrame.state = SF_SERIALMAC_IDLE;
    /**
     * There is no buffer associated to frame as such (only to its payload).
     * Therefore there is now pointer that can be passed here to the upper
     * layer.
     */
    ctx->tx_frame_event ( ctx, NULL, length );
}


static void rxInit ( struct sf_serialmac_ctx *ctx )
{
    initBuffer ( &ctx->rxFrame.headerBuffer, ( uint8_t* )
                 &ctx->rxFrame.headerMemory,
                 ctx->headerLength, rxProcHeaderCB );
    initBuffer ( &ctx->rxFrame.payloadBuffer, NULL, 0, rxProcPayloadCB );
    initBuffer ( &ctx->rxFrame.crcBuffer, ctx->rxFrame.crcMemory,
                 SF_SERIALMAC_PROTOCOL_CRC_FIELD_LEN, rxProcCrcCB );
    initFrame ( &ctx->rxFrame, 0 );
}


static enum sf_serialmac_return rx ( struct sf_serialmac_ctx *ctx,
                                     struct sf_serialmac_buffer
                                     *buffer, size_t bytesWaiting )
{
    size_t byteToReceive = 0;
    size_t bytesReceived = 0;

    if( buffer->memory == NULL )
    {
      /** No buffer assigned. */
      return SF_SERIALMAC_RETURN_ERROR_BUFFER;
    }

    byteToReceive = buffer->remains > bytesWaiting ? bytesWaiting :
                    buffer->remains;
    if ( byteToReceive ) {
        if ( ( bytesReceived = ctx->read ( ctx->portHandle,
                                           ( void* ) ( buffer->memory +
                                                   buffer->length -
                                                   buffer->remains ),
                                           byteToReceive ) ) !=
                                           byteToReceive ) {
            /** This should never happen, but if it does we can catch it. */
            return SF_SERIALMAC_RETURN_ERROR_EXCEPTION;
        }
        buffer->remains -= bytesReceived;
        if ( !buffer->remains ) {
            buffer->callback ( ctx );
        }
    }

    return SF_SERIALMAC_RETURN_SUCCESS;
}


static void rxProcHeaderCB ( struct sf_serialmac_ctx *ctx )
{
    bool frameValid = true;
    uint16_t invertedLength;

    /** Get the length field  */
    ctx->rxFrame.remains = UINT8_TO_UINT16 ( ctx->rxFrame.headerMemory +
                           SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN );

    /* Check if inverted length feature is selected. */
    if( ctx->useInvertedLengthField )
    {
      /** Get the inverted length field */
      invertedLength = UINT8_TO_UINT16 ( ctx->rxFrame.headerMemory +
                                SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN +
                                SF_SERIALMAC_PROTOCOL_LENGTH_FIELD_LEN );

      /** Validate the header. */
      if( ctx->rxFrame.remains != ( uint16_t ) ( ~invertedLength ) )
      {
        frameValid = false;
        /** Inform upper layer that the frame header was not valid. */
        ctx->error_event ( ctx, SF_SERIALMAC_ERROR_LENGTH_VERIFICATION_FAILED );
        /** Header validation failed. Reset RX state. */
        initBuffer ( &ctx->rxFrame.headerBuffer, ( uint8_t* )
                     &ctx->rxFrame.headerMemory,
                     ctx->headerLength,
                     rxProcHeaderCB );
        ctx->rxFrame.state = SF_SERIALMAC_IDLE;
      }
    }

    if( frameValid )
    {
        /** Inform upper layer that there has been a frame header received */
        ctx->rxFrame.state = SF_SERIALMAC_PAYLOAD;
        ctx->rx_buffer_event ( ctx, NULL, ctx->rxFrame.remains );
    }
}


static void rxProcPayloadCB ( struct sf_serialmac_ctx *ctx )
{
    ctx->rxFrame.remains -= ctx->rxFrame.payloadBuffer.length;
    /**
     * Unlike the TX branch, in RX there is only one payload buffer allowed.
     * That is why the state is switched here.
     */
    ctx->rxFrame.state = SF_SERIALMAC_CRC;
}


static void rxProcCrcCB ( struct sf_serialmac_ctx *ctx )
{
    uint16_t length = 0;
    uint16_t crcRx = 0;
    uint16_t crcCalc = 0;
    /**
     * The length is needed for calculating the CRC and for signaling the upper
     * layer.
     */
    length = UINT8_TO_UINT16 ( ctx->rxFrame.headerMemory +
                               SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN );
    /** The received CRC has to checked. */
    crcRx = UINT8_TO_UINT16 ( ctx->rxFrame.crcBuffer.memory );
    crcCalc = crc_calc_finalize ( ( uint8_t* )
                                  ctx->rxFrame.payloadBuffer.memory,
                                  length );
    if ( crcRx != crcCalc ) {
        /**
         * CRC verification failed. Inform the upper layer.
         */
        ctx->error_event( ctx, SF_SERIALMAC_ERROR_INVALID_CRC );
    }
    else
    {
        /**
         * Inform the upper layer that a frame has been completed.
         */
        ctx->rx_frame_event ( ctx, ctx->rxFrame.payloadBuffer.memory, length );
    }
    /** Regardless of the CRC, start waiting for the next frame. */
    rxInit ( ctx );
    ctx->rxFrame.state = SF_SERIALMAC_IDLE;
}


size_t sf_serialmac_ctx_size ( void )
{
    return sizeof ( struct sf_serialmac_ctx );
}


enum sf_serialmac_return sf_serialmac_init ( struct sf_serialmac_ctx *ctx,
        void *portHandle, SF_SERIALMAC_HAL_READ_FUNCTION read,
        SF_SERIALMAC_HAL_READ_WAIT_FUNCTION readWaiting,
        SF_SERIALMAC_HAL_WRITE_FUNCTION write, SF_SERIALMAC_EVENT rxEvt,
        SF_SERIALMAC_EVENT rxBufEvt, SF_SERIALMAC_EVENT rxSyncEvt,
        SF_SERIALMAC_EVENT txEvt, SF_SERIALMAC_EVENT txBufEvt,
        SF_SERIALMAC_EVENT_ERROR error_event,
        bool useInvertedLengthField)
{
    if ( !ctx ) {
        return SF_SERIALMAC_RETURN_ERROR_NPE;
    }

    #ifndef SF_SERIALMAC_INVERTED_LENGTH_RUNTIME_SEL
    if( !useInvertedLengthField )
    {
        return SF_SERIALMAC_RETURN_UNSUPPORTED_PARAMETER;
    }
    #endif

    ctx->portHandle = portHandle;
    ctx->read = read;
    ctx->readWait = readWaiting;
    ctx->write = write;
    ctx->rx_frame_event = rxEvt;
    ctx->rx_buffer_event = rxBufEvt;
    ctx->rx_sync_event = rxSyncEvt;
    ctx->tx_frame_event = txEvt;
    ctx->tx_buffer_event = txBufEvt;
    ctx->error_event = error_event;

    ctx->useInvertedLengthField = useInvertedLengthField;
    ctx->headerLength = SF_SERIALMAC_PROTOCOL_HEADER_LEN;

    if( ctx->useInvertedLengthField )
    {
        ctx->headerLength = SF_SERIALMAC_PROTOCOL_HEADER_LEN;
    }
    else
    {
        ctx->headerLength = ( SF_SERIALMAC_PROTOCOL_HEADER_LEN -
                              SF_SERIALMAC_PROTOCOL_LENGTH_FIELD_LEN );
    }

    /** Reset the context states and variables. */
    sf_serialmac_reset( ctx );

    return SF_SERIALMAC_RETURN_SUCCESS;
}

enum sf_serialmac_return sf_serialmac_reset ( struct sf_serialmac_ctx *ctx )
{
  if ( !ctx ) {
    return SF_SERIALMAC_RETURN_ERROR_NPE;
  }

  /** Reset the context states and variables. */
  txInit ( ctx );
  ctx->txFrame.state = SF_SERIALMAC_IDLE;
  rxInit ( ctx );
  ctx->rxFrame.state = SF_SERIALMAC_IDLE;

  return SF_SERIALMAC_RETURN_SUCCESS;
}

enum sf_serialmac_return sf_serialmac_tx_frame_start ( struct sf_serialmac_ctx
        *ctx, size_t len )
{
    if ( !ctx ) {
        return SF_SERIALMAC_RETURN_ERROR_NPE;
    }
    if ( ctx->txFrame.state != SF_SERIALMAC_IDLE ) {
        return SF_SERIALMAC_RETURN_ERROR_FRM_PENDING;
    }

    /** Write frame length into the length field of the frame header */
    UINT16_TO_UINT8 ( ctx->txFrame.headerMemory +
                      SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN,
                      len );

    if( ctx->useInvertedLengthField )
    {
      /** Write inverted frame length into the inverted length
       *  field of the frame header */
      UINT16_TO_UINT8 ( ctx->txFrame.headerMemory +
                        SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN +
                        SF_SERIALMAC_PROTOCOL_LENGTH_FIELD_LEN,
                        ~len );
    }

    ctx->txFrame.remains = len;
    ctx->txFrame.state = SF_SERIALMAC_HEADER;
    return SF_SERIALMAC_RETURN_SUCCESS;
}


enum sf_serialmac_return sf_serialmac_tx_frame_append ( struct sf_serialmac_ctx
        *ctx, const uint8_t *frmBufLoc, size_t frmBufSize )
{
    size_t buff = 0;
    if ( !ctx || !frmBufLoc ) {
        return SF_SERIALMAC_RETURN_ERROR_NPE;
    }
    /**
     * Check if a payload buffer has been assigned before and is not
     * completely processed. And prevent upper layer to append more payload
     * before the previous frame has been started.
     */
    if ( ctx->txFrame.payloadBuffer.memory || ctx->txFrame.state == SF_SERIALMAC_CRC ) {
        return SF_SERIALMAC_RETURN_ERROR_RW_PENDING;
    }
    buff = frmBufSize > ctx->txFrame.remains ? ctx->txFrame.remains :
           frmBufSize;
    /**
     * By assigning memory to the payload buffer the payload is transmitted with
     * the next call to sf_serialmac_halTxCb() and consecutive tries
     * to assign payload buffers are prevented untill the currently assigned
     * buffer has been processed.
     */
    initBuffer ( &ctx->txFrame.payloadBuffer, ( uint8_t* ) frmBufLoc, buff,
                 txProcPayloadCB );
    return SF_SERIALMAC_RETURN_SUCCESS;
}


enum sf_serialmac_return sf_serialmac_tx_frame ( struct sf_serialmac_ctx *ctx,
        size_t frmLen, const uint8_t *frmBufLoc, size_t frmBufSize )
{
  if( !ctx || !frmBufLoc ) {
    return SF_SERIALMAC_RETURN_ERROR_NPE;
  }

    sf_serialmac_tx_frame_start ( ctx, frmLen );
    return sf_serialmac_tx_frame_append ( ctx, frmBufLoc, frmBufSize );
}


enum sf_serialmac_return sf_serialmac_rx_frame ( struct sf_serialmac_ctx *ctx,
        uint8_t *frmBufLoc, size_t frmBufSize )
{
    if ( !ctx || !frmBufLoc ) {
        return SF_SERIALMAC_RETURN_ERROR_NPE;
    }
    if ( ctx->rxFrame.payloadBuffer.memory != NULL ) {
        return SF_SERIALMAC_RETURN_ERROR_RW_PENDING;
    }
    if ( frmBufSize < ctx->rxFrame.remains ) {
        return SF_SERIALMAC_RETURN_ERROR_BUFFER;
    }
    initBuffer ( &ctx->rxFrame.payloadBuffer, frmBufLoc, ctx->rxFrame.remains,
                 rxProcPayloadCB );
    memset ( ( void * ) ctx->rxFrame.payloadBuffer.memory, 0,
             ctx->rxFrame.payloadBuffer.length );
    return SF_SERIALMAC_RETURN_SUCCESS;
}


enum sf_serialmac_return sf_serialmac_hal_tx_callback ( struct sf_serialmac_ctx
        *ctx )
{
    enum sf_serialmac_return ret = SF_SERIALMAC_RETURN_SUCCESS;

    /** Do nothing if there is no context. */
    if ( !ctx ) {
        return SF_SERIALMAC_RETURN_ERROR_NPE;
    }

    /**
     * Try to process a whole frame at once before handing control back to
     * main()
     */
    do {
        switch ( ctx->txFrame.state ) {
        case SF_SERIALMAC_IDLE:
            /** Nothing to do. */
            break;
        case SF_SERIALMAC_HEADER:
            ret = tx ( ctx, &ctx->txFrame.headerBuffer, NULL );
            break;
        case SF_SERIALMAC_PAYLOAD:
            /**
             * If a write buffer has been assigned process it.
             */
            if ( ctx->txFrame.payloadBuffer.memory ) {
                /**
                 * Send the payload and calculate the CRC.
                 * The second parameter contains the payload, the last parameter
                 * is for storing the CRC which is calculated by tx().
                 */
                ret = tx ( ctx, &ctx->txFrame.payloadBuffer,
                           ctx->txFrame.crcMemory );
            }
            else {
              ret = SF_SERIALMAC_RETURN_ERROR_NPE;
            }
            break;
        case SF_SERIALMAC_CRC:
            ret = tx ( ctx, &ctx->txFrame.crcBuffer, NULL );
            break;
        }
    } while (
        /**
         * In case a frame has been processed give other processes a
         * chance to jump in
         */
        ctx->txFrame.state != SF_SERIALMAC_IDLE &&
        /** If the last action has been successful we proceed */
        ( ret == SF_SERIALMAC_RETURN_SUCCESS
          /** This is a workaround for slow serial ports. */
          || ret == SF_SERIALMAC_RETURN_ERROR_HAL_SLOW ) );

    return ret;
}


enum sf_serialmac_return sf_serialmac_hal_rx_callback ( struct sf_serialmac_ctx
        *ctx )
{
    size_t bytesWaiting = 0;
    enum sf_serialmac_return ret = SF_SERIALMAC_RETURN_SUCCESS;

    /** Do nothing if there is no context. */
    if ( !ctx ) {
        return SF_SERIALMAC_RETURN_ERROR_NPE;
    }



    /**
     * Try to process a whole frame at once before handing control back to
     * main()
     */
    do {
        /** Is there anything to receive? */
        if ( ( bytesWaiting = ctx->readWait ( ctx->portHandle ) ) > 0 ) {

            switch ( ctx->rxFrame.state ) {
            case SF_SERIALMAC_IDLE:
                ret = rx ( ctx, &ctx->rxFrame.headerBuffer,
                           SF_SERIALMAC_PROTOCOL_SYNC_WORD_LEN );
                /** FIXME: this only works for sync words of 1 byte length! */
                if ( ctx->rxFrame.headerBuffer.memory[0] == ( uint8_t )
                        SF_SERIALMAC_PROTOCOL_SYNC_WORD ) {
                    ctx->rxFrame.state = SF_SERIALMAC_HEADER;
                    /** Inform the upper layer that a sync byte has been received. */
                    ctx->rx_sync_event( ctx, NULL, 0U);
                } else {
                    initBuffer ( &ctx->rxFrame.headerBuffer, ( uint8_t* )
                                 &ctx->rxFrame.headerMemory,
                                 ctx->headerLength,
                                 rxProcHeaderCB );
                    /** The received byte was no sync byte. Inform the upper layer. */
                    ctx->error_event( ctx, SF_SERIALMAC_ERROR_INVALID_SYNC_BYTE );
                }

                break;
            case SF_SERIALMAC_HEADER:
                ret = rx ( ctx, &ctx->rxFrame.headerBuffer, bytesWaiting );
                break;
            case SF_SERIALMAC_PAYLOAD:
                ret = rx ( ctx, &ctx->rxFrame.payloadBuffer, bytesWaiting );
                break;
            case SF_SERIALMAC_CRC:
                ret = rx ( ctx, &ctx->rxFrame.crcBuffer, bytesWaiting );
                break;
            default:
                /** This should never happen, but if it does we can catch it. */
                ret = SF_SERIALMAC_RETURN_ERROR_EXCEPTION;
            }
        }
    } while (
        /** Hand the control back to main() if there is nothing to do */
        bytesWaiting  > 0
        /** or in case of errors */
        && ret == SF_SERIALMAC_RETURN_SUCCESS
        /**
         * or if a whole frame has been processed (also to prevent DOS
         * attacks).
         */
        && ctx->rxFrame.state != SF_SERIALMAC_IDLE );

    return ret;
}


void sf_serialmac_entry ( struct sf_serialmac_ctx *ctx )
{
    /***************************************************************************
     * TX
     */
    sf_serialmac_hal_tx_callback ( ctx );

    /***************************************************************************
     * RX
     */
    sf_serialmac_hal_rx_callback ( ctx );
}

#ifdef __cplusplus
}
#endif
