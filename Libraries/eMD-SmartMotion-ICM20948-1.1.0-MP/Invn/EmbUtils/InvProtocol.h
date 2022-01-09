/*
    Copyright (c) 2014-2015 InvenSense Inc. Portions Copyright (c) 2014-2015 Movea. All rights reserved.

    This software, related documentation and any modifications thereto (collectively "Software") is subject
    to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
    other intellectual property rights laws.

    InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
    and any use, reproduction, disclosure or distribution of the Software without an express license
    agreement from InvenSense is strictly prohibited.
*/

/** @defgroup InvProtocol InvProtocol
	@brief Functions to decode and format frames according to the InvProtocol
    @ingroup EmbUtils
    @{
*/

#ifndef _INV_PROTOCOL_H_
#define _INV_PROTOCOL_H_

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief InvProtocol error definitions states */
enum InvProtocolError {
	INVPROTOCOL_OK				 	= 0,	/** no error */
	INVPROTOCOL_NOK 				= -1,	/** error */
	INVPROTOCOL_INCOMPLETE		 	= -2,	/** decoding or formating is in progress */
	INVPROTOCOL_INVALID_HEADER  	= -3,	/** received header is invalid */
	INVPROTOCOL_INVALID_CRC	 		= -4,	/** received crc is invalid */
	INVPROTOCOL_INVALID_SIZE	 	= -5,	/** received size is invalid */
};

/** @brief InvProtocol decoder states */
typedef struct {
	uint8_t		state;
	uint16_t	rcrc;
	uint16_t	ccrc;
	uint16_t	count;
} InvProtocolDecoder;

/** @brief Initialize the InvProtocol decoder states.
	@param self		placeholder to InvProtocol formater states

*/
extern void InvProtocolDecoder_init(InvProtocolDecoder *self);

/** @brief check input buffer for a valid frame

	This function is intented to be called in a loop feeding one char at a time.
	Argument are used as placeholder. Take care to provided same address or value
	while InvProtocolDecoder_processByte() returns INVPROTOCOL_INCOMPLETE

	You must call InvProtocolDecoder_init() before calling InvProtocol_decodeByte()
	to reset the decoder state machine (usually, the first time, or after on error or success)

	@param[in]	byte input byte to process
	@param[out] type the type of received frame
	@param[out] code the code of received frame
	@param[out]	size the size of received arguments

	@param[out] arg  buffer for received arguments (can be null)
	@param[in]	max	 maximum size of argument buffer placeholder

	@return 	INVPROTOCOL_OK a frame was received and output are set
				INVPROTOCOL_INCOMPLETE a full frame was no received yet, this is not an error.
				INVPROTOCOL_INVALID_CRC CRCs don't match
				INVPROTOCOL_INVALID_SIZE size of argument placeholder was too small to retreive all argument cata
				INVPROTOCOL_INVALID_HEADER the header was not valid
*/
extern int InvProtocolDecoder_processByte(InvProtocolDecoder *self,
                uint8_t 	byte,
                uint8_t 	*type,
                uint8_t 	*code,
                size_t 	*size,
                void 		*arg,
                size_t 		max
                                         );

/** @brief parse input buffer for valid a valid frame

	@param[in]	input 	input buffer to process
	@param[in]	sinput  size of input buffer
	@param[out] type the type of received frame
	@param[out] code the code of received frame
	@param[out]	size the size of received arguments
	@param[out] arg  buffer for received arguments (can be null)
	@param[in]	max	 maximum size of argument buffer placeholder
	@param[out]	idx	 current input buffer idx when process stop

	@return 	INVPROTOCOL_OK he value indicates number of
				INVPROTOCOL_INCOMPLETE all input buffer was processed but no valid frame was decoded
				INVPROTOCOL_INVALID_CRC a frame was but CRCs don't match
				INVPROTOCOL_INVALID_SIZE size of argument placeholder was too small to retreive all argument cata
				INVPROTOCOL_INVALID_HEADER the header was not valid
*/
extern int InvProtocolDecoder_decodeBuffer(
        const uint8_t 	*input,
        size_t 		sinput,
        uint8_t 	*type,
        uint8_t 	*code,
        size_t 	*scontent,
        void 		*content,
        size_t 		max,
        size_t 	*idx
);

/** @brief InvProtocol formater states */
typedef struct {
	uint8_t		state;
	uint16_t	ccrc;
	uint16_t	count;
} InvProtocolFormater;

/** @brief Initialize states for InvProtocol formater object
	@param self		placeholder to InvProtocol formater states
*/
extern void InvProtocolFormater_init(InvProtocolFormater *self);

/** @brief Format a frame following the InvProtocol into a buffer

	This function is intended to be called in a loop.
	It allows to send bytes imediatly without using temporary buffer.
	Input can be read at different iterations. Take to provide same inputs
	while InvProtocol_formatByte() returns INVPROTOCOL_INCOMPLETE.
	You must call InvProtocolFormater_init() to reset states, usually the first
	time or after success or error.

	@param 	   self	placeholder to InvProtocol formater states
	@param[in] type type
	@param[in] code code
	@param[in] size size of arguments
	@param[in] arg  arguments to send
	@return InvProtocolError
*/
extern int InvProtocolFormater_processByte(InvProtocolFormater *self,
                uint8_t 		type,
                uint8_t 		code,
                const void 	*data,
                size_t 			size,
                uint8_t 		*outbyte
                                          );

/** @brief Format a frame following the InvProtocol into a buffer
	@param type[in] type
	@param code[in] code
	@param size[in] size of arguments
	@param arg[in]  arguments to send
	@return > 0 indicating the size of the outputbuffer to transmit or InvProtocolError
*/
extern int InvProtocolFormater_formatBuffer(
        uint8_t 		type,
        uint8_t 		code,
        const void 	*arg,
        size_t 			size,
        uint8_t 		*outbuffer,
        size_t 			soutbuffer
);

#ifdef __cplusplus
}
#endif

#endif

/** @} */
