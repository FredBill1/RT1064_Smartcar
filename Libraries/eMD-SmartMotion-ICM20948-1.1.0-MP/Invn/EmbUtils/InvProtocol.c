/*
    Copyright (c) 2014-2015 InvenSense Inc. Portions Copyright (c) 2014-2015 Movea. All rights reserved.

    This software, related documentation and any modifications thereto (collectively "Software") is subject
    to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
    other intellectual property rights laws.

    InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
    and any use, reproduction, disclosure or distribution of the Software without an express license
    agreement from InvenSense is strictly prohibited.
*/

#include "InvProtocol.h"

enum InvProtocolState {
	INVPROTOCOL_STATE_HEADER,
	INVPROTOCOL_STATE_TYPE,
	INVPROTOCOL_STATE_CODE,
	INVPROTOCOL_STATE_SIZE0, INVPROTOCOL_STATE_SIZE1,
	INVPROTOCOL_STATE_ARG,
	INVPROTOCOL_STATE_CRC0, INVPROTOCOL_STATE_CRC1
};

static const uint8_t sInvFrameHeader[] = {(uint8_t)0x55, (uint8_t)0xaa, (uint8_t)0x55, (uint8_t)0xaa};

#define INVPROTOCOL_HEADER_SIZE		(sizeof(sInvFrameHeader))
#define INVPROTOCOL_TYPE_SIZE		(sizeof(uint8_t))
#define INVPROTOCOL_CODE_SIZE		(sizeof(uint8_t))
#define INVPROTOCOL_DATA_SIZE		(sizeof(uint16_t))
#define INVPROTOCOL_CRC_SIZE		(sizeof(uint16_t))

static __inline uint16_t cksum_reset(void)
{
	return 1;
}

static __inline uint16_t cksum_update(uint16_t chk, uint8_t byte)
{
	return (chk << 1) + chk + byte;
}

void InvProtocolDecoder_init(InvProtocolDecoder *self)
{
	self->state = INVPROTOCOL_STATE_HEADER;
	self->rcrc = 0;
	self->ccrc = 0;
	self->count = 0;
}

int InvProtocolDecoder_processByte(InvProtocolDecoder *self,
                                   uint8_t 	byte,
                                   uint8_t 	*type,
                                   uint8_t 	*code,
                                   size_t *size,
                                   void 		*data,
                                   size_t maxSize
                                  )
{
	int rc = INVPROTOCOL_NOK;

	switch (self->state) {
	case INVPROTOCOL_STATE_HEADER: {
			if (byte != sInvFrameHeader[self->count++]) {
				rc = INVPROTOCOL_INVALID_HEADER;
				break;
			}

			if (self->count == INVPROTOCOL_HEADER_SIZE) {
				self->state = INVPROTOCOL_STATE_TYPE;
			}

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_TYPE: {
			*type = byte;
			self->state = INVPROTOCOL_STATE_CODE;

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_CODE: {
			*code = byte;
			self->state = INVPROTOCOL_STATE_SIZE0;

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_SIZE0: {
			*size = byte;
			self->state = INVPROTOCOL_STATE_SIZE1;

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_SIZE1: {
			*size |= (uint16_t)byte << 8;
			self->state = (*size != 0) ? INVPROTOCOL_STATE_ARG : INVPROTOCOL_STATE_CRC0;
			self->ccrc = cksum_reset();
			self->count = 0;

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_ARG: {
			if (self->count < maxSize && data != 0) {
				uint8_t *ptr = data;
				ptr[self->count] = (uint8_t)byte;
			}

			self->count++;
			self->ccrc = cksum_update(self->ccrc, (uint8_t)byte);

			if (self->count == *size) {
				self->state = INVPROTOCOL_STATE_CRC0;
			}

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_CRC0: {
			self->rcrc = (uint8_t)byte;
			self->state = INVPROTOCOL_STATE_CRC1;

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_CRC1: {
			self->rcrc |= (uint16_t)byte << 8;

			if (self->rcrc != self->ccrc) {
				rc = INVPROTOCOL_INVALID_CRC;
				break;
			}

			if (self->count > maxSize) {
				rc = INVPROTOCOL_INVALID_SIZE;
				break;
			}

			rc = INVPROTOCOL_OK;
			break;
		}

	default:
		break;
	}

	InvProtocolDecoder_init(self);

	return rc;
}

int InvProtocolDecoder_decodeBuffer(
        const uint8_t 	*input,
        size_t 		sinput,
        uint8_t 	*type,
        uint8_t 	*code,
        size_t 	*scontent,
        void 		*content,
        size_t 		max,
        size_t 	*idx
)
{
	InvProtocolDecoder state;
	int rc = INVPROTOCOL_INCOMPLETE;

	InvProtocolDecoder_init(&state);

	for (*idx = 0; *idx < sinput && rc == INVPROTOCOL_INCOMPLETE; ++(*idx)) {
		rc = InvProtocolDecoder_processByte(&state, input[*idx], type, code, scontent,
		                                    content, max);
	}

	return rc;
}

void InvProtocolFormater_init(InvProtocolFormater *self)
{
	self->state = INVPROTOCOL_STATE_HEADER;
	self->ccrc 	= 0;
	self->count = 0;
}

int InvProtocolFormater_processByte(InvProtocolFormater *self,
                                    uint8_t 		type,
                                    uint8_t 		code,
                                    const void 	*data,
                                    size_t 			size,
                                    uint8_t 		*outbyte
                                   )
{
	switch (self->state) {
	case INVPROTOCOL_STATE_HEADER: {
			*outbyte = sInvFrameHeader[self->count++];

			if (self->count == INVPROTOCOL_HEADER_SIZE) {
				self->state = INVPROTOCOL_STATE_TYPE;
			}

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_TYPE: {
			*outbyte = type;
			self->state = INVPROTOCOL_STATE_CODE;
			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_CODE: {
			*outbyte = code;
			self->state = INVPROTOCOL_STATE_SIZE0;
			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_SIZE0: {
			*outbyte = ((uint16_t)size & (uint16_t)0x00FF);
			self->state = INVPROTOCOL_STATE_SIZE1;
			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_SIZE1: {
			*outbyte = ((uint16_t)size & (uint16_t)0xFF00) >> 8;
			self->state = (size != 0) ? INVPROTOCOL_STATE_ARG : INVPROTOCOL_STATE_CRC0;
			self->ccrc = cksum_reset();
			self->count = 0;

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_ARG: {
			*outbyte = ((const uint8_t *)data)[self->count++];
			self->ccrc = cksum_update(self->ccrc, *outbyte);

			if (self->count == (uint16_t)size) {
				self->state = INVPROTOCOL_STATE_CRC0;
			}

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_CRC0: {
			*outbyte = (self->ccrc & (uint16_t)0x00FF);
			self->state = INVPROTOCOL_STATE_CRC1;

			return INVPROTOCOL_INCOMPLETE;
		}
	case INVPROTOCOL_STATE_CRC1: {
			*outbyte = (self->ccrc & (uint16_t)0xFF00) >> 8;

			return INVPROTOCOL_OK;
		}

	default:
		break;
	}

	return INVPROTOCOL_NOK;
}

int InvProtocolFormater_formatBuffer(
        uint8_t 		type,
        uint8_t 		code,
        const void 	*data,
        size_t			size,
        uint8_t 		*outbuffer,
        size_t 			soutbuffer
)
{
	InvProtocolFormater state;
	size_t i;
	int rc = INVPROTOCOL_INCOMPLETE;

	InvProtocolFormater_init(&state);

	for (i = 0; i < soutbuffer && rc == INVPROTOCOL_INCOMPLETE; ++i) {
		rc = InvProtocolFormater_processByte(&state, type, code, data, size,
		                                     &outbuffer[i]);
	}

	return (rc == INVPROTOCOL_OK) ? (int)i : rc;
}
