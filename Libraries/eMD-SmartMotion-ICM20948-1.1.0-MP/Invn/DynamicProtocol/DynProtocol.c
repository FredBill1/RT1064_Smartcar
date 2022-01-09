/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "DynProtocol.h"

#include "EmbUtils/Message.h"
#include "EmbUtils/DataConverter.h"
#include "EmbUtils/InvError.h"

#include <string.h>

/** A frame transfer have the format bellow
	<EVENT_TYPE|IDD_WRAPPER_GRP> <PACKET>
	 According to the Event Type:
	- CMD=0
	- RESPONSE=1
	- ASYNC_EVENT=2
	Packet should contains :
	- <CMD> <ARG>
	- <CMD> <ERROR_STATUS> <ARG>
	- <SENSOR_STATUS> <SENSOR_ID> <TIMESTAMP> <DATA>
	*/

#define MAX_EXPECTED_PAYLOAD (sizeof((*(DynProtocol_t *)(0)).decode_state_machine.tmp_buffer))

#define EVENT_TYPE_CMD   (0 << 6)
#define EVENT_TYPE_RESP  (1 << 6)
#define EVENT_TYPE_ASYNC (2 << 6)
#define EVENT_TYPE_MASK 0xC0

#define PROTOCOL_ACCELEROMETER_PRECISION     11
#define PROTOCOL_MAGNETOMETER_PRECISION       4
#define PROTOCOL_GYROSCOPE_PRECISION          4
#define PROTOCOL_QUATERNION_PRECISION        14
#define PROTOCOL_RAW_PRECISION                0
#define PROTOCOL_ORIENTATION_PRECISION        6
#define PROTOCOL_TEMPERATURE_PRECISION        8
#define PROTOCOL_CUSTOM_PRESSURE_PRECISION    8
#define PROTOCOL_HRM_PRECISION                7

#define PROTOCOL_HEADING_ACCURACY_PRECISION   7

#define DYN_PROTOCOL_QX_TO_QY(value, qx, qy)	((qx >= qy) ? (value >> (qx-qy)) : (value << (qy-qx)) )

static inline int DynProtocol_decodeVect16QxToQy(const uint8_t * bytes, unsigned len, int qxIn, int qxOut,  int32_t * out)
{
	unsigned i;
	for(i = 0; i < len; ++i) {
		int32_t x = (int32_t)(((int8_t *)bytes)[2*i+1] << 8) | bytes[2*i];
		out[i] = (int32_t)DYN_PROTOCOL_QX_TO_QY(x, qxIn ,qxOut);
	}

	return 2*len;
}

static inline int DynProtocol_encodeQxToQyVect16(const int32_t * in, unsigned len, int qxIn, int qxOut, uint8_t * bytes)
{
	unsigned i;
	for(i = 0; i < len; ++i) {
		int16_t x = (int16_t)DYN_PROTOCOL_QX_TO_QY(in[i], qxIn ,qxOut);
		bytes[2*i]   = (uint8_t)((uint16_t)x & 0xFF);
		bytes[2*i+1] = (uint8_t)(((uint16_t)x & 0xFF00) >> 8U);
	}

	return 2*len;
}

static int DynProtocol_getPrecision(DynProtocol_t * self, int sensor_id)
{
	int QxIn;

	switch (sensor_id)
	{
	case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:
	case DYN_PRO_SENSOR_TYPE_GRAVITY:
	case DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION:
		QxIn = self->precision.acc;
		break;
	case DYN_PRO_SENSOR_TYPE_GYROSCOPE:
	case DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE:
		QxIn = self->precision.gyro;
		break;
	case DYN_PRO_SENSOR_TYPE_MAGNETOMETER:
	case DYN_PRO_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		QxIn = PROTOCOL_MAGNETOMETER_PRECISION;
		break;
	case DYN_PRO_SENSOR_TYPE_3AXIS:
	case DYN_PRO_SENSOR_TYPE_ROTATION_VECTOR:
	case DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR:
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_0:
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_1:
	case DYN_PRO_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		QxIn = PROTOCOL_QUATERNION_PRECISION;
		break;
	case DYN_PRO_SENSOR_TYPE_ORIENTATION:
		QxIn = PROTOCOL_ORIENTATION_PRECISION;
		break;
	case DYN_PRO_SENSOR_TYPE_AMBIENT_TEMPERATURE:
		QxIn = PROTOCOL_TEMPERATURE_PRECISION;
		break;
	default:
		QxIn = PROTOCOL_RAW_PRECISION;
		break;
	}

	return QxIn;
}

static int16_t DynProtocol_getPayload(DynProtocol_t * self)
{
	const uint8_t eventType = self->decode_state_machine.event_type;
	const enum DynProtocolEid cmdId = (enum DynProtocolEid)self->decode_state_machine.cmd_id;

	if(self->decode_state_machine.received_size == 0) {
		switch(eventType) {
		case EVENT_TYPE_CMD:
			switch(cmdId) {
			case DYN_PROTOCOL_EID_PROTOCOLVERSION:    return 0;
			case DYN_PROTOCOL_EID_GET_FW_INFO:        return 0;
			case DYN_PROTOCOL_EID_WHO_AM_I:           return 0;
			case DYN_PROTOCOL_EID_RESET:              return 0;
			case DYN_PROTOCOL_EID_SETUP:              return 0;
			case DYN_PROTOCOL_EID_CLEANUP:            return 0;
			case DYN_PROTOCOL_EID_SELF_TEST:          return 1;
			case DYN_PROTOCOL_EID_PING_SENSOR:        return 1;
			case DYN_PROTOCOL_EID_START_SENSOR:       return 1;
			case DYN_PROTOCOL_EID_STOP_SENSOR:        return 1;
			case DYN_PROTOCOL_EID_SET_SENSOR_PERIOD:  return 1 + 4;
			case DYN_PROTOCOL_EID_SET_SENSOR_TIMEOUT: return 1 + 4;
			case DYN_PROTOCOL_EID_FLUSH_SENSOR:       return 1;
			case DYN_PROTOCOL_EID_SET_SENSOR_MMATRIX: return 1 + 1 + (9 * 4); /* sensor + cfg_type + 3x3 q30 */
			case DYN_PROTOCOL_EID_GET_SENSOR_DATA:    return 1;
			case DYN_PROTOCOL_EID_GET_SW_REG:         return 1 + 1;
			case DYN_PROTOCOL_EID_SET_SENSOR_CFG:     return 1 + 1 + 1 + 64; // sensor + size + cfg_type + max_data_size
			case DYN_PROTOCOL_EID_GET_SENSOR_CFG:     return 1 + 1; // sensor + cfg_type
			case DYN_PROTOCOL_EID_SET_SENSOR_BIAS:    return 1 + 1 + (3 * 2); /* sensor + cfg_type + bias (x,y,z) in Qx */
			case DYN_PROTOCOL_EID_GET_SENSOR_BIAS:    return 1 + 1; /* sensor + cfg_type */
			default:
				break;
			}
			break;

		case EVENT_TYPE_RESP:
			switch(cmdId) {
			case DYN_PROTOCOL_EID_PROTOCOLVERSION:    return 16;
			case DYN_PROTOCOL_EID_GET_FW_INFO:        return 16;
			case DYN_PROTOCOL_EID_WHO_AM_I:           return 1;
			case DYN_PROTOCOL_EID_RESET:              return 1;
			case DYN_PROTOCOL_EID_SETUP:              return 1;
			case DYN_PROTOCOL_EID_CLEANUP:            return 1;
			case DYN_PROTOCOL_EID_SELF_TEST:          return 1;
			case DYN_PROTOCOL_EID_PING_SENSOR:        return 1;
			case DYN_PROTOCOL_EID_START_SENSOR:       return 1;
			case DYN_PROTOCOL_EID_STOP_SENSOR:        return 1;
			case DYN_PROTOCOL_EID_SET_SENSOR_PERIOD:  return 1;
			case DYN_PROTOCOL_EID_SET_SENSOR_TIMEOUT: return 1;
			case DYN_PROTOCOL_EID_FLUSH_SENSOR:       return 1;
			case DYN_PROTOCOL_EID_SET_SENSOR_MMATRIX: return 1;
			case DYN_PROTOCOL_EID_GET_SENSOR_DATA:    return 1+1+1+4+64; // rc + sensorStatus + sensorId + timestamp[0-3] + max_data_size
			case DYN_PROTOCOL_EID_GET_SW_REG:         return 1;
			case DYN_PROTOCOL_EID_SET_SENSOR_CFG:     return 1;
			case DYN_PROTOCOL_EID_GET_SENSOR_CFG:     return 1+1+1+1+64; // rc + sensor + size + cfg_type + max_data_size
			case DYN_PROTOCOL_EID_SET_SENSOR_BIAS:    return 1; /* rc */
			case DYN_PROTOCOL_EID_GET_SENSOR_BIAS:    return 1 + 1 + 1 + (3 * 2); /* rc + precision + cfg_type + bias (x,y,z) in Qx */
			default:
				break;
			}
			break;

		case EVENT_TYPE_ASYNC:
			switch(cmdId) {
			case DYN_PROTOCOL_EID_NEW_SENSOR_DATA:
				/* need at least two more byte to determine payload (sensor status + sensor id) */
				return 2;
			default:
				break;
			}
			break;

		default:
			break;
		}
	}
	else {

		/* Payload content : 
		 *   - byte 0 <sensor status> 
		 *   - byte 1 <sensor id> 
		 *   - <sensor data> 
		 */
		const uint8_t sensor_id = self->decode_state_machine.tmp_buffer[1];

		switch(eventType) {
		case EVENT_TYPE_ASYNC:
			switch(sensor_id) {
			case DYN_PRO_SENSOR_TYPE_RESERVED:
				return self->decode_state_machine.received_size;
			case DYN_PRO_SENSOR_TYPE_CUSTOM_PRESSURE:
				return 1+1+4+16;
			case DYN_PRO_SENSOR_TYPE_HIGH_RATE_GYRO:
				return 1+1+6;
			case DYN_PRO_SENSOR_TYPE_RAW_ACCELEROMETER:
			case DYN_PRO_SENSOR_TYPE_RAW_MAGNETOMETER:
			case DYN_PRO_SENSOR_TYPE_RAW_GYROSCOPE:
			case DYN_PRO_SENSOR_TYPE_OIS:
				return 1+1+4+6;
			case DYN_PRO_SENSOR_TYPE_FSYNC_EVENT:
				return 1+1+4+2;
			case DYN_PRO_SENSOR_TYPE_EIS:
				return 1+1+4+6+6+2;
			case DYN_PRO_SENSOR_TYPE_AMBIENT_TEMPERATURE:
				return 1+1+4+2;
			case DYN_PRO_SENSOR_TYPE_RAW_TEMPERATURE:
				return 1+1+4+4;
                        case DYN_PRO_SENSOR_TYPE_RAW_PPG:
                                return 1+1+4+4+1;
			case DYN_PRO_SENSOR_TYPE_ENERGY_EXPENDITURE:
				return 1+1+4+16;
			case DYN_PRO_SENSOR_TYPE_DISTANCE:
				return 1+1+4+8;
			case DYN_PRO_SENSOR_TYPE_SLEEP_ANALYSIS:
				return 1+1+4+25;
			case DYN_PRO_SENSOR_TYPE_BAC_EXTENDED:
				return 1+1+4+4;
			case DYN_PRO_SENSOR_TYPE_BAC_STATISTICS:
				return 1+1+4+44;
			case DYN_PRO_SENSOR_TYPE_FLOOR_CLIMB_COUNTER:
				return 1+1+4+8;
			case DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION:
			case DYN_PRO_SENSOR_TYPE_GRAVITY:
			case DYN_PRO_SENSOR_TYPE_ORIENTATION:
				return 1+1+4+6+1;
			case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:
			case DYN_PRO_SENSOR_TYPE_GYROSCOPE:
			case DYN_PRO_SENSOR_TYPE_MAGNETOMETER:
				return 1+1+4+6+1;
			case DYN_PRO_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			case DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE:
				return 1+1+4+12+1;
			case DYN_PRO_SENSOR_TYPE_PRED_QUAT_0:
			case DYN_PRO_SENSOR_TYPE_PRED_QUAT_1:
			
			case DYN_PRO_SENSOR_TYPE_3AXIS:
			case DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR:
				return 1+1+4+8+1;
			case DYN_PRO_SENSOR_TYPE_ROTATION_VECTOR:
			case DYN_PRO_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
				return 1+1+4+10+1;
			case DYN_PRO_SENSOR_TYPE_B2S:
			case DYN_PRO_SENSOR_TYPE_SHAKE:
			case DYN_PRO_SENSOR_TYPE_DOUBLE_TAP:
			case DYN_PRO_SENSOR_TYPE_SEDENTARY_REMIND:
			case DYN_PRO_SENSOR_TYPE_SMD:
			case DYN_PRO_SENSOR_TYPE_STEP_DETECTOR:
			case DYN_PRO_SENSOR_TYPE_TILT_DETECTOR:
			case DYN_PRO_SENSOR_TYPE_WAKE_GESTURE:
			case DYN_PRO_SENSOR_TYPE_GLANCE_GESTURE:
			case DYN_PRO_SENSOR_TYPE_PICK_UP_GESTURE:
			case DYN_PRO_SENSOR_TYPE_PRESSURE:
			case DYN_PRO_SENSOR_TYPE_LIGHT:
				return 1+1+4+4;
			case DYN_PRO_SENSOR_TYPE_WOM:
			case DYN_PRO_SENSOR_TYPE_BAC:
				return 1+1+4+1+4;
			case DYN_PRO_SENSOR_TYPE_STEP_COUNTER:
				return 1+1+4+4+4;
			case DYN_PRO_SENSOR_TYPE_PROXIMITY:
				return 1+1+4+2;
			case DYN_PRO_SENSOR_TYPE_CUSTOM0:
			case DYN_PRO_SENSOR_TYPE_CUSTOM1:
			case DYN_PRO_SENSOR_TYPE_CUSTOM2:
			case DYN_PRO_SENSOR_TYPE_CUSTOM3:
			case DYN_PRO_SENSOR_TYPE_CUSTOM4:
			case DYN_PRO_SENSOR_TYPE_CUSTOM5:
			case DYN_PRO_SENSOR_TYPE_CUSTOM6:
			case DYN_PRO_SENSOR_TYPE_CUSTOM7:
				return 1+1+4+65;
			case DYN_PRO_SENSOR_TYPE_HEART_RATE:
				return 1+1+4+2+1+1;
			case DYN_PRO_SENSOR_TYPE_HRV:
				return 1+1+4+1+1+(2*4);
                          
			default:
				/* undefined for now */
				return -1;
			}

		default:
			/* do not need to update expected payload */
			return self->decode_state_machine.expected_size;
		}
	}

	INV_MSG(INV_MSG_LEVEL_WARNING, "DynProtocol: returned payload is -1");

	return -1;
}

int16_t inv_dc_little8_to_int16(const uint8_t * little8)
{
        int16_t x = 0;

        x |= ((int16_t)little8[1] << 8);
        x |= ((int16_t)little8[0]);

        return x;
}

static int DynProtocol_decodeSensorEvent(DynProtocol_t * self, const uint8_t * buffer, uint16_t size,
		DynProtocolEdata_t *edata, enum DynProtocolEtype etype)
{
	uint16_t idx = 0, i=0;
	uint32_t *sensorStatus;
	VSensorDataAny *vSensordata;

	switch(etype) {
	case DYN_PROTOCOL_ETYPE_RESP:
		sensorStatus = &edata->d.response.sensorData.status;
		vSensordata = &edata->d.response.sensorData.vdata;
		break;
	case DYN_PROTOCOL_ETYPE_ASYNC:
		sensorStatus = &edata->d.async.sensorEvent.status;
		vSensordata = &edata->d.async.sensorEvent.vdata;
		break;
	case DYN_PROTOCOL_ETYPE_CMD:
	default:
		return INV_ERROR_BAD_ARG;
	}

	memset(vSensordata, 0, sizeof(VSensorDataAny));

	*sensorStatus    = (uint32_t)(buffer[idx] & 0x03);
	idx += 1;
	edata->sensor_id = (int)(buffer[idx]);
	idx += 1;

	vSensordata->base.timestamp = (uint32_t)inv_dc_little8_to_int32(&buffer[idx]);
	idx += 4;

	switch(edata->sensor_id) {
	case DYN_PRO_SENSOR_TYPE_RESERVED:
		break;
	case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:
	case DYN_PRO_SENSOR_TYPE_GRAVITY:
	case DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, self->precision.acc, 16, (int32_t*)&vSensordata->data.u32[0]);
		vSensordata->base.meta_data = (uint32_t)buffer[idx++];
		break;
	case DYN_PRO_SENSOR_TYPE_MAGNETOMETER:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, PROTOCOL_MAGNETOMETER_PRECISION, 16, (int32_t*)&vSensordata->data.u32[0]);
		vSensordata->base.meta_data = (uint32_t)buffer[idx++];
		break;
	case DYN_PRO_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, PROTOCOL_MAGNETOMETER_PRECISION, 16, (int32_t*)&vSensordata->data.u32[0]);
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, PROTOCOL_MAGNETOMETER_PRECISION, 16, (int32_t*)&vSensordata->data.u32[3]);
		vSensordata->base.meta_data = (uint32_t)buffer[idx++];
		break;
	case DYN_PRO_SENSOR_TYPE_GYROSCOPE:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, self->precision.gyro, 16, (int32_t*)&vSensordata->data.u32[0]);
		vSensordata->base.meta_data = (uint32_t)buffer[idx++];
		break;
	case DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, self->precision.gyro, 16, (int32_t*)&vSensordata->data.u32[0]);
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, self->precision.gyro, 16, (int32_t*)&vSensordata->data.u32[3]);
		vSensordata->base.meta_data = (uint32_t)buffer[idx++];
		break;
	case DYN_PRO_SENSOR_TYPE_3AXIS:
	case DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR:
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_0:
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_1:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 4, PROTOCOL_QUATERNION_PRECISION, 30, (int32_t*)&vSensordata->data.u32[0]);
		vSensordata->base.meta_data = (uint32_t)buffer[idx++];
		break;
	case DYN_PRO_SENSOR_TYPE_ROTATION_VECTOR:
	case DYN_PRO_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 4, PROTOCOL_QUATERNION_PRECISION, 30, (int32_t*)&vSensordata->data.u32[0]);
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 1, PROTOCOL_HEADING_ACCURACY_PRECISION, 16, (int32_t*)&vSensordata->data.u32[4]);
		vSensordata->base.meta_data = (uint32_t)buffer[idx++];
		break;
	case DYN_PRO_SENSOR_TYPE_HIGH_RATE_GYRO:
		vSensordata->base.timestamp = 0;
		idx -= 4;
		vSensordata->data.u32[0] = inv_dc_le_to_int16(&buffer[idx]);
		vSensordata->data.u32[1] = inv_dc_le_to_int16(&buffer[idx+2]);
		vSensordata->data.u32[2] = inv_dc_le_to_int16(&buffer[idx+4]);
		break;
	case DYN_PRO_SENSOR_TYPE_RAW_ACCELEROMETER:
	case DYN_PRO_SENSOR_TYPE_RAW_GYROSCOPE:
	case DYN_PRO_SENSOR_TYPE_RAW_MAGNETOMETER:
	case DYN_PRO_SENSOR_TYPE_OIS:
		vSensordata->data.u32[0] = inv_dc_le_to_int16(&buffer[idx]);
		vSensordata->data.u32[1] = inv_dc_le_to_int16(&buffer[idx+2]);
		vSensordata->data.u32[2] = inv_dc_le_to_int16(&buffer[idx+4]);
		break;
	case DYN_PRO_SENSOR_TYPE_AMBIENT_TEMPERATURE:
		DynProtocol_decodeVect16QxToQy(&buffer[idx], 1, PROTOCOL_TEMPERATURE_PRECISION, 16, (int32_t*)&vSensordata->data.u32[0]);
		break;
	case DYN_PRO_SENSOR_TYPE_RAW_TEMPERATURE:
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_RAW_PPG:
                vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]); 
                vSensordata->data.u8[0] = buffer[idx+4]; 
		break;                
	case DYN_PRO_SENSOR_TYPE_ENERGY_EXPENDITURE:
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]);
		vSensordata->data.u32[1] = inv_dc_little8_to_int32(&buffer[idx+4]);
		vSensordata->data.u32[2] = inv_dc_little8_to_int32(&buffer[idx+8]);
		vSensordata->data.u32[3] = inv_dc_little8_to_int32(&buffer[idx+12]);
		break;
	case DYN_PRO_SENSOR_TYPE_DISTANCE:
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]);
		vSensordata->data.u32[1] = inv_dc_little8_to_int32(&buffer[idx+4]);
		break;
	case DYN_PRO_SENSOR_TYPE_SLEEP_ANALYSIS:
		vSensordata->data.u8[0]  = buffer[idx];
		vSensordata->data.u8[1]  = buffer[idx+1];
		vSensordata->data.u8[2]  = buffer[idx+2];
		vSensordata->data.u8[3]  = buffer[idx+3];
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx+4]);
		vSensordata->data.u32[1] = inv_dc_little8_to_int32(&buffer[idx+8]);
		vSensordata->data.u32[2] = inv_dc_little8_to_int32(&buffer[idx+12]);
		vSensordata->data.u32[3] = inv_dc_little8_to_int32(&buffer[idx+16]);
		vSensordata->data.u32[4] = inv_dc_little8_to_int32(&buffer[idx+20]);
		vSensordata->data.u8[4]  = buffer[idx+24];
		break;
	case DYN_PRO_SENSOR_TYPE_BAC_EXTENDED:
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_BAC_STATISTICS:
		vSensordata->data.u32[0]  = inv_dc_little8_to_int32(&buffer[idx]);
		vSensordata->data.u32[1]  = inv_dc_little8_to_int32(&buffer[idx+4]);
		vSensordata->data.u32[2]  = inv_dc_little8_to_int32(&buffer[idx+8]);
		vSensordata->data.u32[3]  = inv_dc_little8_to_int32(&buffer[idx+12]);
		vSensordata->data.u32[4]  = inv_dc_little8_to_int32(&buffer[idx+16]);
		vSensordata->data.u32[5]  = inv_dc_little8_to_int32(&buffer[idx+20]);
		vSensordata->data.u32[6]  = inv_dc_little8_to_int32(&buffer[idx+24]);
		vSensordata->data.u32[7]  = inv_dc_little8_to_int32(&buffer[idx+28]);
		vSensordata->data.u32[8]  = inv_dc_little8_to_int32(&buffer[idx+32]);
		vSensordata->data.u32[9]  = inv_dc_little8_to_int32(&buffer[idx+36]);
		vSensordata->data.u32[10] = inv_dc_little8_to_int32(&buffer[idx+40]);
		break;
	case DYN_PRO_SENSOR_TYPE_FLOOR_CLIMB_COUNTER:
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]);
		vSensordata->data.u32[1] = inv_dc_little8_to_int32(&buffer[idx+4]);
		break;
	case DYN_PRO_SENSOR_TYPE_STEP_COUNTER:
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]);
		vSensordata->base.timestamp = inv_dc_little8_to_int32(&buffer[idx+4]);
		break;
	case DYN_PRO_SENSOR_TYPE_BAC:
		vSensordata->data.u8[0] = buffer[idx++];
		vSensordata->base.timestamp = inv_dc_little8_to_int32(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_WOM:
		vSensordata->data.u8[0] = buffer[idx++];
		vSensordata->base.timestamp = inv_dc_little8_to_int32(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_B2S:
	case DYN_PRO_SENSOR_TYPE_SHAKE:
	case DYN_PRO_SENSOR_TYPE_DOUBLE_TAP:
	case DYN_PRO_SENSOR_TYPE_SEDENTARY_REMIND:
	case DYN_PRO_SENSOR_TYPE_SMD:
	case DYN_PRO_SENSOR_TYPE_STEP_DETECTOR:
	case DYN_PRO_SENSOR_TYPE_TILT_DETECTOR:
	case DYN_PRO_SENSOR_TYPE_WAKE_GESTURE:
	case DYN_PRO_SENSOR_TYPE_GLANCE_GESTURE:
	case DYN_PRO_SENSOR_TYPE_PICK_UP_GESTURE:
		vSensordata->base.timestamp = inv_dc_little8_to_int32(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_PRESSURE:
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_ORIENTATION:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, PROTOCOL_ORIENTATION_PRECISION, 16, (int32_t*)&vSensordata->data.u32[0]);
		vSensordata->base.meta_data = (uint32_t)buffer[idx++];
		break;
	case DYN_PRO_SENSOR_TYPE_LIGHT:
		vSensordata->data.u32[0] = inv_dc_little8_to_int32(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_PROXIMITY:
		vSensordata->data.u32[0] = (uint32_t)inv_dc_le_to_int16(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_FSYNC_EVENT:
		vSensordata->data.u32[0] = (uint32_t)inv_dc_le_to_int16(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_EIS:
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, self->precision.gyro, 16, (int32_t*)&vSensordata->data.u32[0]);
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 3, self->precision.gyro, 16, (int32_t*)&vSensordata->data.u32[3]);
		vSensordata->data.u32[6] = (uint32_t)inv_dc_le_to_int16(&buffer[idx]);
		break;
	case DYN_PRO_SENSOR_TYPE_CUSTOM0:
	case DYN_PRO_SENSOR_TYPE_CUSTOM1:
	case DYN_PRO_SENSOR_TYPE_CUSTOM2:
	case DYN_PRO_SENSOR_TYPE_CUSTOM3:
	case DYN_PRO_SENSOR_TYPE_CUSTOM4:
	case DYN_PRO_SENSOR_TYPE_CUSTOM5:
	case DYN_PRO_SENSOR_TYPE_CUSTOM6:
	case DYN_PRO_SENSOR_TYPE_CUSTOM7:
	{
		/* meta data contains size of custom sensors.
		 * Check size is not bigger than VSensorData array */
		vSensordata->base.meta_data = buffer[idx++];
		if(vSensordata->base.meta_data > sizeof(vSensordata->data.u8))
			return -1;

		memcpy(vSensordata->data.u8, &buffer[idx], vSensordata->base.meta_data);
		//hard code the payload size until payload has a fixed value for custom sensors
		idx += sizeof(vSensordata->data.u8);
		break;
	}

	case DYN_PRO_SENSOR_TYPE_CUSTOM_PRESSURE:
		for (i=0; i<4; i++) {
			vSensordata->data.u32[i] = inv_dc_little8_to_int32(&buffer[idx]); // raw pressure
			idx += 4;
		}
		break;

	case DYN_PRO_SENSOR_TYPE_HEART_RATE:
	{
		// ppm
		idx += DynProtocol_decodeVect16QxToQy(&buffer[idx], 1, PROTOCOL_HRM_PRECISION, 16, (int32_t*)&vSensordata->data.u32[0]);
				  
		// confidence
		vSensordata->data.u8[0] = buffer[idx++];

		// sqi
		vSensordata->data.u8[1] = buffer[idx++];

		break;
	}
        
	case DYN_PRO_SENSOR_TYPE_HRV:
	{
		// RR_count
		vSensordata->data.u8[0] = buffer[idx];
		idx += 1;

		// paddingDummy
		vSensordata->data.u8[1] = buffer[idx];
		idx += 1;
		
		// RR_interval
		for (i=0; i<4; i++) {
			vSensordata->data.u16[i] = inv_dc_little8_to_int16(&buffer[idx]);
			idx += 2;
		}

		break;
	}                
                
                
	default:
		return -1;
	}

	// check if we did not read more bytes than actually transfered by protocol
	if(idx > size)
		return -1;

	return 0;

}

static int DynProtocol_decodePktCommand(DynProtocol_t * self,
		struct DynProtocolEdata * edata)
{
	int i, precision;
	const uint8_t * buf = self->decode_state_machine.tmp_buffer;

	if(self->decode_state_machine.cmd_id == DYN_PROTOCOL_EID_PROTOCOLVERSION) {
		edata->sensor_id = 0;
	}
	else {
		edata->sensor_id = buf[0];

		switch(self->decode_state_machine.cmd_id) {
		case DYN_PROTOCOL_EID_SET_SENSOR_PERIOD:
			edata->d.command.period = (uint32_t)inv_dc_little8_to_int32(&buf[1]);
			break;

		case DYN_PROTOCOL_EID_SET_SENSOR_TIMEOUT:
			edata->d.command.timeout = (uint32_t)inv_dc_little8_to_int32(&buf[1]);
			break;

		case DYN_PROTOCOL_EID_SET_SENSOR_MMATRIX:
		{
			edata->d.command.cfg.base.type = buf[1];
			for (i = 0; i < 9; i++)
				((VSensorConfigReferenceFrame *)&edata->d.command.cfg)->matrix[i] = (intq30_t) inv_dc_little8_to_int32(&buf[2 + i * 4]);
			break;
		}

		case DYN_PROTOCOL_EID_GET_SW_REG:
			edata->d.command.regAddr = buf[1];
			break;

		case DYN_PROTOCOL_EID_SET_SENSOR_CFG:
		{
			edata->d.command.cfg.base.type = buf[1];
			edata->d.command.cfg.base.size = (uint32_t)buf[2];

			switch (edata->d.command.cfg.base.type) {
			case VSENSOR_CONFIG_TYPE_OFFSET:
			{
				precision = DynProtocol_getPrecision(self, edata->sensor_id);
				DynProtocol_decodeVect16QxToQy(&buf[3], 3, precision, 16, (int32_t*)&edata->d.command.cfg.buffer[0]);
				break;
			}

			default:
				if(buf[2] > sizeof(edata->d.command.cfg.buffer))
					return -1;

				memcpy(edata->d.command.cfg.buffer, &buf[3], buf[2]);
				break;
			}
			break;
		}

		case DYN_PROTOCOL_EID_GET_SENSOR_CFG:
			edata->d.command.cfg.base.type = buf[1];
			break;

		case DYN_PROTOCOL_EID_GET_SENSOR_BIAS:
			edata->d.command.cfg.base.type = buf[1];
			break;

		case DYN_PROTOCOL_EID_SET_SENSOR_BIAS:
			edata->d.command.cfg.base.type = buf[1];
			precision = DynProtocol_getPrecision(self, edata->sensor_id);
			DynProtocol_decodeVect16QxToQy(&buf[2], 3, precision, 16, &((VSensorConfigOffset *)&edata->d.command.cfg)->vect[0]);
			break;

		default:
			break;
		}
	}

	return 0;
}

static int DynProtocol_decodePktResponse(DynProtocol_t * self,
		struct DynProtocolEdata * edata)
{
	int precision;
	int rc = 0;
	const uint8_t * buf = self->decode_state_machine.tmp_buffer;
	const uint16_t len = self->decode_state_machine.received_size;

	switch(self->decode_state_machine.cmd_id) {
	case DYN_PROTOCOL_EID_PROTOCOLVERSION:
	case DYN_PROTOCOL_EID_GET_FW_INFO:
		memcpy(edata->d.response.version, buf, sizeof(edata->d.response.version) - 1);
		edata->d.response.version[sizeof(edata->d.response.version)-1] = '\0';
		break;

	case DYN_PROTOCOL_EID_GET_SENSOR_DATA:
	{
		edata->d.response.sensorData.rc = (int)(int8_t)buf[0];
		if(edata->d.response.sensorData.rc == 0)
			rc = DynProtocol_decodeSensorEvent(self, &buf[1], len, edata, DYN_PROTOCOL_ETYPE_RESP);
		break;
	}

	case DYN_PROTOCOL_EID_GET_SENSOR_CFG:
	{
		edata->d.response.sensorcfg.rc = (int)(int8_t)buf[0];
		edata->d.response.sensorcfg.cfg.base.type = buf[1];
		edata->d.response.sensorcfg.cfg.base.size = (uint32_t)buf[2];

		switch (edata->d.response.sensorcfg.cfg.base.type) {
		case VSENSOR_CONFIG_TYPE_OFFSET:
		{
			edata->sensor_id = (int)(int8_t)buf[3];
			precision = DynProtocol_getPrecision(self, edata->sensor_id);
			DynProtocol_decodeVect16QxToQy(&buf[4], 3, precision, 16, (int32_t*)&edata->d.response.sensorcfg.cfg.buffer[0]);
			break;
		}

		default:
			if(buf[2] > sizeof(edata->d.response.sensorcfg.cfg.buffer))
				return -1;

			memcpy(edata->d.response.sensorcfg.cfg.buffer, &buf[3], buf[2]);
			break;
		}

		break;
	}

	case DYN_PROTOCOL_EID_GET_SENSOR_BIAS:
		edata->d.response.sensorcfg.rc = (int)(int8_t)buf[0];
		edata->sensor_id = (int)(int8_t)buf[1];
		edata->d.response.sensorcfg.cfg.base.type = buf[2];
		precision = DynProtocol_getPrecision(self, edata->sensor_id);
		DynProtocol_decodeVect16QxToQy(&buf[3], 3, precision, 16, &((VSensorConfigOffset *)&edata->d.response.sensorcfg.cfg)->vect[0]);
		break;

	default:
		edata->d.response.rc = (int)(int8_t)buf[0];
	}

	return rc;
}

static int DynProtocol_decodePktAsync(DynProtocol_t * self,
		struct DynProtocolEdata * edata)
{
	const uint8_t * buf = self->decode_state_machine.tmp_buffer;
	const uint16_t len = self->decode_state_machine.received_size;

	switch(self->decode_state_machine.cmd_id) {
	case DYN_PROTOCOL_EID_NEW_SENSOR_DATA:
		return DynProtocol_decodeSensorEvent(self, buf, len, edata, DYN_PROTOCOL_ETYPE_ASYNC);

	default:
		return -1;
	}
}

static inline void DynProtocol_callEventCB(DynProtocol_t * self,
	enum DynProtocolEtype etype,
	enum DynProtocolEid eid,
	const DynProtocolEdata_t * edata
)
{
	if(self->event_cb) {
		self->event_cb(etype, eid, edata, self->event_cb_cookie);
	}
}

static int DynProtocol_doProcess(DynProtocol_t * self)
{
	struct DynProtocolEdata edata;
	enum DynProtocolEtype etype;
	int rc;

	self->decode_state_machine.state = PROTOCOL_STATE_IDLE;

	switch(self->decode_state_machine.event_type) {
	case EVENT_TYPE_CMD:
		rc = DynProtocol_decodePktCommand(self, &edata);
		etype = DYN_PROTOCOL_ETYPE_CMD;
		break;

	case EVENT_TYPE_RESP:
		rc = DynProtocol_decodePktResponse(self, &edata);
		etype = DYN_PROTOCOL_ETYPE_RESP;
		break;

	case EVENT_TYPE_ASYNC:
		rc = DynProtocol_decodePktAsync(self, &edata);
		etype = DYN_PROTOCOL_ETYPE_ASYNC;
		break;

	default:
		INV_MSG(INV_MSG_LEVEL_WARNING, "DynProtocol: Unexpected packet type");
		return -1;
	}

	if(rc == 0) {
		DynProtocol_callEventCB(self, etype, (enum DynProtocolEid)self->decode_state_machine.cmd_id, &edata);
		return 1;
	} else {
		INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: Unexpected packet received.");
	}

	return rc;
}

static int DynProtocol_encodeSensorEvent(DynProtocol_t * self, const DynProtocolEdata_t *edata,
		uint8_t * outBuffer, uint16_t maxBufferSize, enum DynProtocolEtype etype)
{
	uint16_t idx = 0, i=0;
	uint32_t sensorStatus;
	uint32_t timestamp;
	const VSensorDataAny *vSensordata;

	switch(etype) {
	case DYN_PROTOCOL_ETYPE_RESP:
		sensorStatus = edata->d.response.sensorData.status;
		vSensordata = &edata->d.response.sensorData.vdata;
		break;

	case DYN_PROTOCOL_ETYPE_ASYNC:
		sensorStatus = edata->d.async.sensorEvent.status;
		vSensordata = &edata->d.async.sensorEvent.vdata;
		break;

	case DYN_PROTOCOL_ETYPE_CMD:
	default:
		return INV_ERROR_BAD_ARG;
	}

	timestamp = (uint32_t)vSensordata->base.timestamp;

	if(maxBufferSize < 4) {
		goto error_size;
	}

	outBuffer[idx] = (uint8_t)(sensorStatus & 0x03);
	idx += 1;
	outBuffer[idx] = (uint8_t)edata->sensor_id;
	idx += 1;

	inv_dc_int32_to_little8(timestamp, &outBuffer[idx]);
	idx += 4;

	maxBufferSize -= idx;

	switch(edata->sensor_id) {
	case DYN_PRO_SENSOR_TYPE_RESERVED:
		/* no data */
		break;

	case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:
	case DYN_PRO_SENSOR_TYPE_GRAVITY:
	case DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION:
		if(maxBufferSize < 6+1)
			goto error_size;
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 3, 16, self->precision.acc, &outBuffer[idx]);
		/* report accuracy */
		outBuffer[idx++] = (uint8_t)(vSensordata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
		break;

	case DYN_PRO_SENSOR_TYPE_MAGNETOMETER:
		if(maxBufferSize < 6+1)
			goto error_size;
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 3, 16, PROTOCOL_MAGNETOMETER_PRECISION, &outBuffer[idx]);
		outBuffer[idx++] = (uint8_t)(vSensordata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
		break;

	case DYN_PRO_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		if(maxBufferSize < 13)
			goto error_size;
		// uncalibrated data
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 3, 16, PROTOCOL_MAGNETOMETER_PRECISION, &outBuffer[idx]);
		// bias
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[3], 3, 16, PROTOCOL_MAGNETOMETER_PRECISION, &outBuffer[idx]);
		// accuracy
		outBuffer[idx++] = (uint8_t)(vSensordata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
		break;

	case DYN_PRO_SENSOR_TYPE_GYROSCOPE:
		if(maxBufferSize < 6+1)
			goto error_size;
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 3, 16, self->precision.gyro, &outBuffer[idx]);
		outBuffer[idx++] = (uint8_t)(vSensordata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
		break;

	case DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE:
		if(maxBufferSize < 13)
			goto error_size;
		// uncalibrated data
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 3, 16, self->precision.gyro, &outBuffer[idx]);
		// bias
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[3], 3, 16, self->precision.gyro, &outBuffer[idx]);
		// accuracy
		outBuffer[idx++] = (uint8_t)(vSensordata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
		break;

	case DYN_PRO_SENSOR_TYPE_3AXIS:
	case DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR:
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_0:
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_1:
		if(maxBufferSize < 9)
			goto error_size;
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 4, 30, PROTOCOL_QUATERNION_PRECISION, &outBuffer[idx]);
		outBuffer[idx++] = (uint8_t)(vSensordata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
		break;

	case DYN_PRO_SENSOR_TYPE_ROTATION_VECTOR:
	case DYN_PRO_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		if(maxBufferSize < 11)
			goto error_size;
		// w,x,y,z in Q30
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 4, 30, PROTOCOL_QUATERNION_PRECISION, &outBuffer[idx]);
		// accuracy in q16
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[4], 1, 16, PROTOCOL_HEADING_ACCURACY_PRECISION, &outBuffer[idx]);
		outBuffer[idx++] = (uint8_t)(vSensordata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
		break;

	case DYN_PRO_SENSOR_TYPE_HIGH_RATE_GYRO:
		// no timestamp
		idx -= 4;
		maxBufferSize += idx;
		if(maxBufferSize < 6)
			goto error_size;
		inv_dc_int16_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		inv_dc_int16_to_little8(vSensordata->data.u32[1], &outBuffer[idx+2]);
		inv_dc_int16_to_little8(vSensordata->data.u32[2], &outBuffer[idx+4]);
		idx += 6;
		break;

	case DYN_PRO_SENSOR_TYPE_RAW_ACCELEROMETER:
	case DYN_PRO_SENSOR_TYPE_RAW_GYROSCOPE:
	case DYN_PRO_SENSOR_TYPE_RAW_MAGNETOMETER:
	case DYN_PRO_SENSOR_TYPE_OIS:
		if(maxBufferSize < 6)
			goto error_size;
		inv_dc_int16_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		inv_dc_int16_to_little8(vSensordata->data.u32[1], &outBuffer[idx+2]);
		inv_dc_int16_to_little8(vSensordata->data.u32[2], &outBuffer[idx+4]);
		idx += 6;
		break;

	case DYN_PRO_SENSOR_TYPE_AMBIENT_TEMPERATURE:
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 1, 16, PROTOCOL_TEMPERATURE_PRECISION, &outBuffer[idx]);
		break;

	case DYN_PRO_SENSOR_TYPE_RAW_TEMPERATURE:
		if(maxBufferSize < 4)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		break;

	case DYN_PRO_SENSOR_TYPE_RAW_PPG:
		if(maxBufferSize < 5)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]); 
                idx += 4;
                outBuffer[idx] = vSensordata->data.u8[0]; 
                idx += 1;
                break;
                
	case DYN_PRO_SENSOR_TYPE_ENERGY_EXPENDITURE:
		if(maxBufferSize < 16)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[1], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[2], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[3], &outBuffer[idx]);
		idx += 4;
		break;

	case DYN_PRO_SENSOR_TYPE_DISTANCE:
		if(maxBufferSize < 8)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[1], &outBuffer[idx]);
		idx += 4;
		break;

	case DYN_PRO_SENSOR_TYPE_SLEEP_ANALYSIS:
		if(maxBufferSize < 25)
			goto error_size;
		outBuffer[idx] = vSensordata->data.u8[0]; 
		idx += 1;
		outBuffer[idx] = vSensordata->data.u8[1]; 
		idx += 1;
		outBuffer[idx] = vSensordata->data.u8[2]; 
		idx += 1;
		outBuffer[idx] = vSensordata->data.u8[3]; 
		idx += 1;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[1], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[2], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[3], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[4], &outBuffer[idx]);
		idx += 4;
		outBuffer[idx] = vSensordata->data.u8[4]; 
		idx += 1;
		break;

	case DYN_PRO_SENSOR_TYPE_BAC_EXTENDED:
		if(maxBufferSize < 4)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		break;

	case DYN_PRO_SENSOR_TYPE_BAC_STATISTICS:
		if(maxBufferSize < 44)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[1], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[2], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[3], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[4], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[5], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[6], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[7], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[8], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[9], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[10], &outBuffer[idx]);
		idx += 4;
		break;

	case DYN_PRO_SENSOR_TYPE_FLOOR_CLIMB_COUNTER:
		if(maxBufferSize < 8)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(vSensordata->data.u32[1], &outBuffer[idx]);
		idx += 4;
		break;
		
	case DYN_PRO_SENSOR_TYPE_STEP_COUNTER:
		if(maxBufferSize < 8)
			goto error_size;
		inv_dc_int32_to_little8((int32_t)vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		inv_dc_int32_to_little8(timestamp, &outBuffer[idx]);
		idx += 4;
		break;

	case DYN_PRO_SENSOR_TYPE_BAC:
		if(maxBufferSize < 5)
			goto error_size;
		outBuffer[idx++] = vSensordata->data.u8[0];
		inv_dc_int32_to_little8(timestamp, &outBuffer[idx]);
		idx += 4;
		break;
	case DYN_PRO_SENSOR_TYPE_WOM:
		if(maxBufferSize < 5)
			goto error_size;
		outBuffer[idx++] = vSensordata->data.u8[0];
		inv_dc_int32_to_little8(timestamp, &outBuffer[idx]);
		idx += 4;
		break;
	case DYN_PRO_SENSOR_TYPE_B2S:
	case DYN_PRO_SENSOR_TYPE_SHAKE:
	case DYN_PRO_SENSOR_TYPE_DOUBLE_TAP:
	case DYN_PRO_SENSOR_TYPE_SEDENTARY_REMIND:
	case DYN_PRO_SENSOR_TYPE_SMD:
	case DYN_PRO_SENSOR_TYPE_STEP_DETECTOR:
	case DYN_PRO_SENSOR_TYPE_TILT_DETECTOR:
	case DYN_PRO_SENSOR_TYPE_WAKE_GESTURE:
	case DYN_PRO_SENSOR_TYPE_GLANCE_GESTURE:
	case DYN_PRO_SENSOR_TYPE_PICK_UP_GESTURE:
	{
		if(maxBufferSize < 4)
			goto error_size;
		inv_dc_int32_to_little8(timestamp, &outBuffer[idx]);
		idx += 4;
		break;
	}

	case DYN_PRO_SENSOR_TYPE_PRESSURE:
	{
		if(maxBufferSize < 4)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		break;
	}

	case DYN_PRO_SENSOR_TYPE_ORIENTATION:
	{
		if(maxBufferSize < 6+1)
			goto error_size;
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 3, 16, PROTOCOL_ORIENTATION_PRECISION, &outBuffer[idx]);
		outBuffer[idx++] = (uint8_t)(vSensordata->base.meta_data & VSENSOR_DATA_ACCURACY_MASK);
		break;
	}

	case DYN_PRO_SENSOR_TYPE_LIGHT:
	{
		if(maxBufferSize < 4)
			goto error_size;
		inv_dc_int32_to_little8(vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 4;
		break;
	}

	case DYN_PRO_SENSOR_TYPE_PROXIMITY:
	{
		if(maxBufferSize < 2)
			goto error_size;
		inv_dc_int16_to_little8((int16_t)vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 2;
		break;
	}

	case DYN_PRO_SENSOR_TYPE_FSYNC_EVENT:
	{
		if(maxBufferSize < 2)
			goto error_size;
		// delta_ts
		inv_dc_int16_to_little8((int16_t)vSensordata->data.u32[0], &outBuffer[idx]);
		idx += 2;
		break;
	}

	case DYN_PRO_SENSOR_TYPE_EIS:
	{
		if(maxBufferSize < 14)
			goto error_size;
		// uncalibrated data
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 3, 16, self->precision.gyro, &outBuffer[idx]);
		// bias
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[3], 3, 16, self->precision.gyro, &outBuffer[idx]);
		// delta_ts
		inv_dc_int16_to_little8((int16_t)vSensordata->data.u32[6], &outBuffer[idx]);
		idx += 2;
		break;
	}

	case DYN_PRO_SENSOR_TYPE_CUSTOM0:
	case DYN_PRO_SENSOR_TYPE_CUSTOM1:
	case DYN_PRO_SENSOR_TYPE_CUSTOM2:
	case DYN_PRO_SENSOR_TYPE_CUSTOM3:
	case DYN_PRO_SENSOR_TYPE_CUSTOM4:
	case DYN_PRO_SENSOR_TYPE_CUSTOM5:
	case DYN_PRO_SENSOR_TYPE_CUSTOM6:
	case DYN_PRO_SENSOR_TYPE_CUSTOM7:
	{
		if(maxBufferSize < 64)
			goto error_size;
		// meta data contains size of custom sensors, it can be checked in host
		outBuffer[idx++] = vSensordata->base.meta_data;
		// one byte less available in outBuffer
		maxBufferSize--;
		// let's make sure that size to be copied is not bigger than size left in outBuffer
		if(vSensordata->base.meta_data > maxBufferSize)
			goto error_size;

		memcpy(&outBuffer[idx], vSensordata->data.u8, vSensordata->base.meta_data);
		//set hardly the payload size until payload has a fixed value for custom sensors
		idx += sizeof(vSensordata->data.u8);
		break;
	}

	case DYN_PRO_SENSOR_TYPE_CUSTOM_PRESSURE:
	{
		if(maxBufferSize < 16)
			goto error_size;

		for (i=0; i<4; i++) {
			inv_dc_int32_to_little8(vSensordata->data.u32[i], &outBuffer[idx]); // raw pressure
			idx += 4;
		}
		break;
	}

	case DYN_PRO_SENSOR_TYPE_HEART_RATE:
	{
		if(maxBufferSize < 4)
			goto error_size;
                        
		// ppm
		idx += DynProtocol_encodeQxToQyVect16((int32_t*)&vSensordata->data.u32[0], 1, 16, PROTOCOL_HRM_PRECISION, &outBuffer[idx]);

		// confidence
		outBuffer[idx] = vSensordata->data.u8[0]; 
		idx++;

		// sqi
		outBuffer[idx] = vSensordata->data.u8[1]; 
		idx++;                

		break;
	}
        
	case DYN_PRO_SENSOR_TYPE_HRV:
	{
		if(maxBufferSize < 10)
			goto error_size;
                
		// RR_count
		outBuffer[idx] = vSensordata->data.u8[1]; 
		idx += 1;

		// paddingDummy
		outBuffer[idx] = vSensordata->data.u8[0]; 
		idx += 1;
                
		// RR_interval
		for (i=0; i<4; i++) {
			inv_dc_int16_to_little8(vSensordata->data.u16[i], &outBuffer[idx]);
			idx += 2;
		}                

		break;
	}

	default:
		return -1;
	}

	return idx;

error_size:
	return maxBufferSize + idx + 1; /* +1 to indicate buffer is too small */
}


static int DynProtocol_checkFrameSize(DynProtocol_t * self)
{
	if(self->decode_state_machine.expected_size == UINT16_MAX) {
		/* assume frame is unknown, return error */
		INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: Unknown frame");
		return INV_ERROR_SIZE;
	}

	/*
	 * expected_size is the payload expected size (not the total frame expected size).
	 * it should respect this condititon:
	 *  current_frame_size >= 1 byte (GID) + 1 byte (CID) + expected_size
	 * note : current_frame_size == UINT16_MAX means check on frame size disabled (default).
	 */
	if(self->decode_state_machine.current_frame_size != UINT16_MAX) {
		if(self->decode_state_machine.current_frame_size < (self->decode_state_machine.expected_size + 2)) {
			INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: Frame size error. size=%dB (expected %dB)",
				self->decode_state_machine.current_frame_size, self->decode_state_machine.expected_size + 2);
			return INV_ERROR_SIZE;
		}
	}

	return INV_ERROR_SUCCESS;
}

void DynProtocol_init(DynProtocol_t * self,
		DynProtocolEvent_cb event_cb, void * event_cb_cookie)
{
	memset(self, 0, sizeof(*self));

	self->event_cb        = event_cb;
	self->event_cb_cookie = event_cb_cookie;
	self->decode_state_machine.current_frame_size = UINT16_MAX;
	self->precision.acc = PROTOCOL_ACCELEROMETER_PRECISION;
	self->precision.gyro = PROTOCOL_GYROSCOPE_PRECISION;
}

void DynProtocol_processReset(DynProtocol_t * self)
{
	self->decode_state_machine.state = PROTOCOL_STATE_IDLE;
}

void DynProtocol_setCurrentFrameSize(DynProtocol_t * self, uint16_t frameSizeB)
{
	self->decode_state_machine.current_frame_size = frameSizeB;
}

int DynProtocol_setPrecision(DynProtocol_t * self, int sensor, uint8_t precision)
{
	int rc = 0;
	switch(sensor)
	{
		case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:
			self->precision.acc = precision;
			break;
		case DYN_PRO_SENSOR_TYPE_GYROSCOPE:
			self->precision.gyro = precision;
			break;
		default :
			rc = -1;
			break;
	}
	return rc;
}

int DynProtocol_processPktByte(DynProtocol_t * self, uint8_t rcvByte)
{
	switch(self->decode_state_machine.state) {
	case PROTOCOL_STATE_GID:
	{
		self->decode_state_machine.event_type = (rcvByte & EVENT_TYPE_MASK);
		self->decode_state_machine.group_id   = (rcvByte & ~EVENT_TYPE_MASK);
		self->decode_state_machine.state      = PROTOCOL_STATE_CID;

		if(self->decode_state_machine.group_id != DYN_PROTOCOL_GROUP_ID) {
			self->decode_state_machine.state  = PROTOCOL_STATE_IDLE;
			INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: Invalid group ID");
			return -1;
		}
		break;
	}

	case PROTOCOL_STATE_CID:
	{
		int rc;

		self->decode_state_machine.cmd_id     = rcvByte;
		self->decode_state_machine.state      = PROTOCOL_STATE_PAYLOAD;
		self->decode_state_machine.received_size = 0;
		self->decode_state_machine.expected_size = 0;
		self->decode_state_machine.expected_size = DynProtocol_getPayload(self);

		if((rc = DynProtocol_checkFrameSize(self)) != INV_ERROR_SUCCESS) {
			self->decode_state_machine.state = PROTOCOL_STATE_IDLE;
			return rc;
		}

		if(self->decode_state_machine.expected_size == 0) {
			return DynProtocol_doProcess(self);
		}
		break;
	}

	case PROTOCOL_STATE_PAYLOAD:
	{
		int rc;

		if(self->decode_state_machine.received_size >= MAX_EXPECTED_PAYLOAD)
			INV_MSG(INV_MSG_LEVEL_WARNING, "DynProtocol: internal buffer size full");
		else
			self->decode_state_machine.tmp_buffer[self->decode_state_machine.received_size] = rcvByte;

		self->decode_state_machine.received_size++;
		if(self->decode_state_machine.received_size == self->decode_state_machine.expected_size)
			/* update expected payload, in case actual payload cannot be determined using only CID */
			self->decode_state_machine.expected_size = DynProtocol_getPayload(self);

		if((rc = DynProtocol_checkFrameSize(self)) != INV_ERROR_SUCCESS) {
			self->decode_state_machine.state = PROTOCOL_STATE_IDLE;
			return rc;
		}

		if(self->decode_state_machine.received_size == self->decode_state_machine.expected_size)
			return DynProtocol_doProcess(self);

		break;
	}
	}

	return 0;
}

int DynProtocol_encodeCommand(DynProtocol_t * self,
		enum DynProtocolEid eid, const DynProtocolEdata_t * edata,
		uint8_t * outBuffer, uint16_t maxBufferSize, uint16_t *outBufferSize)
{
	int i, precision;
	uint16_t idx = 0;

	(void)self;

	*outBufferSize = 0;

	if(maxBufferSize < 2)
		goto error_size;

	outBuffer[idx]  = EVENT_TYPE_CMD; // Set event type
	outBuffer[idx++] |= (DYN_PROTOCOL_GROUP_ID & ~EVENT_TYPE_MASK); // Set group ID
	outBuffer[idx++] = (uint8_t)eid;

	switch(eid) {
	case DYN_PROTOCOL_EID_PROTOCOLVERSION:
	case DYN_PROTOCOL_EID_GET_FW_INFO:
	case DYN_PROTOCOL_EID_WHO_AM_I:
	case DYN_PROTOCOL_EID_RESET:
	case DYN_PROTOCOL_EID_SETUP:
	case DYN_PROTOCOL_EID_CLEANUP:
		break;

	case DYN_PROTOCOL_EID_SELF_TEST:
	case DYN_PROTOCOL_EID_PING_SENSOR:
	case DYN_PROTOCOL_EID_START_SENSOR:
	case DYN_PROTOCOL_EID_STOP_SENSOR:
	case DYN_PROTOCOL_EID_FLUSH_SENSOR:
	case DYN_PROTOCOL_EID_GET_SENSOR_DATA:
		if((maxBufferSize - idx) < 1)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		break;

	case DYN_PROTOCOL_EID_SET_SENSOR_PERIOD:
		if((maxBufferSize - idx) < 5)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		inv_dc_int32_to_little8(edata->d.command.period, &outBuffer[idx]);
		idx += 4;
		break;

	case DYN_PROTOCOL_EID_SET_SENSOR_TIMEOUT:
		if((maxBufferSize - idx) < 5)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		inv_dc_int32_to_little8(edata->d.command.timeout, &outBuffer[idx]);
		idx += 4;
		break;

	case DYN_PROTOCOL_EID_GET_SW_REG:
		if((maxBufferSize - idx) < 2)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		outBuffer[idx++] = (uint8_t)edata->d.command.regAddr;
		break;

	case DYN_PROTOCOL_EID_SET_SENSOR_MMATRIX:
		if ((maxBufferSize - idx) < 1 + 1 + (9 * 4))
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		outBuffer[idx++] = (uint8_t)((VSensorConfigReferenceFrame *)&edata->d.command.cfg)->base.type;
		for (i = 0; i < 9; i++) {
			inv_dc_int32_to_little8(((VSensorConfigReferenceFrame *)&edata->d.command.cfg)->matrix[i], &outBuffer[idx]);
			idx += 4;
		}
		break;

	case DYN_PROTOCOL_EID_SET_SENSOR_CFG:
		if((maxBufferSize - idx) < 67)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		outBuffer[idx++] = (uint8_t)edata->d.command.cfg.base.type;
		outBuffer[idx++] = (uint8_t)edata->d.command.cfg.base.size;
		switch (edata->d.command.cfg.base.type) {
		case VSENSOR_CONFIG_TYPE_OFFSET:
			precision = DynProtocol_getPrecision(self, edata->sensor_id);
			DynProtocol_encodeQxToQyVect16((int32_t*)&edata->d.command.cfg.buffer[0], 3, 16, precision, &outBuffer[idx]);
			break;
		default:
			// we don't know what is the value of cfg.size so let's make sure
			// there is enough room in outBuffer.
			if((uint32_t)(maxBufferSize - idx) < edata->d.command.cfg.base.size)
				goto error_size;
			memcpy(&outBuffer[idx], edata->d.command.cfg.buffer, edata->d.command.cfg.base.size);
			break;
		}
		idx = 67 + 2;
		break;

	case DYN_PROTOCOL_EID_GET_SENSOR_CFG:
		if((maxBufferSize - idx) < 2)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		outBuffer[idx++] = (uint8_t)edata->d.command.cfg.base.type;
		break;

	case DYN_PROTOCOL_EID_GET_SENSOR_BIAS:
		if((maxBufferSize - idx) < 2)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		outBuffer[idx++] = (uint8_t)edata->d.command.cfg.base.type;
		break;

	case DYN_PROTOCOL_EID_SET_SENSOR_BIAS:
		if ((maxBufferSize - idx) < 1 + (3 * 2))
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		outBuffer[idx++] = (uint8_t)edata->d.command.cfg.base.type;
		precision = DynProtocol_getPrecision(self, edata->sensor_id);
		idx += DynProtocol_encodeQxToQyVect16(&((VSensorConfigOffset *)&edata->d.command.cfg)->vect[0], 3, 16, precision, &outBuffer[idx]);
		break;

	default:
		INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: Unexpected argument for encode_command()");
		return -1;
	}

	*outBufferSize = (idx);
	return 0;

error_size:
	INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: output buffer size too small");
	return -1;
}

int DynProtocol_encodeResponse(DynProtocol_t * self,
		enum DynProtocolEid eid, const DynProtocolEdata_t * edata,
		uint8_t * outBuffer, uint16_t maxBufferSize, uint16_t *outBufferSize)
{
	int precision;
	uint16_t idx = 0;

	(void)self;

	*outBufferSize = 0;

	if(maxBufferSize < 2)
		goto error_size;

	outBuffer[idx]  = EVENT_TYPE_RESP; // Set event type
	outBuffer[idx++] |= DYN_PROTOCOL_GROUP_ID & ~EVENT_TYPE_MASK; // Set group ID
	outBuffer[idx++] = (uint8_t)eid;

	switch(eid) {
	case DYN_PROTOCOL_EID_PROTOCOLVERSION:
	case DYN_PROTOCOL_EID_GET_FW_INFO:
	{
		if((maxBufferSize - idx) < 16)
			goto error_size;
		memcpy(&outBuffer[idx], edata->d.response.version, 15);
		outBuffer[idx+15] = '\0';
		idx += 16;
		break;
	}

	case DYN_PROTOCOL_EID_WHO_AM_I:
	case DYN_PROTOCOL_EID_RESET:
	case DYN_PROTOCOL_EID_SETUP:
	case DYN_PROTOCOL_EID_CLEANUP:
	case DYN_PROTOCOL_EID_SELF_TEST:
	case DYN_PROTOCOL_EID_PING_SENSOR:
	case DYN_PROTOCOL_EID_START_SENSOR:
	case DYN_PROTOCOL_EID_STOP_SENSOR:
	case DYN_PROTOCOL_EID_FLUSH_SENSOR:
	case DYN_PROTOCOL_EID_SET_SENSOR_PERIOD:
	case DYN_PROTOCOL_EID_SET_SENSOR_TIMEOUT:
	case DYN_PROTOCOL_EID_SET_SENSOR_MMATRIX:
	case DYN_PROTOCOL_EID_GET_SW_REG:
	case DYN_PROTOCOL_EID_SET_SENSOR_CFG:
	case DYN_PROTOCOL_EID_SET_SENSOR_BIAS:
	{
		if((maxBufferSize - idx) < 1)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->d.response.rc;
		break;
	}

	case DYN_PROTOCOL_EID_GET_SENSOR_DATA:
	{
		if((maxBufferSize - idx) < (1+1+1+4+64)) // need room for rc+sensorstatus+sensorid+timestamp+data
			goto error_size;
		outBuffer[idx++] = edata->d.response.sensorData.rc;
 		if(DynProtocol_encodeSensorEvent(self, edata, &outBuffer[idx], maxBufferSize - idx, DYN_PROTOCOL_ETYPE_RESP) == -1)
			goto error_arg;

		// GET_SENSOR_DATA response frame size is fixed:
		//  EVT_TYPE + EVT_ID + RC + SENSOR_STATUS + SENSORID + TIMESTAMP[0-3] + MAX_SENSOR_EVENT_DATA_SIZE (arbitrary fixed to 64B)
		idx = 1+1+1+1+1+4+64;
		break;
	}

	case DYN_PROTOCOL_EID_GET_SENSOR_CFG:
	{
		if((maxBufferSize - idx) < 68)
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->d.response.sensorcfg.rc;
		outBuffer[idx++] = (uint8_t)edata->d.response.sensorcfg.cfg.base.type;
		outBuffer[idx++] = (uint8_t)edata->d.response.sensorcfg.cfg.base.size;
		switch (edata->d.response.sensorcfg.cfg.base.type) {
		case VSENSOR_CONFIG_TYPE_OFFSET:
			outBuffer[idx++] = (uint8_t)edata->sensor_id;
			precision = DynProtocol_getPrecision(self, edata->sensor_id);
			DynProtocol_encodeQxToQyVect16((int32_t*)&edata->d.response.sensorcfg.cfg.buffer[0], 3, 16, precision, &outBuffer[idx]);
			break;
		default:
			// we don't know what is the value of cfg.size so let's make sure
			// there is enough room in outBuffer.
			if((uint32_t)(maxBufferSize - idx) < edata->d.response.sensorcfg.cfg.base.size)
				goto error_size;
			memcpy(&outBuffer[idx], edata->d.response.sensorcfg.cfg.buffer, edata->d.response.sensorcfg.cfg.base.size);
			break;
		}
		idx = 68 + 2;
		break;
	}

	case DYN_PROTOCOL_EID_GET_SENSOR_BIAS:
	{
		if ((maxBufferSize - idx) < 1 + 1 + 1 + (3 * 2))
			goto error_size;
		outBuffer[idx++] = (uint8_t)edata->d.response.sensorcfg.rc;
		outBuffer[idx++] = (uint8_t)edata->sensor_id;
		outBuffer[idx++] = (uint8_t)edata->d.response.sensorcfg.cfg.base.type;
		precision = DynProtocol_getPrecision(self, edata->sensor_id);
		idx += DynProtocol_encodeQxToQyVect16(&((VSensorConfigOffset *)&edata->d.response.sensorcfg.cfg)->vect[0], 3, 16, precision, &outBuffer[idx]);
		break;
	}

	default:
		goto error_arg;
	}

	*outBufferSize = (idx);

	return 0;

error_arg:
	INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: Unexpected argument for DynProtocol_encodeResponse()");
	return -1;

error_size:
	INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: output buffer size too small");
	return -1;
}

int DynProtocol_encodeAsync(DynProtocol_t * self,
		enum DynProtocolEid eid, const DynProtocolEdata_t * edata,
		uint8_t * outBuffer, uint16_t maxBufferSize, uint16_t *outBufferSize)
{
	uint16_t idx = 0;
	int len;

	(void)self;

	*outBufferSize = 0;

	if(maxBufferSize < 2)
		goto error_size;

	outBuffer[idx]  = EVENT_TYPE_ASYNC; // Set event type
	outBuffer[idx++] |= DYN_PROTOCOL_GROUP_ID & ~EVENT_TYPE_MASK; // Set group ID
	outBuffer[idx++] = (uint8_t)eid;

	switch(eid) {
	case DYN_PROTOCOL_EID_NEW_SENSOR_DATA:
	{
		len = DynProtocol_encodeSensorEvent(self, edata, &outBuffer[idx], maxBufferSize - idx, DYN_PROTOCOL_ETYPE_ASYNC);
		if(len == -1)
			goto error_arg;
		else if(len > maxBufferSize - idx)
			goto error_size;

		idx += len;
		break;
	}

	default:
		goto error_arg;
	}

	*outBufferSize = (idx);

	return 0;

error_arg:
	INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: Unexpected argument for encode_async()");
	return -1;

error_size:
	INV_MSG(INV_MSG_LEVEL_ERROR, "DynProtocol: output buffer size too small");
	return -1;
}

const char * DynProtocol_sensorTypeToStr(int type)
{
	switch(type) {
	case DYN_PRO_SENSOR_TYPE_RESERVED:               return "RESERVED";
	case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:          return "ACCELEROMETER";
	case DYN_PRO_SENSOR_TYPE_MAGNETOMETER:           return "MAGNETOMETER";
	case DYN_PRO_SENSOR_TYPE_ORIENTATION:            return "ORIENTATION";
	case DYN_PRO_SENSOR_TYPE_GYROSCOPE:              return "GYROSCOPE";
	case DYN_PRO_SENSOR_TYPE_LIGHT:                  return "LIGHT";
	case DYN_PRO_SENSOR_TYPE_PRESSURE:               return "PRESSURE";
	case DYN_PRO_SENSOR_TYPE_TEMPERATURE:            return "TEMPERATURE";
	case DYN_PRO_SENSOR_TYPE_PROXIMITY:              return "PROXIMITY";
	case DYN_PRO_SENSOR_TYPE_GRAVITY:                return "GRAVITY";
	case DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION:    return "LINEAR_ACCELERATION";
	case DYN_PRO_SENSOR_TYPE_ROTATION_VECTOR:        return "ROTATION_VECTOR";
	case DYN_PRO_SENSOR_TYPE_HUMIDITY:               return "HUMIDITY";
	case DYN_PRO_SENSOR_TYPE_AMBIENT_TEMPERATURE:    return "AMBIENT_TEMPERATURE";
	case DYN_PRO_SENSOR_TYPE_UNCAL_MAGNETOMETER:     return "UNCAL_MAGNETOMETER";
	case DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR:   return "GAME_ROTATION_VECTOR";
	case DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE:        return "UNCAL_GYROSCOPE";
	case DYN_PRO_SENSOR_TYPE_SMD:                    return "SMD";
	case DYN_PRO_SENSOR_TYPE_STEP_DETECTOR:          return "STEP_DETECTOR";
	case DYN_PRO_SENSOR_TYPE_STEP_COUNTER:           return "STEP_COUNTER";
	case DYN_PRO_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR: return "GEOMAG_ROTATION_VECTOR";
	case DYN_PRO_SENSOR_TYPE_HEART_RATE:             return "HEART_RATE";
	case DYN_PRO_SENSOR_TYPE_TILT_DETECTOR:          return "TILT_DETECTOR";
	case DYN_PRO_SENSOR_TYPE_WAKE_GESTURE:           return "WAKE_GESTURE";
	case DYN_PRO_SENSOR_TYPE_GLANCE_GESTURE:         return "GLANCE_GESTURE";
	case DYN_PRO_SENSOR_TYPE_PICK_UP_GESTURE:        return "PICK_UP_GESTURE";
	case DYN_PRO_SENSOR_TYPE_BAC:                    return "BAC";
	case DYN_PRO_SENSOR_TYPE_PDR:                    return "PDR";
	case DYN_PRO_SENSOR_TYPE_B2S:                    return "B2S";
	case DYN_PRO_SENSOR_TYPE_3AXIS:                  return "3AXIS";
	case DYN_PRO_SENSOR_TYPE_EIS:                    return "EIS";
	case DYN_PRO_SENSOR_TYPE_OIS:                    return "OIS";
	case DYN_PRO_SENSOR_TYPE_RAW_ACCELEROMETER:      return "RAW_ACCELEROMETER";
	case DYN_PRO_SENSOR_TYPE_RAW_GYROSCOPE:          return "RAW_GYROSCOPE";
	case DYN_PRO_SENSOR_TYPE_RAW_MAGNETOMETER:       return "RAW_MAGNETOMETER";
	case DYN_PRO_SENSOR_TYPE_RAW_TEMPERATURE:        return "RAW_TEMPERATURE";
	case DYN_PRO_SENSOR_TYPE_CUSTOM_PRESSURE:        return "CUSTOM_PRESSURE";
	case DYN_PRO_SENSOR_TYPE_MIC:                    return "MIC";
	case DYN_PRO_SENSOR_TYPE_TSIMU:                  return "TSIMU";
	case DYN_PRO_SENSOR_TYPE_RAW_PPG:                return "RAW_PPG";
	case DYN_PRO_SENSOR_TYPE_HRV:                    return "HRV";
	case DYN_PRO_SENSOR_TYPE_SLEEP_ANALYSIS:         return "SLEEP_ANALYSIS";
	case DYN_PRO_SENSOR_TYPE_BAC_EXTENDED:           return "BAC_EXTENDED";
	case DYN_PRO_SENSOR_TYPE_BAC_STATISTICS:         return "BAC_STATISTICS";
	case DYN_PRO_SENSOR_TYPE_FLOOR_CLIMB_COUNTER:    return "FLOOR_CLIMB_COUNTER";
	case DYN_PRO_SENSOR_TYPE_ENERGY_EXPENDITURE:     return "ENERGY_EXPENDITURE";
	case DYN_PRO_SENSOR_TYPE_DISTANCE:               return "DISTANCE";
	case DYN_PRO_SENSOR_TYPE_SHAKE:                  return "SHAKE";
	case DYN_PRO_SENSOR_TYPE_DOUBLE_TAP:             return "DOUBLE_TAP";
	case DYN_PRO_SENSOR_TYPE_CUSTOM0:                return "CUSTOM0";
	case DYN_PRO_SENSOR_TYPE_CUSTOM1:                return "CUSTOM1";
	case DYN_PRO_SENSOR_TYPE_CUSTOM2:                return "CUSTOM2";
	case DYN_PRO_SENSOR_TYPE_CUSTOM3:                return "CUSTOM3";
	case DYN_PRO_SENSOR_TYPE_CUSTOM4:                return "CUSTOM4";
	case DYN_PRO_SENSOR_TYPE_CUSTOM5:                return "CUSTOM5";
	case DYN_PRO_SENSOR_TYPE_CUSTOM6:                return "CUSTOM6";
	case DYN_PRO_SENSOR_TYPE_CUSTOM7:                return "CUSTOM7";
	case DYN_PRO_SENSOR_TYPE_WOM:                    return "WOM";
	case DYN_PRO_SENSOR_TYPE_SEDENTARY_REMIND:       return "SEDENTARY_REMIND";
	case DYN_PRO_SENSOR_TYPE_FSYNC_EVENT:            return "FSYNC_EVENT";
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_0:            return "PRED_QUAT_0";
	case DYN_PRO_SENSOR_TYPE_PRED_QUAT_1:            return "PRED_QUAT_1";
	default:                                         return "";
	}
}

