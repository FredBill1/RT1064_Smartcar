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

#ifndef _DYN_PROTOCOL_H_
#define _DYN_PROTOCOL_H_

#include <stdint.h>
#include "../VSensor/VSensorConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DYN_PROTOCOL_GROUP_ID  3
#define DYN_PROTOCOL_VERSION "0.0.1-dev1"

/** @brief Sensor status definition
 */
enum DynProSensorStatus
{
	DYN_PRO_SENSOR_STATUS_DATA_UPDATED      = 0,    /**< new sensor data */
	DYN_PRO_SENSOR_STATUS_STATE_CHANGED     = 1,    /**< dummy sensor data indicating
                                                     to a change in sensor state */
	DYN_PRO_SENSOR_STATUS_FLUSH_COMPLETE    = 2,    /**< dummy sensor data indicating
                                                     a end of batch after a manual flush */
	DYN_PRO_SENSOR_STATUS_POLLED_DATA       = 3,    /**< sensor data value after manual request */
};


/** @brief Sensor type identifier definition
 */
enum DynSensorType {
	DYN_PRO_SENSOR_TYPE_RESERVED                     = 0 ,  /**< Reserved ID: do not use */
	DYN_PRO_SENSOR_TYPE_ACCELEROMETER                = 1 ,  /**< Accelerometer */
	DYN_PRO_SENSOR_TYPE_MAGNETOMETER                 = 2 ,  /**< Magnetic field */
	DYN_PRO_SENSOR_TYPE_ORIENTATION                  = 3 ,  /**< Deprecated orientation */
	DYN_PRO_SENSOR_TYPE_GYROSCOPE                    = 4 ,  /**< Gyroscope */
	DYN_PRO_SENSOR_TYPE_LIGHT                        = 5 ,  /**< Ambient light sensor */
	DYN_PRO_SENSOR_TYPE_PRESSURE                     = 6 ,  /**< Barometer */
	DYN_PRO_SENSOR_TYPE_TEMPERATURE                  = 7 ,  /**< Temperature */
	DYN_PRO_SENSOR_TYPE_PROXIMITY                    = 8 ,  /**< Proximity */
	DYN_PRO_SENSOR_TYPE_GRAVITY                      = 9 ,  /**< Gravity */
	DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION          = 10,  /**< Linear acceleration */
	DYN_PRO_SENSOR_TYPE_ROTATION_VECTOR              = 11,  /**< Rotation vector */
	DYN_PRO_SENSOR_TYPE_HUMIDITY                     = 12,  /**< Relative humidity */
	DYN_PRO_SENSOR_TYPE_AMBIENT_TEMPERATURE          = 13,  /**< Ambient temperature */
	DYN_PRO_SENSOR_TYPE_UNCAL_MAGNETOMETER           = 14,  /**< Uncalibrated magnetic field */
	DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR         = 15,  /**< Game rotation vector */
	DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE              = 16,  /**< Uncalibrated gyroscope */
	DYN_PRO_SENSOR_TYPE_SMD                          = 17,  /**< Significant motion detection */
	DYN_PRO_SENSOR_TYPE_STEP_DETECTOR                = 18,  /**< Step detector */
	DYN_PRO_SENSOR_TYPE_STEP_COUNTER                 = 19,  /**< Step counter */
	DYN_PRO_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR       = 20,  /**< Geomagnetic rotation vector */
	DYN_PRO_SENSOR_TYPE_HEART_RATE                   = 21,  /**< Heart rate */
	DYN_PRO_SENSOR_TYPE_TILT_DETECTOR                = 22,  /**< Tilt detector */
	DYN_PRO_SENSOR_TYPE_WAKE_GESTURE                 = 23,  /**< Wake-up gesture  */
	DYN_PRO_SENSOR_TYPE_GLANCE_GESTURE               = 24,  /**< Glance gesture  */
	DYN_PRO_SENSOR_TYPE_PICK_UP_GESTURE              = 25,  /**< Pick-up gesture */
	DYN_PRO_SENSOR_TYPE_BAC                          = 26,  /**< Basic Activity Classifier */
	DYN_PRO_SENSOR_TYPE_PDR                          = 27,  /**< Pedestrian Dead Reckoning */
	DYN_PRO_SENSOR_TYPE_B2S                          = 28,  /**< Bring to see */
	DYN_PRO_SENSOR_TYPE_3AXIS                        = 29,  /**< 3 Axis sensor */
	DYN_PRO_SENSOR_TYPE_EIS                          = 30,  /**< Electronic Image Stabilization */
	DYN_PRO_SENSOR_TYPE_OIS                          = 31,  /**< Optical Image Stabilization */
	DYN_PRO_SENSOR_TYPE_RAW_ACCELEROMETER            = 32,  /**< Raw accelerometer */
	DYN_PRO_SENSOR_TYPE_RAW_GYROSCOPE                = 33,  /**< Raw gyroscope */
	DYN_PRO_SENSOR_TYPE_RAW_MAGNETOMETER             = 34,  /**< Raw magnetometer */
	DYN_PRO_SENSOR_TYPE_RAW_TEMPERATURE              = 35,  /**< Raw temperature */
	DYN_PRO_SENSOR_TYPE_CUSTOM_PRESSURE              = 36,  /**< Custom Pressure Sensor */
	DYN_PRO_SENSOR_TYPE_MIC                          = 37,  /**< Stream audio from microphone */
	DYN_PRO_SENSOR_TYPE_TSIMU                        = 38,  /**< TS-IMU */
	DYN_PRO_SENSOR_TYPE_RAW_PPG                      = 39,  /**< Raw Photoplethysmogram */
	DYN_PRO_SENSOR_TYPE_HRV                          = 40,  /**< Heart rate variability */
	DYN_PRO_SENSOR_TYPE_SLEEP_ANALYSIS               = 41,  /**< Sleep analysis */
	DYN_PRO_SENSOR_TYPE_BAC_EXTENDED                 = 42,  /**< Basic Activity Classifier Extended */
	DYN_PRO_SENSOR_TYPE_BAC_STATISTICS               = 43,  /**< Basic Activity Classifier Statistics */
	DYN_PRO_SENSOR_TYPE_FLOOR_CLIMB_COUNTER          = 44,  /**< Floor Climbed Counter */
	DYN_PRO_SENSOR_TYPE_ENERGY_EXPENDITURE           = 45,  /**< Energy Expenditure */
	DYN_PRO_SENSOR_TYPE_DISTANCE                     = 46,  /**< Distance */
	DYN_PRO_SENSOR_TYPE_SHAKE                        = 47,  /**< Shake Gesture */
	DYN_PRO_SENSOR_TYPE_DOUBLE_TAP                   = 48,  /**< Double Tap */
	DYN_PRO_SENSOR_TYPE_CUSTOM0,                            /**< Custom sensor ID 0 */
	DYN_PRO_SENSOR_TYPE_CUSTOM1,                            /**< Custom sensor ID 1 */
	DYN_PRO_SENSOR_TYPE_CUSTOM2,                            /**< Custom sensor ID 2 */
	DYN_PRO_SENSOR_TYPE_CUSTOM3,                            /**< Custom sensor ID 3 */
	DYN_PRO_SENSOR_TYPE_CUSTOM4,                            /**< Custom sensor ID 4 */
	DYN_PRO_SENSOR_TYPE_CUSTOM5,                            /**< Custom sensor ID 5 */
	DYN_PRO_SENSOR_TYPE_CUSTOM6,                            /**< Custom sensor ID 6 */
	DYN_PRO_SENSOR_TYPE_CUSTOM7,                            /**< Custom sensor ID 7 */
	DYN_PRO_SENSOR_TYPE_WOM,                                /**< Wake-up on motion */
	DYN_PRO_SENSOR_TYPE_SEDENTARY_REMIND,                   /**< Sedentary Remind */
	DYN_PRO_SENSOR_TYPE_DATA_ENCRYPTION,                    /**< Data Encryption */
	DYN_PRO_SENSOR_TYPE_FSYNC_EVENT,                        /**< FSYNC event */
	DYN_PRO_SENSOR_TYPE_HIGH_RATE_GYRO,                     /**< High Rate Gyro */
	DYN_PRO_SENSOR_TYPE_CUSTOM_BSCD,                        /**< Custom BAC StepCounter Calorie counter and Distance counter */
	DYN_PRO_SENSOR_TYPE_HRM_LOGGER,                         /**< HRM Logger */
	DYN_PRO_SENSOR_TYPE_PRED_QUAT_0,                        /**< Predictive Quaternion instance 0 */
	DYN_PRO_SENSOR_TYPE_PRED_QUAT_1,                        /**< Predictive Quaternion instance 1 */
	
	DYN_PRO_SENSOR_TYPE_MAX                                 /**< sentinel value for sensor type */
};



/** @brief Event identifier definition
 */
/* Commented label are currently not implemented */
enum DynProtocolEid {
	/* Protocol */
	DYN_PROTOCOL_EID_PROTOCOLVERSION = 0x00,

	/* IDD methods */
	DYN_PROTOCOL_EID_WHO_AM_I           = 0x10,
	DYN_PROTOCOL_EID_RESET              = 0x11,
	DYN_PROTOCOL_EID_SETUP              = 0x12,
	DYN_PROTOCOL_EID_CLEANUP            = 0x13,
	// DYN_PROTOCOL_EID_LOAD               = 0x14,
	DYN_PROTOCOL_EID_SELF_TEST          = 0x15,
	DYN_PROTOCOL_EID_GET_FW_INFO        = 0x16,
	DYN_PROTOCOL_EID_PING_SENSOR        = 0x17,
	// DYN_PROTOCOL_EID_SET_RUNNING_STATE  = 0x18,
	DYN_PROTOCOL_EID_START_SENSOR       = 0x19,
	DYN_PROTOCOL_EID_STOP_SENSOR        = 0x1A,
	DYN_PROTOCOL_EID_SET_SENSOR_PERIOD  = 0x1B,
	DYN_PROTOCOL_EID_SET_SENSOR_TIMEOUT = 0x1C,
	DYN_PROTOCOL_EID_FLUSH_SENSOR       = 0x1D,
	DYN_PROTOCOL_EID_SET_SENSOR_BIAS    = 0x1E,
	DYN_PROTOCOL_EID_GET_SENSOR_BIAS    = 0x1F,
	DYN_PROTOCOL_EID_SET_SENSOR_MMATRIX = 0x20,
	DYN_PROTOCOL_EID_GET_SENSOR_DATA    = 0x21,
	DYN_PROTOCOL_EID_GET_SW_REG         = 0x22,
	DYN_PROTOCOL_EID_SET_SENSOR_CFG     = 0x23,
	DYN_PROTOCOL_EID_GET_SENSOR_CFG     = 0x24,

	/* Events */
	DYN_PROTOCOL_EID_NEW_SENSOR_DATA    = 0x30,
};

/** @brief Event type definition
 */
enum DynProtocolEtype {
	DYN_PROTOCOL_ETYPE_CMD   = 0,
	DYN_PROTOCOL_ETYPE_RESP  = 1,
	DYN_PROTOCOL_ETYPE_ASYNC = 2
};

enum dynamic_protocol_ereg {
	DYN_PROTOCOL_EREG_MAJOR_VERSION   = 0,
	DYN_PROTOCOL_EREG_MINOR_VERSION,
	DYN_PROTOCOL_EREG_REV_VERSION,
	DYN_PROTOCOL_EREG_SUFFIX_VERSION,
	DYN_PROTOCOL_EREG_HANDSHAKE_SUPPORT
};

/** @brief Event data structure definition
 */
typedef struct DynProtocolEdata {
	int sensor_id; /** 0 if not applicable */
	union {
		union {
			uint32_t period;  /** for EID_SET_SENSOR_PERIOD */
			uint32_t timeout; /** for EID_SET_SENSOR_TIMEOUT */
			uint8_t regAddr;  /** for EID GET_SW_REG */
			VSensorConfigContext cfg; /** for EID_SET_SENSOR_CFG */
		} command;
		union {
			char version[16]; /** for EID_PROTOCOLVERSION */
			int rc;           /** return code */
			struct {          /** for EID GET_SENSOR_DATA */
				int rc;
				uint32_t status;    /** sensor status (see enum DynSensorStatus) */
				VSensorDataAny vdata;
			} sensorData;
			struct {          /** for EID_GET_SENSOR_CFG */
				int rc;
				VSensorConfigContext cfg;
			}sensorcfg;
		} response;           /** returned data if applicable */
		union {
			struct {
				uint32_t status;    /** sensor status (see enum DynSensorStatus) */
				VSensorDataAny vdata;
			} sensorEvent;          /** for EID_NEW_SENSOR_DATA */
			uint8_t buffer[128];
		} async;
	} d;
} DynProtocolEdata_t;

/** @brief Protocol Event callback prototype definition
 */
typedef void (*DynProtocolEvent_cb)(
		enum DynProtocolEtype etype,
		enum DynProtocolEid eid,
		const DynProtocolEdata_t * edata,
		void * cookie
	);

/** @brief Protocol handler definition
 */
typedef struct DynProtocol {
	DynProtocolEvent_cb event_cb;
	void * event_cb_cookie;
	struct {
		enum sm {
			PROTOCOL_STATE_IDLE = 0,
			PROTOCOL_STATE_GID = PROTOCOL_STATE_IDLE,
			PROTOCOL_STATE_CID,
			PROTOCOL_STATE_PAYLOAD,
		} state;
		uint16_t current_frame_size;
		uint8_t event_type;
		uint8_t group_id;
		uint8_t cmd_id;
		uint16_t expected_size;
		uint16_t received_size;
		uint8_t  tmp_buffer[256];
	} decode_state_machine;
	struct {
		uint8_t acc;
		uint8_t gyro;
	} precision;
} DynProtocol_t;

void DynProtocol_init(DynProtocol_t * self,
		DynProtocolEvent_cb event_cb, void * event_cb_cookie);

void DynProtocol_processReset(DynProtocol_t * self);

void DynProtocol_setCurrentFrameSize(DynProtocol_t * self, uint16_t frameSizeB);

int DynProtocol_setPrecision(DynProtocol_t * self, int sensor, uint8_t precision);

int DynProtocol_processPktByte(DynProtocol_t * self, uint8_t rcv_byte);

int DynProtocol_encodeAsync(DynProtocol_t * self,
		enum DynProtocolEid eid, const DynProtocolEdata_t * edata,
		uint8_t * outBuffer, uint16_t maxBufferSize, uint16_t *outBufferSize);

int DynProtocol_encodeResponse(DynProtocol_t * self,
		enum DynProtocolEid eid, const DynProtocolEdata_t * edata,
		uint8_t * outBuffer, uint16_t maxBufferSize, uint16_t *outBufferSize);

int DynProtocol_encodeCommand(DynProtocol_t * self,
		enum DynProtocolEid eid, const DynProtocolEdata_t * edata,
		uint8_t * outBuffer, uint16_t maxBufferSize, uint16_t *outBufferSize);

/** @brief Utility function that returns a string from a sensor type
 *  Empty string is returned if sensor is invalid
 */
const char * DynProtocol_sensorTypeToStr(int type);

int16_t inv_dc_little8_to_int16(const uint8_t * little8);

#ifdef __cplusplus
}
#endif

#endif /* _EMD_PROTOCOL_H_ */