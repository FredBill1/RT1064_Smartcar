/*
* ________________________________________________________________________________________________________
* Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively “Software”) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/

#include "Icm20948.h"
#include "Icm20948DataBaseControl.h"

#include "Icm20948AuxCompassAkm.h"
#include "Icm20948AuxTransport.h"

#include "Icm20948Augmented.h"
#include "Icm20948Dmp3Driver.h"

#include <string.h>

// BAC ped y ration for wearable, the value will influence pedometer result
#define BAC_PED_Y_RATIO_WEARABLE 1073741824

static int inv_enable_sensor_internal(struct inv_icm20948 * s, unsigned char androidSensor, unsigned char enable, char * mems_put_to_sleep);
static unsigned char sensor_needs_compass(unsigned char androidSensor);
static unsigned char sensor_needs_bac_algo(unsigned char androidSensor);
static int inv_set_hw_smplrt_dmp_odrs(struct inv_icm20948 * s);
static void inv_reGenerate_sensorControl(struct inv_icm20948 * s, const short *sen_num_2_ctrl, unsigned short *sensor_control, uint8_t header2_count);
static short get_multiple_56_rate(unsigned short delayInMs);

unsigned long inv_icm20948_ctrl_androidSensor_enabled(struct inv_icm20948 * s, unsigned char androidSensor)
{
	return s->inv_androidSensorsOn_mask[(androidSensor>>5)] & (1L << (androidSensor&0x1F));
}

typedef	struct {
	enum ANDROID_SENSORS AndroidSensor;
	enum INV_SENSORS     InvSensor;
}	MinDelayGenElementT;

#define MinDelayGen(s, list) MinDelayGenActual(s, list, sizeof(list) / sizeof (MinDelayGenElementT))

static unsigned short MinDelayGenActual(struct inv_icm20948 *s, const MinDelayGenElementT *element, unsigned long elementQuan)
{
	unsigned short minDelay = (unsigned short) -1;

	while(elementQuan--) {
		if (inv_icm20948_ctrl_androidSensor_enabled(s, element->AndroidSensor)) {
			unsigned short odrDelay = s->inv_dmp_odr_delays[element->InvSensor];

			if (minDelay > odrDelay)
					minDelay = odrDelay;
		}
		element++;
	} // end while elements to process

	return	minDelay;
}

static int DividerRateSet(struct inv_icm20948 *s, unsigned short minDelay, unsigned short hwSampleRateDivider, enum INV_SENSORS InvSensor)
{
	int result = 0;
	
	if (minDelay != 0xFFFF) {
		unsigned short dmpOdrDivider = (minDelay * 1125L) / (hwSampleRateDivider * 1000L); // a divider from (1125Hz/hw_smplrt_divider).

		s->inv_dmp_odr_dividers[InvSensor] = hwSampleRateDivider * dmpOdrDivider;
		result |= dmp_icm20948_set_sensor_rate(s, InvSensor, (dmpOdrDivider - 1));
	}
	
	return result;
}

static unsigned short SampleRateDividerGet(unsigned short minDelay)
{
	unsigned short delay = min(INV_ODR_MIN_DELAY, minDelay); // because of GYRO_SMPLRT_DIV which relies on 8 bits, we can't have ODR value higher than 200ms
	return delay * 1125L / 1000L; // a divider from 1125Hz.
}



/** @brief Get minimum ODR to be applied to accel engine based on all accel-based enabled sensors.
* @return ODR in ms we expect to be applied to accel engine
*/
static unsigned short getMinDlyAccel(struct inv_icm20948 *s)
{
	const MinDelayGenElementT MinDelayGenAccelList[] ={
		{ANDROID_SENSOR_ACCELEROMETER,                      INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_RAW_ACCELEROMETER,                  INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_WAKEUP_ACCELEROMETER,               INV_SENSOR_WAKEUP_ACCEL         },
		{ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,        INV_SENSOR_GEOMAG               },
		{ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR, INV_SENSOR_WAKEUP_GEOMAG        },
		{ANDROID_SENSOR_STEP_DETECTOR,                      INV_SENSOR_STEP_COUNTER         },
		{ANDROID_SENSOR_STEP_COUNTER,                       INV_SENSOR_STEP_COUNTER         },
		{ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,               INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_STEP_COUNTER,                INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,          INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,               INV_SENSOR_WAKEUP_TILT_DETECTOR },
		{ANDROID_SENSOR_GRAVITY,                            INV_SENSOR_SIXQ_accel           },
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,               INV_SENSOR_SIXQ_accel           },
		{ANDROID_SENSOR_LINEAR_ACCELERATION,                INV_SENSOR_SIXQ_accel           },
		{ANDROID_SENSOR_WAKEUP_GRAVITY,                     INV_SENSOR_WAKEUP_SIXQ_accel    },
		{ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,        INV_SENSOR_WAKEUP_SIXQ_accel    },
		{ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,         INV_SENSOR_WAKEUP_SIXQ_accel    },
		{ANDROID_SENSOR_ORIENTATION,                        INV_SENSOR_NINEQ_accel          },
		{ANDROID_SENSOR_ROTATION_VECTOR,                    INV_SENSOR_NINEQ_accel          },
		{ANDROID_SENSOR_WAKEUP_ORIENTATION,                 INV_SENSOR_WAKEUP_NINEQ_accel   },
		{ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,             INV_SENSOR_WAKEUP_NINEQ_accel   }
	};

	unsigned short lMinOdr = MinDelayGen(s, MinDelayGenAccelList);

	if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ACCELEROMETER))
		if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER))
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = min(s->odr_acc_ms,s->odr_racc_ms);
		else
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = s->odr_acc_ms;
	else
		if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER))
			s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = s->odr_racc_ms;

	if (s->bac_status != 0)
		lMinOdr = min(lMinOdr, s->inv_dmp_odr_delays[INV_SENSOR_ACTIVITY_CLASSIFIER]);
	if (s->flip_pickup_status != 0)
		lMinOdr = min(lMinOdr, s->inv_dmp_odr_delays[INV_SENSOR_FLIP_PICKUP]);
	if (s->b2s_status != 0)
		lMinOdr = min(lMinOdr, s->inv_dmp_odr_delays[INV_SENSOR_BRING_TO_SEE]);
	
	/** To have correct algorithm performance and quick convergence of GMRV, it is advised to set accelerometer to 225Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR) 
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) )
		lMinOdr = min(lMinOdr, 5);

	/** To have correct algorithm performance and quick convergence of RV, it is advised to set accelerometer to 225Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) 
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ROTATION_VECTOR) )
		lMinOdr = min(lMinOdr, 5);

	return lMinOdr;
}

/** @brief Get minimum ODR to be applied to gyro engine based on all gyro-based enabled sensors.
* @return ODR in ms we expect to be applied to gyro engine
*/
static unsigned short getMinDlyGyro(struct inv_icm20948 *s)
{
	const MinDelayGenElementT MinDelayGenGyroList[] = {
		{ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,        INV_SENSOR_GYRO              },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED, INV_SENSOR_WAKEUP_GYRO       },
		{ANDROID_SENSOR_GYROSCOPE,                     INV_SENSOR_CALIB_GYRO        },
		{ANDROID_SENSOR_RAW_GYROSCOPE,                 INV_SENSOR_GYRO              },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE,              INV_SENSOR_WAKEUP_CALIB_GYRO },
		{ANDROID_SENSOR_GRAVITY,                       INV_SENSOR_SIXQ              },
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,          INV_SENSOR_SIXQ              },
		{ANDROID_SENSOR_LINEAR_ACCELERATION,           INV_SENSOR_SIXQ              },
		{ANDROID_SENSOR_WAKEUP_GRAVITY,                INV_SENSOR_WAKEUP_SIXQ       },
		{ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,   INV_SENSOR_WAKEUP_SIXQ       },
		{ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,    INV_SENSOR_WAKEUP_SIXQ       },
		{ANDROID_SENSOR_ORIENTATION,                   INV_SENSOR_NINEQ             },
		{ANDROID_SENSOR_ROTATION_VECTOR,               INV_SENSOR_NINEQ             },
		{ANDROID_SENSOR_WAKEUP_ORIENTATION,            INV_SENSOR_WAKEUP_NINEQ      },
		{ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,        INV_SENSOR_WAKEUP_NINEQ      }
	};

	unsigned short lMinOdr = MinDelayGen(s, MinDelayGenGyroList);

	if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED))
		if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE))
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = min(s->odr_gyr_ms,s->odr_rgyr_ms);
		else
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = s->odr_gyr_ms;
	else
		if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE))
			s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = s->odr_rgyr_ms;

	/** To have correct algorithm performance and quick convergence of RV, it is advised to set gyro to 225Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) 
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ROTATION_VECTOR) )
		lMinOdr	= min(lMinOdr, 5);

	return lMinOdr;
}

/** @brief Get minimum ODR to be applied to compass engine based on all compass-based enabled sensors.
* @return ODR in ms we expect to be applied to compass engine
*/
static unsigned short getMinDlyCompass(struct inv_icm20948 *s)
{
	const MinDelayGenElementT MinDelayGenCpassList[] = {
		{ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,        INV_SENSOR_COMPASS              },
		{ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED, INV_SENSOR_WAKEUP_COMPASS       },
		{ANDROID_SENSOR_GEOMAGNETIC_FIELD,                  INV_SENSOR_CALIB_COMPASS        },
		{ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,              INV_SENSOR_WAKEUP_CALIB_COMPASS },
		{ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,        INV_SENSOR_GEOMAG_cpass         },
		{ANDROID_SENSOR_ORIENTATION,                        INV_SENSOR_NINEQ_cpass          },
		{ANDROID_SENSOR_ROTATION_VECTOR,                    INV_SENSOR_NINEQ_cpass          },
		{ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR, INV_SENSOR_WAKEUP_GEOMAG_cpass  },
		{ANDROID_SENSOR_WAKEUP_ORIENTATION,                 INV_SENSOR_WAKEUP_NINEQ_cpass   },
		{ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,             INV_SENSOR_WAKEUP_NINEQ_cpass   }
	};

	unsigned short lMinOdr = MinDelayGen(s, MinDelayGenCpassList);

	/** To have correct algorithm performance and quick convergence of GMRV, it is advised to set compass to 70Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR) 
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) )
		lMinOdr= min(lMinOdr, 15);
	/** To have correct algorithm performance and quick convergence of RV, it is advised to set compass to 35Hz.
	    In case power consumption is to be improved at the expense of performance, this setup should be commented out */
	if (   inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) 
		|| inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ROTATION_VECTOR) )
		lMinOdr = min(lMinOdr, 28);

	return lMinOdr;
}

int inv_icm20948_base_control_init(struct inv_icm20948 * s)
{
	int result = 0;
	unsigned int i;

	memset(s->inv_dmp_odr_dividers, 0, sizeof(s->inv_dmp_odr_dividers));
	
	for(i = 0; i < (sizeof(s->inv_dmp_odr_delays)/sizeof(unsigned short)); i++) {
		if((i == INV_SENSOR_ACTIVITY_CLASSIFIER) ||
		   (i == INV_SENSOR_STEP_COUNTER) ||
		   (i == INV_SENSOR_WAKEUP_STEP_COUNTER) ||
		   (i == INV_SENSOR_WAKEUP_TILT_DETECTOR) ||
		   (i == INV_SENSOR_FLIP_PICKUP) )
			s->inv_dmp_odr_delays[i] = INV_ODR_DEFAULT_BAC;
		else if(i == INV_SENSOR_BRING_TO_SEE)
			s->inv_dmp_odr_delays[i] = INV_ODR_DEFAULT_B2S;
		else
			s->inv_dmp_odr_delays[i] = INV_ODR_MIN_DELAY;
	}
	for(i = 0; i < (sizeof(s->inv_androidSensorsOdr_boundaries)/sizeof(s->inv_androidSensorsOdr_boundaries[0])); i++) {
		if ((i == ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) || (i == ANDROID_SENSOR_GEOMAGNETIC_FIELD) ||
		    (i == ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED) || (i == ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD)) {
			s->inv_androidSensorsOdr_boundaries[i][0] = INV_MIN_ODR_CPASS;
			s->inv_androidSensorsOdr_boundaries[i][1] = INV_MAX_ODR_CPASS;
		} else if ((i == ANDROID_SENSOR_GAME_ROTATION_VECTOR) || (i == ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR) ||
		           (i == ANDROID_SENSOR_GRAVITY) || (i == ANDROID_SENSOR_WAKEUP_GRAVITY) ||
		           (i == ANDROID_SENSOR_LINEAR_ACCELERATION) || (i == ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION) ||
		           (i == ANDROID_SENSOR_ROTATION_VECTOR) || (i == ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) ||
		           (i == ANDROID_SENSOR_ORIENTATION) || (i == ANDROID_SENSOR_WAKEUP_ORIENTATION)) {
			s->inv_androidSensorsOdr_boundaries[i][0] = INV_MIN_ODR_GRV;
			s->inv_androidSensorsOdr_boundaries[i][1] = INV_MAX_ODR_GRV;
		} else {
			s->inv_androidSensorsOdr_boundaries[i][0] = INV_MIN_ODR;
			s->inv_androidSensorsOdr_boundaries[i][1] = INV_MAX_ODR;
		}
	}
	s->lLastHwSmplrtDividerAcc = 0;
	s->lLastHwSmplrtDividerGyr = 0;
	s->sBatchMode              = 0;
	s->header2_count           = 0;
	s->mems_put_to_sleep       = 1;
	s->smd_status              = 0;
	s->ped_int_status          = 0;
	s->b2s_status              = 0;
	s->bac_request             = 0;
	s->odr_acc_ms = INV_ODR_MIN_DELAY;
	//s->odr_acc_wom_ms = INV_ODR_MIN_DELAY;
	s->odr_racc_ms = INV_ODR_MIN_DELAY;
	s->odr_gyr_ms = INV_ODR_MIN_DELAY;
	s->odr_rgyr_ms = INV_ODR_MIN_DELAY;

	return result;
}

static int inv_set_hw_smplrt_dmp_odrs(struct inv_icm20948 * s)
{
	int result = 0;
	unsigned short minDly, minDly_accel, minDly_gyro;
	unsigned short minDly_cpass;
	unsigned short minDly_pressure;
	unsigned short hw_smplrt_divider = 0;
	
	const MinDelayGenElementT MinDelayGenPressureList[] = {
		{ANDROID_SENSOR_PRESSURE,                           INV_SENSOR_PRESSURE             },
		{ANDROID_SENSOR_WAKEUP_PRESSURE,                    INV_SENSOR_WAKEUP_PRESSURE      }
	};
	const MinDelayGenElementT MinDelayGenAccel2List[] = {
		{ANDROID_SENSOR_ACCELEROMETER,                      INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_WAKEUP_ACCELEROMETER,               INV_SENSOR_WAKEUP_ACCEL         },
		{ANDROID_SENSOR_RAW_ACCELEROMETER,                  INV_SENSOR_ACCEL                },
		{ANDROID_SENSOR_LINEAR_ACCELERATION,                INV_SENSOR_SIXQ_accel           },
		{ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,         INV_SENSOR_WAKEUP_SIXQ_accel    }
	};
	const MinDelayGenElementT MinDelayGenAccel3List[] = {
		{ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,        INV_SENSOR_GEOMAG               },
		{ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR, INV_SENSOR_WAKEUP_GEOMAG        }
	};
	const MinDelayGenElementT MinDelayGenAccel4List[] = {
		{ANDROID_SENSOR_STEP_DETECTOR,                      INV_SENSOR_STEP_COUNTER         },
		{ANDROID_SENSOR_STEP_COUNTER,                       INV_SENSOR_STEP_COUNTER         },
		{ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,               INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_STEP_COUNTER,                INV_SENSOR_WAKEUP_STEP_COUNTER  },
		{ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,          INV_SENSOR_WAKEUP_STEP_COUNTER  }
	};
	const MinDelayGenElementT MinDelayGenGyro2List[] = {
		{ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,             INV_SENSOR_GYRO                 },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,      INV_SENSOR_WAKEUP_GYRO          },
		{ANDROID_SENSOR_GYROSCOPE,                          INV_SENSOR_CALIB_GYRO           },
		{ANDROID_SENSOR_RAW_GYROSCOPE,                      INV_SENSOR_GYRO           },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE,                   INV_SENSOR_WAKEUP_CALIB_GYRO    }
	};
	const MinDelayGenElementT MinDelayGenGyro3List[] = {
		{ANDROID_SENSOR_GYROSCOPE,                          INV_SENSOR_CALIB_GYRO           },
		{ANDROID_SENSOR_WAKEUP_GYROSCOPE,                   INV_SENSOR_WAKEUP_CALIB_GYRO    }
	};
	const MinDelayGenElementT MinDelayGenGyro4List[] = {
		{ANDROID_SENSOR_GRAVITY,                            INV_SENSOR_SIXQ                 },
		{ANDROID_SENSOR_GAME_ROTATION_VECTOR,               INV_SENSOR_SIXQ                 },
		{ANDROID_SENSOR_LINEAR_ACCELERATION,                INV_SENSOR_SIXQ                 },
		{ANDROID_SENSOR_WAKEUP_GRAVITY,                     INV_SENSOR_WAKEUP_SIXQ          },
		{ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,        INV_SENSOR_WAKEUP_SIXQ          },
		{ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,         INV_SENSOR_WAKEUP_SIXQ          }
	};
	const MinDelayGenElementT MinDelayGenGyro5List[] = {
		{ANDROID_SENSOR_ORIENTATION,                        INV_SENSOR_NINEQ                },
		{ANDROID_SENSOR_ROTATION_VECTOR,                    INV_SENSOR_NINEQ                },
		{ANDROID_SENSOR_WAKEUP_ORIENTATION,                 INV_SENSOR_WAKEUP_NINEQ         },
		{ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,             INV_SENSOR_WAKEUP_NINEQ         }
	};
	const MinDelayGenElementT MinDelayGenCpass2List[] = {
		{ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,        INV_SENSOR_COMPASS              },
		{ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,	INV_SENSOR_WAKEUP_COMPASS       }
	};
	const MinDelayGenElementT MinDelayGenCpass3List[] = {
		{ANDROID_SENSOR_GEOMAGNETIC_FIELD,                  INV_SENSOR_CALIB_COMPASS        },
		{ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,              INV_SENSOR_WAKEUP_CALIB_COMPASS }
	};
	const MinDelayGenElementT MinDelayGenPressure2List[] = {
		{ANDROID_SENSOR_PRESSURE,                           INV_SENSOR_PRESSURE             },
		{ANDROID_SENSOR_WAKEUP_PRESSURE,                    INV_SENSOR_WAKEUP_PRESSURE      }
	};
	
	// Engine ACCEL Based
	minDly_accel = getMinDlyAccel(s);

	// Engine Gyro Based
	minDly_gyro  = getMinDlyGyro(s);

	// Engine Cpass Based	
	minDly_cpass = getMinDlyCompass(s);

	// Engine Pressure Based	
	minDly_pressure	=	MinDelayGen	(s, MinDelayGenPressureList);

	// get min delay of all enabled sensors of all sensor engine groups
	minDly = min(minDly_gyro, minDly_accel);
	minDly = min(minDly, minDly_cpass);
	minDly = min(minDly, minDly_pressure);
	
	// switch between low power and low noise at 500Hz boundary
	if (minDly != 0xFFFF) {
		// above 500Hz boundary, force LN mode
		if (minDly==1) {
			if (s->base_state.chip_lp_ln_mode == CHIP_LOW_POWER_ICM20948) {
				s->go_back_lp_when_odr_low = 1;
				inv_icm20948_enter_low_noise_mode(s);
			}
		} else { // below 500 Hz boundary, go back to originally requested mode
			if (s->go_back_lp_when_odr_low) {
				s->go_back_lp_when_odr_low = 0;
				inv_icm20948_enter_duty_cycle_mode(s);
			}	
		}
	} else // all sensors are turned OFF, force originally requested mode
	{
		if (s->go_back_lp_when_odr_low) {
			s->go_back_lp_when_odr_low = 0;
			inv_icm20948_enter_duty_cycle_mode(s);
		}
	}
	
	if (minDly_accel != 0xFFFF)    minDly_accel = minDly;
	if (minDly_gyro  != 0xFFFF)    minDly_gyro  = minDly;
	if (minDly_cpass != 0xFFFF)    minDly_cpass = minDly;
	if (minDly_pressure != 0xFFFF) minDly_pressure = minDly;

	if (s->bac_request != 0) {
		unsigned short lBACMinDly = min(INV_ODR_DEFAULT_BAC, minDly_accel);
		// estimate closest decimator value to have 56Hz multiple and apply it
		lBACMinDly = 1000/(get_multiple_56_rate(lBACMinDly));
		dmp_icm20948_set_bac_rate(s, get_multiple_56_rate(lBACMinDly));
		minDly_accel = lBACMinDly;
		hw_smplrt_divider = SampleRateDividerGet(minDly_accel);
		result |= DividerRateSet(s, lBACMinDly, hw_smplrt_divider, INV_SENSOR_ACTIVITY_CLASSIFIER);
	}
	if (s->b2s_status != 0) {
		unsigned short lB2SMinDly = min(INV_ODR_DEFAULT_B2S, minDly_accel);
		lB2SMinDly = 1000/(get_multiple_56_rate(lB2SMinDly));
		dmp_icm20948_set_b2s_rate(s, get_multiple_56_rate(lB2SMinDly));
		minDly_accel = lB2SMinDly;
		hw_smplrt_divider = SampleRateDividerGet(minDly_accel);
		result |= DividerRateSet(s, lB2SMinDly, hw_smplrt_divider, INV_SENSOR_BRING_TO_SEE);
	}

	// set odrs for each enabled sensors

	// Engine ACCEL Based
	if (minDly_accel != 0xFFFF)	{ // 0xFFFF -- none accel based sensor enable
		hw_smplrt_divider = SampleRateDividerGet(minDly_accel);

		if (hw_smplrt_divider != s->lLastHwSmplrtDividerAcc) {
			
			result |= inv_icm20948_ctrl_set_accel_quaternion_gain(s, hw_smplrt_divider);
			result |= inv_icm20948_ctrl_set_accel_cal_params(s, hw_smplrt_divider);
			result |= inv_icm20948_set_accel_divider(s, hw_smplrt_divider - 1);
			s->lLastHwSmplrtDividerAcc = hw_smplrt_divider;
		}

		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenAccel2List), hw_smplrt_divider, INV_SENSOR_ACCEL);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenAccel3List), hw_smplrt_divider, INV_SENSOR_GEOMAG);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenAccel4List), hw_smplrt_divider, INV_SENSOR_STEP_COUNTER);
		
	}

	// Engine Gyro Based
	if (minDly_gyro != 0xFFFF) { // 0xFFFF -- none gyro based sensor enable
		hw_smplrt_divider = SampleRateDividerGet(minDly_gyro);

		if (hw_smplrt_divider != s->lLastHwSmplrtDividerGyr) {
			result |= inv_icm20948_set_gyro_divider(s, (unsigned char)(hw_smplrt_divider - 1));
			s->lLastHwSmplrtDividerGyr = hw_smplrt_divider;
		}

		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro2List), hw_smplrt_divider, INV_SENSOR_GYRO);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro3List), hw_smplrt_divider, INV_SENSOR_CALIB_GYRO);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro4List), hw_smplrt_divider, INV_SENSOR_SIXQ);
		result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenGyro5List), hw_smplrt_divider, INV_SENSOR_NINEQ);
	}

	// Engine Cpass and Pressure Based	
	if ((minDly_cpass != 0xFFFF) || (minDly_pressure != 0xFFFF)) {
		unsigned int lI2cEffectiveDivider = 0;

		// if compass or pressure are alone, compute 1st stage divider, otherwise it will be taken from accel or gyro
		if ( (minDly_accel == 0xFFFF) && (minDly_gyro == 0xFFFF) )
			hw_smplrt_divider = SampleRateDividerGet(minDly);

		// Apply compass or pressure ODR to I2C and get effective ODR
		// so that 2nd level of divider can take into account real frequency we can expect
		// to determine its divider value
		result |= inv_icm20948_secondary_set_odr(s, hw_smplrt_divider, &lI2cEffectiveDivider);

		// if compass or pressure are alone, recompute 1st stage divider based on configured divider for I2C
		// otherwise divider is taken from accel or gyro, so there is no need to recompute effective divider value
		// based on the divider we just applied
		if ( (minDly_accel == 0xFFFF) && (minDly_gyro == 0xFFFF) )
			hw_smplrt_divider = lI2cEffectiveDivider;

		if (minDly_cpass != 0xFFFF) {
			result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenCpass2List), hw_smplrt_divider, INV_SENSOR_COMPASS);
			result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenCpass3List), hw_smplrt_divider, INV_SENSOR_CALIB_COMPASS);
		}

		if (minDly_pressure != 0xFFFF)
			result |= DividerRateSet(s, MinDelayGen(s, MinDelayGenPressure2List), hw_smplrt_divider, INV_SENSOR_PRESSURE);
	}

	return result;
}

static short get_multiple_56_rate(unsigned short delayInMs)
{
	short lfreq = 0;

	// > 1KHz
	if( delayInMs < 2 ){
	lfreq = DMP_ALGO_FREQ_900;
	}
	// 225Hz - 500Hz
	else if(( delayInMs >= 2 ) && ( delayInMs < 4 )){
	lfreq = DMP_ALGO_FREQ_450;
	}
	// 112Hz - 225Hz
	else if(( delayInMs >= 4 ) && ( delayInMs < 8 )){
	lfreq = DMP_ALGO_FREQ_225;
	}
	// 56Hz - 112Hz
	else if(( delayInMs >= 8 ) && ( delayInMs < 17 )){
	lfreq = DMP_ALGO_FREQ_112;
	}
	// < 56Hz
	else if(delayInMs >= 17){
	lfreq = DMP_ALGO_FREQ_56;
	}
	
	return lfreq;
}

int inv_icm20948_set_odr(struct inv_icm20948 * s, unsigned char androidSensor, unsigned short delayInMs)
{
	int result;

	if(sensor_needs_compass(androidSensor))
		if(!inv_icm20948_get_compass_availability(s))
			return -1;
	
	//check if sensor is bac algo dependant
	if(sensor_needs_bac_algo(androidSensor)) {
		// set odr for sensors using BAC (1/56)
		delayInMs = INV_ODR_DEFAULT_BAC;
	}
	
	inv_icm20948_prevent_lpen_control(s);

	// check that requested ODR is within the allowed limits
	if (delayInMs < s->inv_androidSensorsOdr_boundaries[androidSensor][0]) delayInMs = s->inv_androidSensorsOdr_boundaries[androidSensor][0];
	if (delayInMs > s->inv_androidSensorsOdr_boundaries[androidSensor][1]) delayInMs = s->inv_androidSensorsOdr_boundaries[androidSensor][1];
	switch (androidSensor) {
		case ANDROID_SENSOR_ACCELEROMETER:
			if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER))
				s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = min(delayInMs,s->odr_racc_ms);
			else
				s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = delayInMs;
			s->odr_acc_ms = delayInMs;
			break;
		case ANDROID_SENSOR_RAW_ACCELEROMETER:
			if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ACCELEROMETER))
				s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = min(delayInMs,s->odr_acc_ms);
			else
				s->inv_dmp_odr_delays[INV_SENSOR_ACCEL] = delayInMs;
			s->odr_racc_ms = delayInMs;
			break;

		case ANDROID_SENSOR_STEP_DETECTOR:
		case ANDROID_SENSOR_STEP_COUNTER:
			s->inv_dmp_odr_delays[INV_SENSOR_STEP_COUNTER] = delayInMs;
			break;

		case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
			s->inv_dmp_odr_delays[INV_SENSOR_GEOMAG] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_GEOMAG_cpass] = delayInMs;
			break;

		case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON:
			s->inv_dmp_odr_delays[INV_SENSOR_ACTIVITY_CLASSIFIER] = delayInMs;
			break;

		case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:
			if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE))
				s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = min(delayInMs,s->odr_rgyr_ms);
			else
				s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = delayInMs;
			s->odr_gyr_ms = delayInMs;
			break;
		case ANDROID_SENSOR_RAW_GYROSCOPE:
			if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED))
				s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = min(delayInMs,s->odr_gyr_ms);
			else
				s->inv_dmp_odr_delays[INV_SENSOR_GYRO] = delayInMs;
			s->odr_rgyr_ms = delayInMs;
			break;
		case ANDROID_SENSOR_GYROSCOPE:
			s->inv_dmp_odr_delays[INV_SENSOR_CALIB_GYRO] = delayInMs;
			break;

		case ANDROID_SENSOR_GRAVITY:
		case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
		case ANDROID_SENSOR_LINEAR_ACCELERATION:
			// if augmented sensors are handled by this driver,
			// then the fastest 6quat-based sensor which is enabled
			// should be applied to all 6quat-based sensors
			delayInMs = inv_icm20948_augmented_sensors_set_odr(s, androidSensor, delayInMs);
			s->inv_dmp_odr_delays[INV_SENSOR_SIXQ] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_SIXQ_accel] = delayInMs;
			break;

		case ANDROID_SENSOR_ORIENTATION:
		case ANDROID_SENSOR_ROTATION_VECTOR:
			// if augmented sensors are handled by this driver,
			// then the fastest 9quat-based sensor which is enabled
			// should be applied to all 9quat-based sensors
			delayInMs = inv_icm20948_augmented_sensors_set_odr(s, androidSensor, delayInMs);
			s->inv_dmp_odr_delays[INV_SENSOR_NINEQ] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_NINEQ_accel] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_NINEQ_cpass] = delayInMs;
			break;

		case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
			s->inv_dmp_odr_delays[INV_SENSOR_COMPASS] = delayInMs;
			break;

		case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
			s->inv_dmp_odr_delays[INV_SENSOR_CALIB_COMPASS] = delayInMs;
			break;

		case ANDROID_SENSOR_LIGHT:
		case ANDROID_SENSOR_PROXIMITY:
			s->inv_dmp_odr_delays[INV_SENSOR_ALS] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_ACCELEROMETER:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_ACCEL] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_STEP_DETECTOR:
		case ANDROID_SENSOR_WAKEUP_STEP_COUNTER:
		case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_STEP_COUNTER] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_GEOMAG] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_GEOMAG_cpass] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_TILT_DETECTOR] = delayInMs;
			break;

		case ANDROID_SENSOR_B2S:
			s->inv_dmp_odr_delays[INV_SENSOR_BRING_TO_SEE] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_GYRO] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_GYROSCOPE:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_CALIB_GYRO] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_GRAVITY:
		case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
		case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
			// if augmented sensors are handled by this driver,
			// then the fastest 6quat-based sensor which is enabled
			// should be applied to all 6quat-based sensors
			delayInMs = inv_icm20948_augmented_sensors_set_odr(s, androidSensor, delayInMs);
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ_accel] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_ORIENTATION:
		case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
			// if augmented sensors are handled by this driver,
			// then the fastest 9quat-based sensor which is enabled
			// should be applied to all 9quat-based sensors
			delayInMs = inv_icm20948_augmented_sensors_set_odr(s, androidSensor, delayInMs);
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_accel] = delayInMs;
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_cpass] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_COMPASS] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_CALIB_COMPASS] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_LIGHT:
		case ANDROID_SENSOR_WAKEUP_PROXIMITY:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_ALS] = delayInMs;
			break;

		case ANDROID_SENSOR_PRESSURE:
			s->inv_dmp_odr_delays[INV_SENSOR_PRESSURE] = delayInMs;
			break;

		case ANDROID_SENSOR_WAKEUP_PRESSURE:
			s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_PRESSURE] = delayInMs;
			break;

		case ANDROID_SENSOR_FLIP_PICKUP:
			s->inv_dmp_odr_delays[INV_SENSOR_FLIP_PICKUP] = delayInMs;
			break;

		// not support yet
		case ANDROID_SENSOR_META_DATA:
		case ANDROID_SENSOR_TEMPERATURE:
		case ANDROID_SENSOR_AMBIENT_TEMPERATURE:
		case ANDROID_SENSOR_HUMIDITY:
		case ANDROID_SENSOR_HEART_RATE:
		case ANDROID_SENSOR_SCREEN_ROTATION:
		case ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE:
		case ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY:
		case ANDROID_SENSOR_WAKEUP_HEART_RATE:
			break;

		default:
			break;
	}

	result = inv_set_hw_smplrt_dmp_odrs(s);
	result |= inv_icm20948_set_gyro_sf(s, inv_icm20948_get_gyro_divider(s), inv_icm20948_get_gyro_fullscale(s));

	// debug get odr
	// result should be SAME as you entered in Ms in the Rolldice console
	// i.e. If you use: O a 63 [ Press capital O then 'a' then 63 then ENTER]
	// You should get the nearest number to 63 here if you debug  the 'test_odr'  

	//inv_icm20948_ctrl_get_odr( androidSensor, &test_odr );

	inv_icm20948_allow_lpen_control(s);
	return result;
}

/*
   inv_icm20948_ctrl_get_odr(s)
   Function to Query DMP3 DataRate (ODR)
   
   *odr = inv_icm20948_get_odr_in_units( );

    The result in odr_units saved in *odr param
*/
int inv_icm20948_ctrl_get_odr(struct inv_icm20948 * s, unsigned char SensorId, uint32_t *odr, enum INV_ODR_TYPE odr_units)
{
	int result=0;

	if(!odr) // sanity
		return -1;

	*odr = 0;

	/*
	You can obtain the odr in Milliseconds, Micro Seconds or Ticks.
	Use the enum values: ODR_IN_Ms, ODR_IN_Us or ODR_IN_Ticks,
	when calling inv_icm20948_get_odr_in_units().
	*/

	switch (SensorId) {
		case ANDROID_SENSOR_ACCELEROMETER:
		case ANDROID_SENSOR_RAW_ACCELEROMETER:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_ACCEL] , odr_units );
			break;

		case ANDROID_SENSOR_STEP_DETECTOR:
		case ANDROID_SENSOR_STEP_COUNTER:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_STEP_COUNTER] , odr_units );
			break;

		case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_GEOMAG] , odr_units );            
			break;

		case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_ACTIVITY_CLASSIFIER] , odr_units );
			break;

		case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:
		case ANDROID_SENSOR_RAW_GYROSCOPE:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_GYRO] , odr_units );
			break;

		case ANDROID_SENSOR_GYROSCOPE:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_CALIB_GYRO] , odr_units );
			break;

		case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
		case ANDROID_SENSOR_GRAVITY:
		case ANDROID_SENSOR_LINEAR_ACCELERATION:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_SIXQ] , odr_units );
			break;

		case ANDROID_SENSOR_ORIENTATION:
		case ANDROID_SENSOR_ROTATION_VECTOR:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_NINEQ] , odr_units );
			break;

		case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_COMPASS] , odr_units );
			break;

		case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_CALIB_COMPASS] , odr_units );
			break;

		case ANDROID_SENSOR_LIGHT:
		case ANDROID_SENSOR_PROXIMITY:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_ALS] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_ACCELEROMETER:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_ACCEL] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_STEP_DETECTOR:
		case ANDROID_SENSOR_WAKEUP_STEP_COUNTER:
		case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_STEP_COUNTER] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_GEOMAG] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_TILT_DETECTOR] , odr_units );
			break;

		case ANDROID_SENSOR_B2S:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_BRING_TO_SEE] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_GYRO] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_GYROSCOPE:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_CALIB_GYRO] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_SIXQ] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_GRAVITY:
		case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_SIXQ_accel] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_ORIENTATION:
		case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_NINEQ] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_COMPASS] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_CALIB_COMPASS] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_LIGHT:
		case ANDROID_SENSOR_WAKEUP_PROXIMITY:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_ALS] , odr_units );
			break;

		case ANDROID_SENSOR_PRESSURE:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_PRESSURE] , odr_units );
			break;

		case ANDROID_SENSOR_WAKEUP_PRESSURE:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_WAKEUP_PRESSURE] , odr_units );
			break;

		case ANDROID_SENSOR_FLIP_PICKUP:
			*odr = inv_icm20948_get_odr_in_units(s, s->inv_dmp_odr_dividers[INV_SENSOR_FLIP_PICKUP] , odr_units ); 
			break;

		// not support yet
		case ANDROID_SENSOR_META_DATA:
		case ANDROID_SENSOR_TEMPERATURE:
		case ANDROID_SENSOR_AMBIENT_TEMPERATURE:
		case ANDROID_SENSOR_HUMIDITY:
		case ANDROID_SENSOR_HEART_RATE:
		case ANDROID_SENSOR_SCREEN_ROTATION:
		case ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE:
		case ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY:
		case ANDROID_SENSOR_WAKEUP_HEART_RATE:
			*odr=0;
			break;

		default:
			*odr=0;
	}

	return result;
}

static void inv_reGenerate_sensorControl(struct inv_icm20948 * s, const short *sen_num_2_ctrl, unsigned short *sensor_control, uint8_t header2_count)
{
	short delta;
	int i, cntr;
	unsigned long tmp_androidSensorsOn_mask;

	//check if only header2 still remaining
	if(header2_count)
		*sensor_control = HEADER2_SET;
	else
		*sensor_control = 0;
	for (i = 0; i < 2; i++) {
		cntr = 32 * i;
		tmp_androidSensorsOn_mask = s->inv_androidSensorsOn_mask[i];
		while (tmp_androidSensorsOn_mask) {
			if (tmp_androidSensorsOn_mask & 1) {
				delta = sen_num_2_ctrl[cntr];
				if (delta != -1) *sensor_control |= delta;
			}
			tmp_androidSensorsOn_mask >>= 1;
			cntr++;
		}
	}
}

/** Computes the sensor control register that needs to be sent to the DMP
* @param[in] androidSensor A sensor number, the numbers correspond to sensors.h definition in Android
* @param[in] enable non-zero to turn sensor on, 0 to turn sensor off
* @param[in] sen_num_2_ctrl Table matching android sensor number to bits in DMP control register
* @param[in,out] sensor_control Sensor control register to write to DMP to enable/disable sensors
*/
static void inv_convert_androidSensor_to_control(struct inv_icm20948 * s, unsigned char androidSensor, unsigned char enable, const short *sen_num_2_ctrl, unsigned short *sensor_control)
{
	short delta = 0;

	if (androidSensor == ANDROID_SENSOR_ACTIVITY_CLASSIFICATON || androidSensor == ANDROID_SENSOR_FLIP_PICKUP || 
			androidSensor == ANDROID_SENSOR_WAKEUP_TILT_DETECTOR || androidSensor == ANDROID_SENSOR_B2S) {
		if (enable) {
			*sensor_control |= HEADER2_SET;
			//we increment counter
			s->header2_count ++;
		}
		else {
			s->header2_count --;
			// control has to be regenerated when removing sensors because of overlap
			inv_reGenerate_sensorControl(s, sen_num_2_ctrl, sensor_control, s->header2_count);
		}
	}

	if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
		return; // Sensor not supported

	delta = sen_num_2_ctrl[androidSensor];
	if (delta == -1)
		return; // This sensor not supported

	if (enable) {
		s->inv_androidSensorsOn_mask[(androidSensor>>5)] |= 1L << (androidSensor & 0x1F); // Set bit
		*sensor_control |= delta;
	}
	else {
		s->inv_androidSensorsOn_mask[(androidSensor>>5)] &= ~(1L << (androidSensor & 0x1F)); // Clear bit
		// control has to be regenerated when removing sensors because of overlap
		inv_reGenerate_sensorControl(s, sen_num_2_ctrl, sensor_control, s->header2_count);
	}

	return;
}

int inv_icm20948_ctrl_enable_sensor(struct inv_icm20948 * s, unsigned char androidSensor, unsigned char enable)
{
	int result = 0;

	if(sensor_needs_compass(androidSensor))
		if(!inv_icm20948_get_compass_availability(s))
			return -1;

	inv_icm20948_prevent_lpen_control(s);
	if( s->mems_put_to_sleep ) {
		s->mems_put_to_sleep = 0;
		result |= inv_icm20948_wakeup_mems(s);
	}
	result |= inv_enable_sensor_internal(s, androidSensor, enable, &s->mems_put_to_sleep);
	inv_icm20948_allow_lpen_control(s);
	return result;
}

static int inv_enable_sensor_internal(struct inv_icm20948 * s, unsigned char androidSensor, unsigned char enable, char * mems_put_to_sleep)
{
	int result = 0;
	unsigned short inv_event_control = 0;
	unsigned short data_rdy_status = 0;
	unsigned long steps=0;
	const short inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX]=
	{
		// Unsupported Sensors are -1
		-1, // Meta Data
		-32760, //0x8008, // Accelerometer
		0x0028, // Magnetic Field
		0x0408, // Orientation
		0x4048, // Gyroscope
		0x1008, // Light
		0x0088, // Pressure
		-1, // Temperature
		-1, // Proximity <----------- fixme
		0x0808, // Gravity
		-30712, // 0x8808, // Linear Acceleration
		0x0408, // Rotation Vector
		-1, // Humidity
		-1, // Ambient Temperature
		0x2008, // Magnetic Field Uncalibrated
		0x0808, // Game Rotation Vector
		0x4008, // Gyroscope Uncalibrated
		0, // Significant Motion
		0x0018, // Step Detector
		0x0010, // Step Counter <----------- fixme
		0x0108, // Geomagnetic Rotation Vector
		-1, //ANDROID_SENSOR_HEART_RATE,
		-1, //ANDROID_SENSOR_PROXIMITY,

		-32760, // ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
		0x0028, // ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
		0x0408, // ANDROID_SENSOR_WAKEUP_ORIENTATION,
		0x4048, // ANDROID_SENSOR_WAKEUP_GYROSCOPE,
		0x1008, // ANDROID_SENSOR_WAKEUP_LIGHT,
		0x0088, // ANDROID_SENSOR_WAKEUP_PRESSURE,
		0x0808, // ANDROID_SENSOR_WAKEUP_GRAVITY,
		-30712, // ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
		0x0408, // ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
		-1,		// ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
		-1,		// ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
		0x2008, // ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
		0x0808, // ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
		0x4008, // ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
		0x0018, // ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
		0x0010, // ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
		0x0108, // ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
		-1,		// ANDROID_SENSOR_WAKEUP_HEART_RATE,
		0,		// ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
		0x8008, // Raw Acc
		0x4048, // Raw Gyr
	};
	if(enable && !inv_icm20948_ctrl_androidSensor_enabled(s, androidSensor))
		s->skip_sample[inv_icm20948_sensor_android_2_sensor_type(androidSensor)] = 1;
		
	if (androidSensor == ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION) {
		if (enable) {
			s->smd_status = INV_SMD_EN;
			s->bac_request ++;
		}
		else {
			s->smd_status = 0;
			s->bac_request --;
		}
	}

	if (androidSensor == ANDROID_SENSOR_STEP_DETECTOR) {
		if (enable) {
			s->ped_int_status = INV_PEDOMETER_INT_EN;
			s->bac_request ++;
		}
		else {
			s->ped_int_status = 0;
			s->bac_request --;
		}
	}
	
	if (androidSensor == ANDROID_SENSOR_STEP_COUNTER) {
		if (enable) {
			s->bac_request ++;
		}
		else {
			s->bac_request --;
		}
	}

	if (androidSensor == ANDROID_SENSOR_FLIP_PICKUP) {
		if (enable){
			s->flip_pickup_status = FLIP_PICKUP_SET;
		}
		else
			s->flip_pickup_status = 0;
	}

	if (androidSensor == ANDROID_SENSOR_B2S) {
		if(enable){
			s->b2s_status = INV_BTS_EN;
			s->bac_request ++;
		}
		else {
			s->b2s_status = 0;
			s->bac_request --;
		}
	}
	if (androidSensor == ANDROID_SENSOR_ACTIVITY_CLASSIFICATON)
		inv_icm20948_ctrl_enable_activity_classifier(s, enable);

	if (androidSensor == ANDROID_SENSOR_WAKEUP_TILT_DETECTOR)
		inv_icm20948_ctrl_enable_tilt(s, enable);

	inv_convert_androidSensor_to_control(s, androidSensor, enable, inv_androidSensor_to_control_bits, &s->inv_sensor_control);
	result = dmp_icm20948_set_data_output_control1(s, s->inv_sensor_control);
	if (s->b2s_status)
		result |= dmp_icm20948_set_data_interrupt_control(s, s->inv_sensor_control|0x8008);
		// result |= dmp_icm20948_set_data_interrupt_control(s, s->inv_sensor_control|0x0000);
	else
		result |= dmp_icm20948_set_data_interrupt_control(s, s->inv_sensor_control);

	if (s->inv_sensor_control & ACCEL_SET)
		s->inv_sensor_control2 |= ACCEL_ACCURACY_SET;
	else
		s->inv_sensor_control2 &= ~ACCEL_ACCURACY_SET;

	if ((s->inv_sensor_control & GYRO_CALIBR_SET) || (s->inv_sensor_control & GYRO_SET))
		s->inv_sensor_control2 |= GYRO_ACCURACY_SET;
	else
		s->inv_sensor_control2 &= ~GYRO_ACCURACY_SET;

	if ((s->inv_sensor_control & CPASS_CALIBR_SET) || (s->inv_sensor_control & QUAT9_SET)
		|| (s->inv_sensor_control & GEOMAG_SET) || (s->inv_sensor_control & CPASS_SET))
		s->inv_sensor_control2 |= CPASS_ACCURACY_SET;
	else
		s->inv_sensor_control2 &= ~CPASS_ACCURACY_SET;

	if(s->flip_pickup_status)
		s->inv_sensor_control2 |= FLIP_PICKUP_SET;
	else
		s->inv_sensor_control2 &= ~FLIP_PICKUP_SET;

	// inv_event_control   |= s->b2s_status; 
	if(s->b2s_status)
	{
		inv_event_control |= INV_BRING_AND_LOOK_T0_SEE_EN;
		inv_event_control |= INV_PEDOMETER_EN;
#ifndef ICM20948_FOR_MOBILE // Next lines change BAC behavior to wearable platform
		inv_event_control |= INV_BAC_WEARABLE_EN;
		dmp_icm20948_set_ped_y_ratio(s, BAC_PED_Y_RATIO_WEARABLE);
#endif
	}
	else
	{
		inv_event_control &= ~INV_BRING_AND_LOOK_T0_SEE_EN;
		inv_event_control &= ~INV_PEDOMETER_EN;
#ifndef ICM20948_FOR_MOBILE // Next lines change BAC behavior to wearable platform
		inv_event_control &= ~INV_BAC_WEARABLE_EN;
#endif
	}

	result |= dmp_icm20948_set_data_output_control2(s, s->inv_sensor_control2);

	// sets DATA_RDY_STATUS in DMP based on which sensors are on
	if (s->inv_androidSensorsOn_mask[0] & INV_NEEDS_GYRO_MASK || s->inv_androidSensorsOn_mask[1] & INV_NEEDS_GYRO_MASK1)
		data_rdy_status |= GYRO_AVAILABLE;
	
	if (s->inv_androidSensorsOn_mask[0] & INV_NEEDS_ACCEL_MASK || s->inv_androidSensorsOn_mask[1] & INV_NEEDS_ACCEL_MASK1)
		data_rdy_status |= ACCEL_AVAILABLE;

	if (s->flip_pickup_status || s->b2s_status)
		data_rdy_status |= ACCEL_AVAILABLE;

	if (s->bac_status)
		data_rdy_status |= ACCEL_AVAILABLE;

	if (s->inv_androidSensorsOn_mask[0] & INV_NEEDS_COMPASS_MASK || s->inv_androidSensorsOn_mask[1] & INV_NEEDS_COMPASS_MASK1) {
		data_rdy_status |= SECONDARY_COMPASS_AVAILABLE;
		inv_event_control |= INV_COMPASS_CAL_EN;
	}
	// turn on gyro cal only if gyro is available
	if (data_rdy_status & GYRO_AVAILABLE)
		inv_event_control |= INV_GYRO_CAL_EN;
		
	// turn on acc cal only if acc is available
	if (data_rdy_status & ACCEL_AVAILABLE)
		inv_event_control |= INV_ACCEL_CAL_EN;

	inv_event_control |= s->smd_status | s->ped_int_status;

	if (s->inv_sensor_control & QUAT9_SET)
		inv_event_control |= INV_NINE_AXIS_EN;

	if (s->inv_sensor_control & (PED_STEPDET_SET | PED_STEPIND_SET) || inv_event_control & INV_SMD_EN) {
		inv_event_control |= INV_PEDOMETER_EN;
#ifndef ICM20948_FOR_MOBILE // Next lines change BAC behavior to wearable platform
		inv_event_control |= INV_BAC_WEARABLE_EN;
		dmp_icm20948_set_ped_y_ratio(s, BAC_PED_Y_RATIO_WEARABLE);
#endif
	}

	if (s->inv_sensor_control2 & ACT_RECOG_SET) {
		inv_event_control |= INV_PEDOMETER_EN;
#ifndef ICM20948_FOR_MOBILE // Next lines this to change BAC behavior to wearable platform
		inv_event_control |= INV_BAC_WEARABLE_EN;
		dmp_icm20948_set_ped_y_ratio(s, BAC_PED_Y_RATIO_WEARABLE);
#endif
	}

	if (s->inv_sensor_control2 & FLIP_PICKUP_SET){
		inv_event_control |= FLIP_PICKUP_EN;
	}

	if (s->inv_sensor_control & GEOMAG_SET)
		inv_event_control |= GEOMAG_EN;

	result |= dmp_icm20948_set_motion_event_control(s, inv_event_control);
	
	// A sensor was just enabled/disabled, need to recompute the required ODR for all augmented sensor-related sensors
	// The fastest ODR will always be applied to other related sensors
	if (   (androidSensor == ANDROID_SENSOR_GRAVITY) 
		|| (androidSensor == ANDROID_SENSOR_GAME_ROTATION_VECTOR) 
		|| (androidSensor == ANDROID_SENSOR_LINEAR_ACCELERATION) ) {
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_SIXQ]);
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_SIXQ_accel]);
	}

	if (   (androidSensor == ANDROID_SENSOR_ORIENTATION) 
		|| (androidSensor == ANDROID_SENSOR_ROTATION_VECTOR) ) {
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_NINEQ]);
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_NINEQ_accel]);
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_NINEQ_cpass]);
	}

	if (   (androidSensor == ANDROID_SENSOR_WAKEUP_GRAVITY) 
		|| (androidSensor == ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR) 
		|| (androidSensor == ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION) ) {
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ]);
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_SIXQ_accel]);
	}

	if (   (androidSensor == ANDROID_SENSOR_WAKEUP_ORIENTATION) 
		|| (androidSensor == ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR) ) {
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ]);
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_accel]);
		inv_icm20948_augmented_sensors_update_odr(s, androidSensor, &s->inv_dmp_odr_delays[INV_SENSOR_WAKEUP_NINEQ_cpass]);
	}

	result |= inv_set_hw_smplrt_dmp_odrs(s);
	result |= inv_icm20948_set_gyro_sf(s, inv_icm20948_get_gyro_divider(s), inv_icm20948_get_gyro_fullscale(s));

	if (!s->inv_sensor_control && !(s->inv_androidSensorsOn_mask[0] & (1L << ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION)) && !s->b2s_status) {
		*mems_put_to_sleep =1 ;
		result |= inv_icm20948_sleep_mems(s);
	}

	// DMP no longer controls PWR_MGMT_2 because of hardware bug, 0x80 set to override default behaviour of inv_icm20948_enable_hw_sensors()
	result |= inv_icm20948_enable_hw_sensors(s, (int)data_rdy_status | 0x80);

	// set DATA_RDY_STATUS in DMP
	if (data_rdy_status & SECONDARY_COMPASS_AVAILABLE)	{
		data_rdy_status |= SECONDARY_COMPASS_AVAILABLE;
	}

	result |= dmp_icm20948_set_data_rdy_status(s, data_rdy_status);

	// To have the all steps when you enable the sensor
	if (androidSensor == ANDROID_SENSOR_STEP_COUNTER)
	{
		if (enable)
		{
			dmp_icm20948_get_pedometer_num_of_steps(s, &steps);
			s->sStepCounterToBeSubtracted = steps - s->sOldSteps;
		}
	}

	return result;
}

void inv_icm20948_ctrl_enable_activity_classifier(struct inv_icm20948 * s, unsigned char enable) 
{
	s->bac_on = enable;
	if (enable) {
		s->bac_status = ACT_RECOG_SET;
		s->inv_sensor_control2 |= ACT_RECOG_SET;
		s->bac_request ++;
	}
	else {
		// only disable tilt engine if no request for tilt sensor
		if (!inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_TILT_DETECTOR)) {
			s->bac_status = 0;
			s->inv_sensor_control2 &= ~ACT_RECOG_SET;
			s->bac_request --;
		}
	}
}

void inv_icm20948_ctrl_enable_tilt(struct inv_icm20948 * s, unsigned char enable) 
{
	if (enable) {
		s->bac_status = ACT_RECOG_SET;
		s->inv_sensor_control2 |= ACT_RECOG_SET;
		s->bac_request ++;
	}
	else {
		// do not disable BAC engine if BAC sensor is still on even though tilt is off
		if (!s->bac_on) {
			s->bac_status = 0;
			s->inv_sensor_control2 &= ~ACT_RECOG_SET;
			s->bac_request --;
		}
	}
}

int inv_icm20948_ctrl_enable_batch(struct inv_icm20948 * s, unsigned char enable)
{
	int ret = 0;

	if(enable)
		s->inv_sensor_control2 |= BATCH_MODE_EN;
	else
		s->inv_sensor_control2 &= ~BATCH_MODE_EN;

	ret = dmp_icm20948_set_data_output_control2(s, s->inv_sensor_control2);

	/* give batch mode status to mems transport layer 
	to allow disable/enable LP_EN when reading FIFO in batch mode */
	inv_icm20948_ctrl_set_batch_mode_status(s, enable);

	return ret;
}

void inv_icm20948_ctrl_set_batch_mode_status(struct inv_icm20948 * s, unsigned char enable)
{
	if(enable)
		s->sBatchMode=1;
	else
		s->sBatchMode=0;
}

unsigned char inv_icm20948_ctrl_get_batch_mode_status(struct inv_icm20948 * s)
{
	return s->sBatchMode;
}

int inv_icm20948_ctrl_set_batch_timeout(struct inv_icm20948 * s, unsigned short batch_time_in_seconds)
{
	unsigned int timeout = 0;

	if(    s->inv_sensor_control & GYRO_CALIBR_SET 
		|| s->inv_sensor_control & QUAT6_SET 
		|| s->inv_sensor_control & QUAT9_SET 
		|| s->inv_sensor_control & GYRO_SET ) { // If Gyro based sensor is enabled.
		timeout = (unsigned int) (batch_time_in_seconds * (BASE_SAMPLE_RATE/ (inv_icm20948_get_gyro_divider(s) + 1)));
		return dmp_icm20948_set_batchmode_params(s, timeout, GYRO_AVAILABLE);
	}

	if(    s->inv_sensor_control & ACCEL_SET 
		|| s->inv_sensor_control & GEOMAG_SET ) { // If Accel is enabled and no Gyro based sensor is enabled.
		timeout = (unsigned int) (batch_time_in_seconds * (BASE_SAMPLE_RATE/ (inv_icm20948_get_accel_divider(s) + 1)));
		return dmp_icm20948_set_batchmode_params(s, timeout, ACCEL_AVAILABLE);
	}

	if(    s->inv_sensor_control & CPASS_SET 
		|| s->inv_sensor_control & CPASS_CALIBR_SET ) {
		int rc = 0;

		timeout = (unsigned int) (batch_time_in_seconds * (BASE_SAMPLE_RATE/ inv_icm20948_get_secondary_divider(s)));
	
		if(    s->inv_sensor_control & CPASS_SET 
			|| s->inv_sensor_control & CPASS_CALIBR_SET ) {
			rc |= dmp_icm20948_set_batchmode_params(s, timeout, SECONDARY_COMPASS_AVAILABLE);
		}
	
		return rc;
	}

	return -1;  // Call batch only when a sensor is enabled.
}    

int inv_icm20948_ctrl_set_batch_timeout_ms(struct inv_icm20948 * s, unsigned short batch_time_in_ms)
{
	unsigned int timeout = 0;

	if(    s->inv_sensor_control & GYRO_CALIBR_SET 
		|| s->inv_sensor_control & QUAT6_SET 
		|| s->inv_sensor_control & QUAT9_SET 
		|| s->inv_sensor_control & GYRO_SET ) { // If Gyro based sensor is enabled.
		timeout = (unsigned int) ((batch_time_in_ms * (BASE_SAMPLE_RATE/ (inv_icm20948_get_gyro_divider(s) + 1)))/1000);
		if(batch_time_in_ms < s->inv_androidSensorsOdr_boundaries[ANDROID_SENSOR_GYROSCOPE][0]) {
			return -1; // requested batch timeout is not supported
		} else {
			return dmp_icm20948_set_batchmode_params(s, timeout, GYRO_AVAILABLE);
		}
	}

	if(    s->inv_sensor_control & ACCEL_SET
		|| s->inv_sensor_control & GEOMAG_SET ) { // If Accel is enabled and no Gyro based sensor is enabled.
		timeout = (unsigned int) ((batch_time_in_ms * (BASE_SAMPLE_RATE/ (inv_icm20948_get_accel_divider(s) + 1)))/1000);
		if(batch_time_in_ms < s->inv_androidSensorsOdr_boundaries[ANDROID_SENSOR_ACCELEROMETER][0]) {
			return -1; // requested batch timeout is not supported
		} else {
			return dmp_icm20948_set_batchmode_params(s, timeout, ACCEL_AVAILABLE);
		}
	}

	if(    s->inv_sensor_control & CPASS_SET 
		|| s->inv_sensor_control & CPASS_CALIBR_SET ) {
		timeout = (unsigned int) ((batch_time_in_ms * (BASE_SAMPLE_RATE/ inv_icm20948_get_secondary_divider(s)))/1000);
		if(batch_time_in_ms < s->inv_androidSensorsOdr_boundaries[ANDROID_SENSOR_GEOMAGNETIC_FIELD][0]) {
			return -1; // requested batch timeout is not supported
		} else {
			return dmp_icm20948_set_batchmode_params(s, timeout, SECONDARY_COMPASS_AVAILABLE);
		}
	}

	return -1; // Call batch only when a sensor is enabled.
}

/** Each bit corresponds to a sensor being on (Sensors 0 to 21)
*/
unsigned long *inv_icm20948_ctrl_get_androidSensorsOn_mask(struct inv_icm20948 * s)
{
	return s->inv_androidSensorsOn_mask;
}

unsigned short inv_icm20948_ctrl_get_activitiy_classifier_on_flag(struct inv_icm20948 * s)
{
	return s->bac_on;
}

/** @brief Sets accel quaternion gain according to accel engine rate.
* @param[in] hw_smplrt_divider  hardware sample rate divider such that accel engine rate = 1125Hz/hw_smplrt_divider
* @return 0 in case of success, -1 for any error
*/
int inv_icm20948_ctrl_set_accel_quaternion_gain(struct inv_icm20948 * s, unsigned short hw_smplrt_divider)
{
	int accel_gain = 15252014L; //set 225Hz gain as default

	switch (hw_smplrt_divider) {
		case 5: //1125Hz/5 = 225Hz
			accel_gain = 15252014L;
			break;
		case 10: //1125Hz/10 = 112Hz
			accel_gain = 30504029L;
			break;
		case 11: //1125Hz/11 = 102Hz
			accel_gain = 33554432L;
			break;
		case 22: //1125Hz/22 = 51Hz
			accel_gain = 67108864L;
			break;
		default:
			accel_gain = 15252014L;
			break;
	}

	return dmp_icm20948_set_accel_feedback_gain(s, accel_gain);
}

int inv_icm20948_ctrl_set_accel_cal_params(struct inv_icm20948 * s, unsigned short hw_smplrt_divider)
{
	int accel_cal_params[NUM_ACCEL_CAL_PARAMS] = {0};

	if (hw_smplrt_divider <= 5) { // freq = 225Hz
		accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 1026019965L;
		accel_cal_params[ACCEL_CAL_A_VAR] = 47721859L;
	} 
	else if (hw_smplrt_divider <= 10) { // 225Hz > freq >= 112Hz
		accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 977872018L;
		accel_cal_params[ACCEL_CAL_A_VAR] = 95869806L;
	} 
	else if (hw_smplrt_divider <= 11) { // 112Hz > freq >= 102Hz
		accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 858993459L;
		accel_cal_params[ACCEL_CAL_A_VAR] = 214748365L;
		accel_cal_params[ACCEL_CAL_DIV] = 1;
	} 
	else if (hw_smplrt_divider <= 20) { // 102Hz > freq >= 56Hz
		accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 882002213L;
		accel_cal_params[ACCEL_CAL_A_VAR] = 191739611L;
	} 
	else if (hw_smplrt_divider <= 22) { // 56Hz > freq >= 51Hz
		accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 858993459L;
		accel_cal_params[ACCEL_CAL_A_VAR] = 214748365L;
	} 
	else if (hw_smplrt_divider <= 75) { // 51Hz > freq >= 15Hz
		accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 357913941L;
		accel_cal_params[ACCEL_CAL_A_VAR] = 715827883L;
	} 
	else if (hw_smplrt_divider <= 225) { // 15Hz > freq >= 5Hz
		accel_cal_params[ACCEL_CAL_ALPHA_VAR] = 107374182L;
		accel_cal_params[ACCEL_CAL_A_VAR] = 966367642L;
	}

	return dmp_icm20948_set_accel_cal_params(s, accel_cal_params);
}

/* 5061:  this should be used to disable PICKUp after it triggers once
 * DO WE NEED TO CLEAR A BIT IN EVENT CONTROL?
 */
int inv_icm20948_ctrl_enable_pickup(struct inv_icm20948 * s, unsigned char enable)
{
	s->pickup = enable;
	if(enable)
		s->inv_sensor_control2 |= FLIP_PICKUP_EN;
	else
		s->inv_sensor_control2 &= ~FLIP_PICKUP_EN;

	return dmp_icm20948_set_data_output_control2(s, s->inv_sensor_control2);
}

int inv_icm20948_ctrl_get_acc_bias(struct inv_icm20948 * s, int * acc_bias)
{
	return dmp_icm20948_get_bias_acc(s, acc_bias);
}

int inv_icm20948_ctrl_get_gyr_bias(struct inv_icm20948 * s, int * gyr_bias)
{
	return dmp_icm20948_get_bias_gyr(s, gyr_bias);
}

int inv_icm20948_ctrl_get_mag_bias(struct inv_icm20948 * s, int * mag_bias)
{
	return dmp_icm20948_get_bias_cmp(s, mag_bias);
}

int inv_icm20948_ctrl_set_acc_bias(struct inv_icm20948 * s, int * acc_bias)
{
	int rc = 0;
	
	s->bias[0] = acc_bias[0];
	s->bias[1] = acc_bias[1];
	s->bias[2] = acc_bias[2];
	
	rc = dmp_icm20948_set_bias_acc(s, &s->bias[0]);
	
	return rc;
}

int inv_icm20948_ctrl_set_gyr_bias(struct inv_icm20948 * s, int * gyr_bias)
{
	int rc = 0;
	
	s->bias[3] = gyr_bias[0];
	s->bias[4] = gyr_bias[1];
	s->bias[5] = gyr_bias[2];
	
	rc = dmp_icm20948_set_bias_gyr(s, &s->bias[3]);
	
	return rc;
}

int inv_icm20948_ctrl_set_mag_bias(struct inv_icm20948 * s, int * mag_bias)
{
	int rc = 0;
	
	s->bias[6] = mag_bias[0];
	s->bias[7] = mag_bias[1];
	s->bias[8] = mag_bias[2];
	
	rc = dmp_icm20948_set_bias_cmp(s, &s->bias[6]);
	
	return rc;
}
static unsigned char sensor_needs_compass(unsigned char androidSensor)
{
	switch(androidSensor) {
		case ANDROID_SENSOR_GEOMAGNETIC_FIELD:
		case ANDROID_SENSOR_ROTATION_VECTOR:
		case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD:
		case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
		case ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED:
		case ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR:
			return 1;

		default :
			return 0;
	}
}

static unsigned char sensor_needs_bac_algo(unsigned char androidSensor)
{
	switch(androidSensor){
	case ANDROID_SENSOR_FLIP_PICKUP:
	case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON:
	case ANDROID_SENSOR_STEP_DETECTOR:
	case ANDROID_SENSOR_STEP_COUNTER:
	case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR:
	case ANDROID_SENSOR_WAKEUP_STEP_DETECTOR:
	case ANDROID_SENSOR_WAKEUP_STEP_COUNTER:
	case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
	case ANDROID_SENSOR_B2S:
		return 1;
	default:
		return 0;
	}
}
