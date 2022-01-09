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
#include "Icm20948DataConverter.h"

#include <string.h>
#include <math.h>

static void invn_convert_quat_invert_fxp(const long *quat_q30, long *invQuat_q30);

static void invn_convert_quat_mult_fxp(const long *quat1_q30, const long *quat2_q30, long *quatProd_q30)
{
    quatProd_q30[0] = inv_icm20948_convert_mult_q30_fxp(quat1_q30[0], quat2_q30[0]) - inv_icm20948_convert_mult_q30_fxp(quat1_q30[1], quat2_q30[1]) -
               inv_icm20948_convert_mult_q30_fxp(quat1_q30[2], quat2_q30[2]) - inv_icm20948_convert_mult_q30_fxp(quat1_q30[3], quat2_q30[3]);

    quatProd_q30[1] = inv_icm20948_convert_mult_q30_fxp(quat1_q30[0], quat2_q30[1]) + inv_icm20948_convert_mult_q30_fxp(quat1_q30[1], quat2_q30[0]) +
               inv_icm20948_convert_mult_q30_fxp(quat1_q30[2], quat2_q30[3]) - inv_icm20948_convert_mult_q30_fxp(quat1_q30[3], quat2_q30[2]);

    quatProd_q30[2] = inv_icm20948_convert_mult_q30_fxp(quat1_q30[0], quat2_q30[2]) - inv_icm20948_convert_mult_q30_fxp(quat1_q30[1], quat2_q30[3]) +
               inv_icm20948_convert_mult_q30_fxp(quat1_q30[2], quat2_q30[0]) + inv_icm20948_convert_mult_q30_fxp(quat1_q30[3], quat2_q30[1]);

    quatProd_q30[3] = inv_icm20948_convert_mult_q30_fxp(quat1_q30[0], quat2_q30[3]) + inv_icm20948_convert_mult_q30_fxp(quat1_q30[1], quat2_q30[2]) -
               inv_icm20948_convert_mult_q30_fxp(quat1_q30[2], quat2_q30[1]) + inv_icm20948_convert_mult_q30_fxp(quat1_q30[3], quat2_q30[0]);
}

static void invn_convert_quat_invert_fxp(const long *quat_q30, long *invQuat_q30)
{
    invQuat_q30[0] = quat_q30[0];
    invQuat_q30[1] = -quat_q30[1];
    invQuat_q30[2] = -quat_q30[2];
    invQuat_q30[3] = -quat_q30[3];
}

void inv_icm20948_q_mult_q_qi(const long *q1, const long *q2, long *qProd)
{
    qProd[0] = inv_icm20948_convert_mult_q30_fxp(q1[0], q2[0]) + inv_icm20948_convert_mult_q30_fxp(q1[1], q2[1]) +
               inv_icm20948_convert_mult_q30_fxp(q1[2], q2[2]) + inv_icm20948_convert_mult_q30_fxp(q1[3], q2[3]);

    qProd[1] = -inv_icm20948_convert_mult_q30_fxp(q1[0], q2[1]) + inv_icm20948_convert_mult_q30_fxp(q1[1], q2[0]) -
               inv_icm20948_convert_mult_q30_fxp(q1[2], q2[3]) + inv_icm20948_convert_mult_q30_fxp(q1[3], q2[2]);

    qProd[2] = -inv_icm20948_convert_mult_q30_fxp(q1[0], q2[2]) + inv_icm20948_convert_mult_q30_fxp(q1[1], q2[3]) +
               inv_icm20948_convert_mult_q30_fxp(q1[2], q2[0]) - inv_icm20948_convert_mult_q30_fxp(q1[3], q2[1]);

    qProd[3] = -inv_icm20948_convert_mult_q30_fxp(q1[0], q2[3]) - inv_icm20948_convert_mult_q30_fxp(q1[1], q2[2]) +
               inv_icm20948_convert_mult_q30_fxp(q1[2], q2[1]) + inv_icm20948_convert_mult_q30_fxp(q1[3], q2[0]);
}

void inv_icm20948_convert_quat_rotate_fxp(const long *quat_q30, const long *in, long *out)
{
    long q_temp1[4], q_temp2[4];
    long in4[4], out4[4];

    // Fixme optimize
    in4[0] = 0;
    memcpy(&in4[1], in, 3 * sizeof(long));
    invn_convert_quat_mult_fxp(quat_q30, in4, q_temp1);
    invn_convert_quat_invert_fxp(quat_q30, q_temp2);
    invn_convert_quat_mult_fxp(q_temp1, q_temp2, out4);
    memcpy(out, &out4[1], 3 * sizeof(long));
}

/** Set the transformation used for chip to body frame
*/
void inv_icm20948_set_chip_to_body(struct inv_icm20948 * s, long *quat)
{
    memcpy(s->s_quat_chip_to_body, quat, sizeof(s->s_quat_chip_to_body));
}

/** Convert fixed point DMP rotation vector to floating point android notation
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] values 4 element quaternion in Android format
*/
void inv_icm20948_convert_rotation_vector(struct inv_icm20948 * s, const long *quat, float *values)
{
    long quat4[4];
    long quat_body_to_world[4];

    inv_icm20948_convert_compute_scalar_part_fxp(quat, quat4);
    inv_icm20948_q_mult_q_qi(quat4, s->s_quat_chip_to_body, quat_body_to_world);
    if (quat_body_to_world[0] >= 0) {
        values[0] = quat_body_to_world[1] * INV_TWO_POWER_NEG_30;
        values[1] = quat_body_to_world[2] * INV_TWO_POWER_NEG_30;
        values[2] = quat_body_to_world[3] * INV_TWO_POWER_NEG_30;
        values[3] = quat_body_to_world[0] * INV_TWO_POWER_NEG_30;
    } else {
        values[0] = -quat_body_to_world[1] * INV_TWO_POWER_NEG_30;
        values[1] = -quat_body_to_world[2] * INV_TWO_POWER_NEG_30;
        values[2] = -quat_body_to_world[3] * INV_TWO_POWER_NEG_30;
        values[3] = -quat_body_to_world[0] * INV_TWO_POWER_NEG_30;
    }
}

/** Convert 3 element fixed point DMP rotation vector to 4 element rotation vector in world frame
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] values 4 element quaternion
*/
void inv_icm20948_convert_rotation_vector_2(struct inv_icm20948 * s, const long *quat, long *quat4_world)
{
    long quat4[4];
    long quat_body_to_world[4];

    inv_icm20948_convert_compute_scalar_part_fxp(quat, quat4);
    inv_icm20948_q_mult_q_qi(quat4, s->s_quat_chip_to_body, quat_body_to_world);
    memcpy(quat4_world, quat_body_to_world, 4*sizeof(long));
}

/** Convert 4 element rotation vector in world frame to floating point android notation
* @param[in] quat 4 element rotation vector in World frame
* @param[out] values in Android format
*/
void inv_icm20948_convert_rotation_vector_3(const long *quat4_world, float *values)
{
    if (quat4_world[0] >= 0) {
        values[0] = quat4_world[1] * INV_TWO_POWER_NEG_30;
        values[1] = quat4_world[2] * INV_TWO_POWER_NEG_30;
        values[2] = quat4_world[3] * INV_TWO_POWER_NEG_30;
        values[3] = quat4_world[0] * INV_TWO_POWER_NEG_30;
    } else {
        values[0] = -quat4_world[1] * INV_TWO_POWER_NEG_30;
        values[1] = -quat4_world[2] * INV_TWO_POWER_NEG_30;
        values[2] = -quat4_world[3] * INV_TWO_POWER_NEG_30;
        values[3] = -quat4_world[0] * INV_TWO_POWER_NEG_30;
    }
}

static void inv_rotation_to_quaternion(float Rcb[9], long Qcb_fp[4]) {	
	float q[4]; 
	inv_icm20948_convert_matrix_to_quat_flt(Rcb, q); 
	INVN_CONVERT_FLT_TO_FXP(q, Qcb_fp, 4, 30); 
}

void inv_icm20948_set_chip_to_body_axis_quaternion(struct inv_icm20948 * s, signed char *accel_gyro_matrix, float angle)
{
    int i;
    float rot[9];
    long qcb[4],q_all[4];
    long q_adjust[4];
    for (i=0; i<9; i++) {
        rot[i] = (float)accel_gyro_matrix[i];
    }
    // Convert Chip to Body transformation matrix to quaternion
    // inv_icm20948_convert_matrix_to_quat_fxp(rot, qcb);
	inv_rotation_to_quaternion(rot, qcb);
	
    // The quaterion generated is the inverse, take the inverse again.
    qcb[1] = -qcb[1];
    qcb[2] = -qcb[2];
    qcb[3] = -qcb[3];

    // Now rotate by angle, negate angle to rotate other way
    q_adjust[0] = (long)((1L<<30) * cosf(-angle*(float)M_PI/180.f/2.f));
    q_adjust[1] = 0;
    q_adjust[2] = (long)((1L<<30)*sinf(-angle*(float)M_PI/180.f/2.f));
    q_adjust[3] = 0;
    invn_convert_quat_mult_fxp(q_adjust, qcb, q_all);
    inv_icm20948_set_chip_to_body(s, q_all);
}

void inv_icm20948_convert_dmp3_to_body(struct inv_icm20948 * s, const long *vec3, float scale, float *values)
{
    long out[3];
    inv_icm20948_convert_quat_rotate_fxp(s->s_quat_chip_to_body, vec3, out);
    values[0] = out[0] * scale;
    values[1] = out[1] * scale;
    values[2] = out[2] * scale;
}

/** Converts a 32-bit long to a little endian byte stream */
unsigned char *inv_icm20948_int32_to_little8(long x, unsigned char *little8)
{
    little8[3] = (unsigned char)((x >> 24) & 0xff);
    little8[2] = (unsigned char)((x >> 16) & 0xff);
    little8[1] = (unsigned char)((x >> 8) & 0xff);
    little8[0] = (unsigned char)(x & 0xff);
    return little8;
}

float inv_icm20948_convert_deg_to_rad(float deg_val)
{
	float rad_val;
    rad_val = deg_val*(float)M_PI / 180.f;
	return rad_val;
}

long inv_icm20948_convert_mult_q30_fxp(long a_q30, long b_q30)
{
	long long temp;
	long result;
	temp = (long long)a_q30 * b_q30;
	result = (long)(temp >> 30);
	return result;
}

int inv_icm20948_convert_compute_scalar_part_fxp(const long * inQuat_q30, long* outQuat_q30)
{
    long scalarPart = 0;

    scalarPart = inv_icm20948_convert_fast_sqrt_fxp((1L<<30) - inv_icm20948_convert_mult_q30_fxp(inQuat_q30[0], inQuat_q30[0])
                                        - inv_icm20948_convert_mult_q30_fxp(inQuat_q30[1], inQuat_q30[1])
                                        - inv_icm20948_convert_mult_q30_fxp(inQuat_q30[2], inQuat_q30[2]) );
    outQuat_q30[0] = scalarPart;
    outQuat_q30[1] = inQuat_q30[0];
    outQuat_q30[2] = inQuat_q30[1];
    outQuat_q30[3] = inQuat_q30[2];

    return 0;
}

long inv_icm20948_convert_fast_sqrt_fxp(long x0_q30)
{

	//% Square-Root with NR in the neighborhood of 1.3>x>=0.65 (log(2) <= x <= log(4) )
    // Two-variable NR iteration:
    // Initialize: a=x; c=x-1;  
    // 1st Newton Step:  a=a-a*c/2; ( or: a = x - x*(x-1)/2  )
    // Iterate: c = c*c*(c-3)/4
    //          a = a - a*c/2    --> reevaluating c at this step gives error of approximation

	//% Seed equals 1. Works best in this region.
	//xx0 = int32(1*2^30);

	long sqrt2, oneoversqrt2, one_pt5;
	long xx, cc;
	int pow2, sq2scale, nr_iters;

	// Return if input is zero. Negative should really error out. 
	if (x0_q30 <= 0L) {
		return 0L;
	}

	sqrt2 =1518500250L;
	oneoversqrt2=759250125L;
	one_pt5=1610612736L;

	nr_iters = inv_icm20948_convert_test_limits_and_scale_fxp(&x0_q30, &pow2);
	
	sq2scale = 0;
	if (pow2 > 0) 
		sq2scale=pow2%2;  // Find remainder. Is it even or odd?
	pow2 = pow2-sq2scale; // Now pow2 is even. Note we are adding because result is scaled with sqrt(2)

	// Sqrt 1st NR iteration
	cc = x0_q30 - (1L<<30);
	xx = x0_q30 - (inv_icm20948_convert_mult_q30_fxp(x0_q30, cc)>>1);
 	if ( nr_iters>=2 ) {
		// Sqrt second NR iteration
		// cc = cc*cc*(cc-3)/4; = cc*cc*(cc/2 - 3/2)/2;
		// cc = ( cc*cc*((cc>>1) - onePt5) ) >> 1
		cc = inv_icm20948_convert_mult_q30_fxp( cc, inv_icm20948_convert_mult_q30_fxp(cc, (cc>>1) - one_pt5) ) >> 1;
		xx = xx - (inv_icm20948_convert_mult_q30_fxp(xx, cc)>>1);
		if ( nr_iters==3 ) {
			// Sqrt third NR iteration
			cc = inv_icm20948_convert_mult_q30_fxp( cc, inv_icm20948_convert_mult_q30_fxp(cc, (cc>>1) - one_pt5) ) >> 1;
			xx = xx - (inv_icm20948_convert_mult_q30_fxp(xx, cc)>>1);
		}
	}
	if (sq2scale)
		xx = inv_icm20948_convert_mult_q30_fxp(xx,oneoversqrt2);
	// Scale the number with the half of the power of 2 scaling
	if (pow2>0)
		xx = (xx >> (pow2>>1)); 
	else if (pow2 == -1)
		xx = inv_icm20948_convert_mult_q30_fxp(xx,sqrt2);
	return xx;
}

int inv_icm20948_convert_test_limits_and_scale_fxp(long *x0_q30, int *pow)
{
    long lowerlimit, upperlimit, oneiterlothr, oneiterhithr, zeroiterlothr, zeroiterhithr;

    // Lower Limit: ll = int32(log(2)*2^30);
    lowerlimit = 744261118L;
    //Upper Limit ul = int32(log(4)*2^30);
    upperlimit = 1488522236L;
    //  int32(0.9*2^30)
    oneiterlothr = 966367642L;
    // int32(1.1*2^30)
    oneiterhithr = 1181116006L;
    // int32(0.99*2^30)
    zeroiterlothr=1063004406L;
    //int32(1.01*2^30)
    zeroiterhithr=1084479242L;

    // Scale number such that Newton Raphson iteration works best:
    // Find the power of two scaling that leaves the number in the optimal range,
    // ll <= number <= ul. Note odd powers have special scaling further below
	if (*x0_q30 > upperlimit) {
		// Halving the number will push it in the optimal range since largest value is 2
		*x0_q30 = *x0_q30>>1;
		*pow=-1;
	} else if (*x0_q30 < lowerlimit) {
		// Find position of highest bit, counting from left, and scale number 
		*pow=inv_icm20948_convert_get_highest_bit_position((uint32_t*)x0_q30);
		if (*x0_q30 >= upperlimit) {
			// Halving the number will push it in the optimal range
			*x0_q30 = *x0_q30>>1;
			*pow=*pow-1;
		}
		else if (*x0_q30 < lowerlimit) {
			// Doubling the number will push it in the optimal range
			*x0_q30 = *x0_q30<<1;
			*pow=*pow+1;
		}
	} else {
		*pow = 0;
	}
    
    if ( *x0_q30<oneiterlothr || *x0_q30>oneiterhithr )
        return 3; // 3 NR iterations
    if ( *x0_q30<zeroiterlothr || *x0_q30>zeroiterhithr )
        return 2; // 2 NR iteration

    return 1; // 1 NR iteration
}

/** Auxiliary function used by testLimitsAndScale()
* Find the highest nonzero bit in an unsigned 32 bit integer:
* @param[in] value operand Dimension is 1.
* @return highest bit position.
* \note This function performs the log2 of an interger as well. 
* \ingroup binary
**/
int16_t inv_icm20948_convert_get_highest_bit_position(uint32_t *value)
{
    int16_t position;
    position = 0;
    if (*value == 0) return 0;

    if ((*value & 0xFFFF0000) == 0) {
        position += 16;
        *value=*value<<16;
    }
    if ((*value & 0xFF000000) == 0) {
        position += 8;
        *value=*value<<8;
    }
    if ((*value & 0xF0000000) == 0) {
        position += 4;
        *value=*value<<4;
    }
    if ((*value & 0xC0000000) == 0) {
        position += 2;
        *value=*value<<2;
    }

    // If we got too far into sign bit, shift back. Note we are using an
    // unsigned long here, so right shift is going to shift all the bits.
    if ((*value & 0x80000000)) { 
        position -= 1;
        *value=*value>>1;
    }
    return position;
}

void inv_icm20948_convert_matrix_to_quat_fxp(long *Rcb_q30, long *Qcb_q30)
{
         long r11,r12,r13, r21,r22,r23, r31,r32,r33;
         long temp[3];
         long tmp;
         int pow2, shift;

         r11 = Rcb_q30[0]>>1; //assume matrix is stored row wise first, that is rot[1] is row 1, col 2
         r12 = Rcb_q30[1]>>1;
         r13 = Rcb_q30[2]>>1;

         r21 = Rcb_q30[3]>>1;
         r22 = Rcb_q30[4]>>1;
         r23 = Rcb_q30[5]>>1;

         r31 = Rcb_q30[6]>>1;
         r32 = Rcb_q30[7]>>1;
         r33 = Rcb_q30[8]>>1;

         //Qcb[0] = (1.f + r11 + r22 + r33) / 4.f;
         //Qcb[1] = (1.f + r11 - r22 - r33) / 4.f;
         //Qcb[2] = (1.f - r11 + r22 - r33) / 4.f;
         //Qcb[3] = (1.f - r11 - r22 + r33) / 4.f;
         Qcb_q30[0] = (268435456L + (r11>>1) + (r22>>1) + (r33>>1)); // Effectively shifted by 2 bits, one above, one here
         Qcb_q30[1] = (268435456L + (r11>>1) - (r22>>1) - (r33>>1));
         Qcb_q30[2] = (268435456L - (r11>>1) + (r22>>1) - (r33>>1));
         Qcb_q30[3] = (268435456L - (r11>>1) - (r22>>1) + (r33>>1));

         if(Qcb_q30[0] < 0L) Qcb_q30[0] = 0L;
         if(Qcb_q30[1] < 0L) Qcb_q30[1] = 0L;
         if(Qcb_q30[2] < 0L) Qcb_q30[2] = 0L;
         if(Qcb_q30[3] < 0L) Qcb_q30[3] = 0L;
         if (Qcb_q30[0] == 0L && Qcb_q30[1] == 0L && Qcb_q30[2] == 0L && Qcb_q30[3] == 0L) {
             Qcb_q30[0] = 1L<<30;
             return;
         }
         //Qcb[0] = sqrt(Qcb[0]);
         //Qcb[1] = sqrt(Qcb[1]);
         //Qcb[2] = sqrt(Qcb[2]);
         //Qcb[3] = sqrt(Qcb[3]);
         Qcb_q30[0] = inv_icm20948_convert_sqrt_q30_fxp(Qcb_q30[0]);
         Qcb_q30[1] = inv_icm20948_convert_sqrt_q30_fxp(Qcb_q30[1]);
         Qcb_q30[2] = inv_icm20948_convert_sqrt_q30_fxp(Qcb_q30[2]);
         Qcb_q30[3] = inv_icm20948_convert_sqrt_q30_fxp(Qcb_q30[3]);

         if(Qcb_q30[0] >= Qcb_q30[1] && Qcb_q30[0] >= Qcb_q30[2] && Qcb_q30[0] >= Qcb_q30[3]) //Qcb[0] is max
         {
                tmp = inv_icm20948_convert_inverse_q30_fxp(Qcb_q30[0], &pow2);
                shift = 30 - pow2 + 1;
                Qcb_q30[1] = (long)(((long long)(r23 - r32) * tmp) >> shift) ;
                Qcb_q30[2] = (long)(((long long)(r31 - r13) * tmp) >> shift) ;
                Qcb_q30[3] = (long)(((long long)(r12 - r21) * tmp) >> shift) ;
                 //Qcb[1] = (r23 - r32)/(4.f*Qcb[0]);
                 //Qcb[2] = (r31 - r13)/(4.f*Qcb[0]);
                 //Qcb[3] = (r12 - r21)/(4.f*Qcb[0]);
         }
         else if(Qcb_q30[1] >= Qcb_q30[0] && Qcb_q30[1] >= Qcb_q30[2] && Qcb_q30[1] >= Qcb_q30[3]) //Qcb[1] is max
         {
                tmp = inv_icm20948_convert_inverse_q30_fxp(Qcb_q30[1], &pow2);
                shift = 30 - pow2 + 1;
		        Qcb_q30[0] = (long)(((long long)(r23 - r32) * tmp) >> shift) ;
		        Qcb_q30[2] = (long)(((long long)(r12 + r21) * tmp) >> shift) ;
		        Qcb_q30[3] = (long)(((long long)(r31 + r13) * tmp) >> shift) ;
                // Qcb[0] = (r23 - r32)/(4.f*Qcb[1]);
                // Qcb[1] = Qcb[1];
                // Qcb[2] = (r12 + r21)/(4.f*Qcb[1]);
                // Qcb[3] = (r31 + r13)/(4.f*Qcb[1]);
         }
         else if(Qcb_q30[2] >= Qcb_q30[0] && Qcb_q30[2] >= Qcb_q30[1] && Qcb_q30[2] >= Qcb_q30[3]) //Qcb[2] is max
         {
                tmp = inv_icm20948_convert_inverse_q30_fxp(Qcb_q30[2], &pow2);
                shift = 30 - pow2 + 1;
		        Qcb_q30[0] = (long)(((long long)(r31 - r13) * tmp) >> shift) ;
		        Qcb_q30[1] = (long)(((long long)(r12 + r21) * tmp) >> shift) ;
		        Qcb_q30[3] = (long)(((long long)(r23 + r32) * tmp) >> shift) ;
                 //Qcb[0] = (r31 - r13)/(4.f*Qcb[2]);
                 //Qcb[1] = (r12 + r21)/(4.f*Qcb[2]);
                 //Qcb[2] = Qcb[2];
                 //Qcb[3] = (r23 + r32)/(4.f*Qcb[2]);
         }
         else if(Qcb_q30[3] >= Qcb_q30[0] && Qcb_q30[3] >= Qcb_q30[1] && Qcb_q30[3] >= Qcb_q30[2]) //Qcb[3] is max
         {
                tmp = inv_icm20948_convert_inverse_q30_fxp(Qcb_q30[3], &pow2);
                shift = 30 - pow2 + 1;
		        Qcb_q30[0] = (long)(((long long)(r12 - r21) * tmp) >> shift) ;
		        Qcb_q30[1] = (long)(((long long)(r31 + r13) * tmp) >> shift) ;
		        Qcb_q30[2] = (long)(((long long)(r23 + r32) * tmp) >> shift) ;
                 //Qcb[0] = (r12 - r21)/(4.f*Qcb[3]);
                 //Qcb[1] = (r31 + r13)/(4.f*Qcb[3]);
                 //Qcb[2] = (r23 + r32)/(4.f*Qcb[3]);
                 //Qcb[3] = Qcb[3];
         }
         else
         {
                // printf('coding error\n'); //error
             Qcb_q30[0] = 1L<<30;
             Qcb_q30[1] = 0L;
             Qcb_q30[2] = 0L;
             Qcb_q30[3] = 0L;
             return;
         }

        // Normalize
        // compute inverse square root, using first order taylor series
        // Here temp aligns with row 8
        temp[1] = (long)(((long long)Qcb_q30[0] * Qcb_q30[0] +
                          (long long)Qcb_q30[1] * Qcb_q30[1] +
                          (long long)Qcb_q30[2] * Qcb_q30[2] +
                          (long long)Qcb_q30[3] * Qcb_q30[3]) >> 30);
        temp[2] = temp[1] >> 1; // Multiply by 2^29
        temp[0] = (1L<<30) + (1L<<29) - temp[2];

        // Normalize
        Qcb_q30[0] = inv_icm20948_convert_mult_q30_fxp(temp[0], Qcb_q30[0]);
        Qcb_q30[1] = inv_icm20948_convert_mult_q30_fxp(temp[0], Qcb_q30[1]);
        Qcb_q30[2] = inv_icm20948_convert_mult_q30_fxp(temp[0], Qcb_q30[2]);
        Qcb_q30[3] = inv_icm20948_convert_mult_q30_fxp(temp[0], Qcb_q30[3]);
}

long inv_icm20948_convert_sqrt_q30_fxp(long x_q30)
{
    long sqrtx;
    int pow2;

    if (x_q30 <= 0L) {
        sqrtx = 0L;
        return sqrtx;
    }
    sqrtx = inv_icm20948_convert_inv_sqrt_q30_fxp(x_q30, &pow2); // invsqrtx

    sqrtx = inv_icm20948_convert_mult_q30_fxp(x_q30, sqrtx);

power_up:
    if (pow2 > 0) {
        sqrtx = 2*sqrtx;
        pow2=pow2-1;
        goto power_up;
    }
power_down:
    if (pow2 < 0) {
        sqrtx = sqrtx/2;
        pow2=pow2+1;
        goto power_down;
    }

    return sqrtx;
}

long inv_icm20948_convert_inv_sqrt_q30_fxp(long x_q30, int *pow2)
{
    long oneoversqrt2 = 759250125L; // int32(2^30*1/sqrt(2))
    long oneandhalf = 1610612736L; // int32(1.5*2^30);
    long upperlimit = 1488522236; // int32(log(4)*2^30);
    long lowerlimit = 744261118; // int32(log(2)*2^30); 
    long xx, x0_2, invsqrtx;

    *pow2 = 0;
	if (x_q30 <= 0) {
        return 1L<<30;
	}

    xx = x_q30;
    if (xx > upperlimit) {
downscale:
        if (xx > upperlimit) {
            xx = xx/2;
            *pow2 = *pow2 - 1;
            goto downscale;
        }
    }

    if (xx < lowerlimit) {
upscale:
        if (xx < lowerlimit) {
            xx = xx*2;
            *pow2 = *pow2 + 1;
            goto upscale;
        }
    }

    // 3 NR iterations. In some cases second and/or third iteration may not be needed, however
    // for code simplicity always iterate three times. Fourth iteration is below bit precision.
    x0_2 = xx >>1;
    xx = oneandhalf - x0_2;
    xx = inv_icm20948_convert_mult_q30_fxp( xx, ( oneandhalf - inv_icm20948_convert_mult_q30_fxp(x0_2, inv_icm20948_convert_mult_q30_fxp(xx,xx) ) ) );
    xx = inv_icm20948_convert_mult_q30_fxp( xx, ( oneandhalf - inv_icm20948_convert_mult_q30_fxp(x0_2, inv_icm20948_convert_mult_q30_fxp(xx,xx) ) ) );

    if (*pow2 & 1) { // This checks if the number is even or odd.
        *pow2 = (*pow2>>1) + 1; // Account for sqrt(2) in denominator
        invsqrtx = (inv_icm20948_convert_mult_q30_fxp(xx,oneoversqrt2));
    }
    else {
        *pow2 = *pow2>>1;
        invsqrtx =  xx;
    }

    return invsqrtx;
}

long inv_icm20948_convert_inverse_q30_fxp(long x_q30, int *pow2)
{
    long y;
    int negx;

	if (x_q30 == 0) {
		y = 0L;
        *pow2 = 0;
		return y;
	}

    negx=0;
    if (x_q30 < 0 ) {
        if (x_q30 == INT32_MIN)
            x_q30 = INT32_MAX;
        else
            x_q30 = -x_q30;
        negx = 1;
    }

    y = inv_icm20948_convert_inv_sqrt_q30_fxp (x_q30, pow2); // sqrt(y)
    if (y > 1518500249L) // y > int32(sqrt(2) -1 : Largest number that won't overflow q30 multiplication of y*y
        y = INT32_MAX;
    else
        y = inv_icm20948_convert_mult_q30_fxp(y, y);
    *pow2 = *pow2*2;  // Must double exponent due to multiply 

    if (negx)
        y=-y;
    return y;
}

void inv_icm20948_convert_matrix_to_quat_flt(float *R, float *q)
{
	float r11,r12,r13, r21,r22,r23, r31,r32,r33;

	r11 = R[0]; //assume matrix is stored row wise first, that is rot[1] is row 1, col 2
	r12 = R[1];
	r13 = R[2];

	r21 = R[3];
	r22 = R[4];
	r23 = R[5];

	r31 = R[6];
	r32 = R[7];
	r33 = R[8];

	q[0] = (1.f + r11 + r22 + r33) / 4.f;
	q[1] = (1.f + r11 - r22 - r33) / 4.f;
	q[2] = (1.f - r11 + r22 - r33) / 4.f;
	q[3] = (1.f - r11 - r22 + r33) / 4.f;

	if(q[0] < 0.0f) q[0] = 0.0f;
	if(q[1] < 0.0f) q[1] = 0.0f;
	if(q[2] < 0.0f) q[2] = 0.0f;
	if(q[3] < 0.0f) q[3] = 0.0f;
	q[0] = sqrtf(q[0]);
	q[1] = sqrtf(q[1]);
	q[2] = sqrtf(q[2]);
	q[3] = sqrtf(q[3]);

	/* Above paragraph could be reduced in :
	q[0] =(q[0] < 0.0f) ? q[0] = 0.0f : sqrtf(q[0]);
	q[1] =(q[1] < 0.0f) ? q[1] = 0.0f : sqrtf(q[1]);
	q[2] =(q[2] < 0.0f) ? q[2] = 0.0f : sqrtf(q[2]);
	q[3] =(q[3] < 0.0f) ? q[3] = 0.0f : sqrtf(q[3]);
	*/
	
	if(q[0] >= q[1] && q[0] >= q[2] && q[0] >= q[3]) //q[0] is max
	{
		 q[1] = (r23 - r32)/(4.f*q[0]);
		 q[2] = (r31 - r13)/(4.f*q[0]);
		 q[3] = (r12 - r21)/(4.f*q[0]);
	}
	else if(q[1] >= q[0] && q[1] >= q[2] && q[1] >= q[3]) //q[1] is max
	{
		 q[0] = (r23 - r32)/(4.f*q[1]);
		 q[2] = (r12 + r21)/(4.f*q[1]);
		 q[3] = (r31 + r13)/(4.f*q[1]);
	}
	else if(q[2] >= q[0] && q[2] >= q[1] && q[2] >= q[3]) //q[2] is max
	{
		 q[0] = (r31 - r13)/(4.f*q[2]);
		 q[1] = (r12 + r21)/(4.f*q[2]);
		 q[3] = (r23 + r32)/(4.f*q[2]);
	}
	else if(q[3] >= q[0] && q[3] >= q[1] && q[3] >= q[2]) //q[3] is max
	{
		 q[0] = (r12 - r21)/(4.f*q[3]);
		 q[1] = (r31 + r13)/(4.f*q[3]);
		 q[2] = (r23 + r32)/(4.f*q[3]);
	}
}

long inv_icm20948_convert_mult_qfix_fxp(long a, long b, unsigned char qfix)
{
    long long temp;
    long result;
    temp = (long long)a * b;
    result = (long)(temp >> qfix);
    return result;
}

static long invn_convert_mult_q29_fxp(long a_q29, long b_q29)
{
	long long temp;
	long result;
	temp = (long long)a_q29 * b_q29;
	result = (long)(temp >> 29);
	return result;

}

void inv_icm20948_convert_quat_to_col_major_matrix_fxp(const long *quat_q30, long *rot_q30)
{
	//Use q29 in order to skip a multiplication by 2
    rot_q30[0] =
        invn_convert_mult_q29_fxp(quat_q30[1], quat_q30[1]) + invn_convert_mult_q29_fxp(quat_q30[0], quat_q30[0]) - 1073741824L;
    rot_q30[1] =
        invn_convert_mult_q29_fxp(quat_q30[1], quat_q30[2]) - invn_convert_mult_q29_fxp(quat_q30[3], quat_q30[0]);
    rot_q30[2] =
        invn_convert_mult_q29_fxp(quat_q30[1], quat_q30[3]) + invn_convert_mult_q29_fxp(quat_q30[2], quat_q30[0]);
    rot_q30[3] =
        invn_convert_mult_q29_fxp(quat_q30[1], quat_q30[2]) + invn_convert_mult_q29_fxp(quat_q30[3], quat_q30[0]);
    rot_q30[4] =
        invn_convert_mult_q29_fxp(quat_q30[2], quat_q30[2]) + invn_convert_mult_q29_fxp(quat_q30[0], quat_q30[0]) - 1073741824L;
    rot_q30[5] =
        invn_convert_mult_q29_fxp(quat_q30[2], quat_q30[3]) - invn_convert_mult_q29_fxp(quat_q30[1], quat_q30[0]);
    rot_q30[6] =
        invn_convert_mult_q29_fxp(quat_q30[1], quat_q30[3]) - invn_convert_mult_q29_fxp(quat_q30[2], quat_q30[0]);
    rot_q30[7] =
        invn_convert_mult_q29_fxp(quat_q30[2], quat_q30[3]) + invn_convert_mult_q29_fxp(quat_q30[1], quat_q30[0]);
    rot_q30[8] =
        invn_convert_mult_q29_fxp(quat_q30[3], quat_q30[3]) + invn_convert_mult_q29_fxp(quat_q30[0], quat_q30[0]) - 1073741824L;
}

static long invn_convert_mult_q15_fxp(long a_q15, long b_q15)
{
	long out = (long)(((long long)a_q15 * (long long)b_q15) >> 15);
	return out;
}

static long invn_convert_inv_sqrt_q15_fxp(long x_q15)
{
    long oneoversqrt2 = 23170L; // int32(2^15*1/sqrt(2))
    long oneandhalf = 49152L; // int32(1.5*2^15);
    long upperlimit = 45426; // int32(log(4)*2^15);
    long lowerlimit = 22713; // int32(log(2)*2^15); 
    long xx, x0_2, invsqrtx;
    int pow2;

    if (x_q15 <= 0)
        return 0L;

    pow2 = 0;
    xx = x_q15;
    if (xx > upperlimit) {
downscale:
        if (xx > upperlimit) {
            xx = xx/2;
            pow2 = pow2 - 1;
            goto downscale;
        }
        goto newton_raphson;
    }

    if (xx < lowerlimit) {
upscale:
        if (xx < lowerlimit) {
            xx = xx*2;
            pow2 = pow2 + 1;
            goto upscale;
        }
        goto newton_raphson;
    }

newton_raphson:
    // 3 NR iterations. In some cases second and/or third iteration may not be needed, however
    // for code simplicity always iterate three times. Fourth iteration is below bit precision.
    x0_2 = xx >>1;
    xx = oneandhalf - x0_2;
    xx = invn_convert_mult_q15_fxp( xx, ( oneandhalf - invn_convert_mult_q15_fxp(x0_2, invn_convert_mult_q15_fxp(xx,xx) ) ) );
    xx = invn_convert_mult_q15_fxp( xx, ( oneandhalf - invn_convert_mult_q15_fxp(x0_2, invn_convert_mult_q15_fxp(xx,xx) ) ) );

    if (pow2 & 1) { // This checks if the number is even or odd.
        pow2 = (pow2>>1) + 1; // Account for sqrt(2) in denominator
        invsqrtx = (invn_convert_mult_q15_fxp(xx,oneoversqrt2));
    }
    else {
        pow2 = pow2>>1;
        invsqrtx =  xx;
    }

    if (pow2 < 0)
        invsqrtx = invsqrtx>>ABS(pow2);
    else if (pow2>0)
        invsqrtx = invsqrtx <<pow2;

    return invsqrtx;
}

static long invn_convert_inverse_q15_fxp(long x_q15)
{
    long y;
    int negx;

	if (x_q15 == 0) {
		y = 0L;
		return y;
	}

    negx=0;
    if (x_q15 < 0 ) {
        x_q15 = -x_q15;
        negx = 1;
    }

	if(x_q15 >= 1073741824L) { // 2^15 in Q15; underflow number
        if (negx)
            y=-1L;
        else
            y = 1L;
		return y;
	}

    y = invn_convert_inv_sqrt_q15_fxp(x_q15); // sqrt(y)
    y = invn_convert_mult_q15_fxp(y, y);

    if (negx)
        y=-y;
    return y;
}

long inv_icm20948_math_atan2_q15_fxp(long y_q15, long x_q15)
{
    long absy, absx, maxABS, tmp, tmp2, tmp3, Z, angle;
    static long constA7[4] = {32740, -10503,  4751, -1254}; // int32(2^15*[0.999133448222780 -0.320533292381664 0.144982490144465,-0.038254464970299]); %7th order
    static long PI15 = 102944; // int32(2^15*pi): pi in Q15

    absx=ABS(x_q15);
    absy=ABS(y_q15);

    maxABS=MAX(absx, absy);
    // SCALE arguments down to protect from roundoff loss due to 1/x operation.
    //% Threshold for scaling found by numericaly simulating arguments
    //% to yield optimal (minimal) error of less than 0.01 deg through
    //% entire range (for Chebycheff order 7).
//    while ( maxABS >> 13) {  --> Or it can be done this way if DMP code is more efficient
    while ( maxABS > 8192L) {
            maxABS=maxABS/2;
            absx=absx/2;
            absy=absy/2;
    }

    {
        if (absx >= absy) // (0, pi/4]: tmp = abs(y)/abs(x);
            tmp = invn_convert_mult_q15_fxp(absy, invn_convert_inverse_q15_fxp(absx));
        else             // (pi/4, pi/2): tmp = abs(x)/abs(y);
            tmp = invn_convert_mult_q15_fxp(absx, invn_convert_inverse_q15_fxp(absy));

        tmp2=invn_convert_mult_q15_fxp(tmp, tmp);
         // Alternatively:
        tmp3 = invn_convert_mult_q15_fxp(constA7[3], tmp2);
        tmp3 = invn_convert_mult_q15_fxp(constA7[2] + tmp3, tmp2);
        tmp3 = invn_convert_mult_q15_fxp(constA7[1] + tmp3, tmp2);
        Z    = invn_convert_mult_q15_fxp(constA7[0] + tmp3, tmp);

        if (absx < absy)
            Z = PI15/2 - Z;

        if (x_q15 < 0) { // second and third quadrant
            if (y_q15 < 0)
                Z = -PI15 + Z;
            else
                Z = PI15 - Z;
        }
        else { // fourth quadrant
            if (y_q15 < 0)
                Z = -Z;
        }
        angle = Z; // Note the result is angle in radians, expressed in Q15.
    }
    return angle;
}

uint8_t *inv_icm20948_convert_int16_to_big8(int16_t x, uint8_t *big8)
{
    big8[0] = (uint8_t)((x >> 8) & 0xff);
    big8[1] = (uint8_t)(x & 0xff);
    return big8;
}

uint8_t *inv_icm20948_convert_int32_to_big8(int32_t x, uint8_t *big8)
{
    big8[0] = (uint8_t)((x >> 24) & 0xff);
    big8[1] = (uint8_t)((x >> 16) & 0xff);
    big8[2] = (uint8_t)((x >> 8) & 0xff);
    big8[3] = (uint8_t)(x & 0xff);
    return big8;
}

int32_t inv_icm20948_convert_big8_to_int32(const uint8_t *big8)
{
    int32_t x;
    x = ((int32_t)big8[0] << 24) | ((int32_t)big8[1] << 16) | ((int32_t)big8[2] << 8)
        | ((int32_t)big8[3]);
    return x;
}
