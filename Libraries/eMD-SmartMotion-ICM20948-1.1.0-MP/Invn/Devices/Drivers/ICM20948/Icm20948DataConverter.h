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
/** @defgroup icm20948_data_converter data_converter
	@ingroup  SmartSensor_driver
	@{
*/	
#ifndef INV_ICM20948_DATA_CONVERTER_H__
#define INV_ICM20948_DATA_CONVERTER_H__


#ifdef __cplusplus
extern "C"
{
#endif

/* forward declaration */
struct inv_icm20948;

#ifndef M_PI
  #define M_PI 3.14159265358979323846f
#endif 
#define INV_TWO_POWER_NEG_30 9.313225746154785e-010f

#define ABS(x) (((x)>=0)?(x):-(x)) /*!< Computes the absolute value of its argument \a x. \ingroup invn_macro */
#define MAX(x,y) (((x)>(y))?(x):(y)) /*!< Computes the maximum of \a x and \a y. \ingroup invn_macro*/
#define MIN(x,y) (((x)<(y))?(x):(y)) /*!< Computes the minimum of \a x and \a y. \ingroup invn_macro */

//! \def INVN_FLT_TO_FXP
//! Convert the \a value from float to QN value. \ingroup invn_macro 
#define INVN_FLT_TO_FXP(value, shift)	( (int32_t)  ((float)(value)*(1ULL << (shift)) + ( (value>=0)-0.5f )) ) 
//!	Macro to convert float values from an address into QN values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_FLT_TO_FXP(fltptr, fixptr, length, shift)	{ int i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_FLT_TO_FXP((fltptr)[i], shift); }

/** Performs a fixed point quaternion multiply with inverse on second element q1*q2'.
* @param[in] q1 First Quaternion Multicand, length 4. 1.0 scaled
*            to 2^30
* @param[in] q2 Second Quaternion Multicand, length 4. 1.0 scaled
*            to 2^30. Inverse will be take before multiply
* @param[out] qProd Product after quaternion multiply q1*q2'. Length 4.
*             1.0 scaled to 2^30.
*/
void INV_EXPORT inv_icm20948_q_mult_q_qi(const long *q1, const long *q2, long *qProd);

/** @brief Sets the transformation used for chip to body frame
* @param[in] quat 	the quaternion used for the transformation
*/	
void INV_EXPORT inv_icm20948_set_chip_to_body(struct inv_icm20948 * s, long *quat);

/** @brief Converts fixed point DMP rotation vector to floating point android notation
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] values 4 element quaternion in Android format
*/
void INV_EXPORT inv_icm20948_convert_rotation_vector(struct inv_icm20948 * s, const long *quat, float *values);

/** @brief Converts 3 element fixed point DMP rotation vector to 4 element rotation vector in world frame
* @param[in] quat 3 element rotation vector from DMP, missing the scalar part. Converts from Chip frame to World frame
* @param[out] quat4_world 4 element quaternion
*/
void INV_EXPORT inv_icm20948_convert_rotation_vector_2(struct inv_icm20948 * s, const long *quat, long *quat4_world);

/** @brief Converts 4 element rotation vector in world frame to floating point android notation
* @param[in] quat4_world 4 element rotation vector in World frame
* @param[out] values in Android format
*/
void INV_EXPORT inv_icm20948_convert_rotation_vector_3(const long *quat4_world, float *values);

/** @brief Converts the data in android values
* @param[in] vec3 vector of the DMP 
* @param[in] scale scale calculated
* @param[out] values in Android format
*/
void INV_EXPORT inv_icm20948_convert_dmp3_to_body(struct inv_icm20948 * s, const long *vec3, float scale, float *values);

/** @brief Converts the data in android quaternion values
* @param[in] accel_gyro_matrix 	vector of the DMP 
* @param[out] angle 			angle calculated
*/
void INV_EXPORT inv_icm20948_set_chip_to_body_axis_quaternion(struct inv_icm20948 * s,signed char *accel_gyro_matrix, float angle);

/** @brief Converts a 32-bit long to a little endian byte stream
* @param[in] x 				the long to be converted
* @param[in] little8 		little endian byte converted
* @return 					0 on success, negative value on error.
*/	
unsigned char INV_EXPORT *inv_icm20948_int32_to_little8(long x, unsigned char *little8);

/** @brief Converts degree angle to radian
* @param[in] deg_val 		the angle in degree
* @return 					the angle in radian
*/	
float INV_EXPORT inv_icm20948_convert_deg_to_rad(float deg_val);

/** Performs a multiply and shift by 30. 
 * \details These are good functions to write in assembly on
 * with devices with small memory where you want to get rid of the long long which some
 * assemblers don't handle well
 * @param[in] a
 * @param[in] b
 * @return ((long long)a*b)>>30 
 \ingroup ScalarFxp
*/
long INV_EXPORT inv_icm20948_convert_mult_q30_fxp(long a_q30, long b_q30);

/** 
* \brief Compute real part of quaternion, element[0]
* @param[in]  inQuat_q30 3 elements gyro quaternion. Dimension is 3.
* @param[out] outQuat_q30 Quaternion. Dimension is 4. 4 elements gyro quaternion
* \return 0 
* \ingroup QuaternionFxp
*/
// FIXME Define input format and output format. Supposed Q30.
int INV_EXPORT inv_icm20948_convert_compute_scalar_part_fxp(const long * inQuat_q30, long* outQuat_q30);

/** 
* \brief Calculates square-root of a fixed-point number (30 bit mantissa, positive)
* \details Input must be a positive scaled ( \f$ 2^{30} \f$ ) integer
* The number is scaled to lie between a range in which a Newton-Raphson
* iteration works best.
* @param[in] x0_q30 length 1. Fixed point format is Q30
* @return scaled square root if succeed else 0. 
\ingroup ScalarFxp
**/
long INV_EXPORT inv_icm20948_convert_fast_sqrt_fxp(long x0_q30);

/** \brief Auxiliary function used by inv_OneOverX(), inv_fastSquareRoot(), inv_inverseSqrt().
* \details Finds the range of the argument, determines the optimal number of Newton-Raphson
* iterations and .
* Restrictions: Number is represented as Q1.30.
*               Number is betweeen the range 2<x<=0
* @param[in] x0_q30 Input length 1. Number is represented as Q30. Number is betweeen the range 2<x<=0
* @param[out] pow Corresponding square root of the power of two is returned. length 1 
* @return number of Newton Raphson iterations, x0 scaled between log(2) and log(4) and \f$ 2^N \f$ scaling (N=pow)
\ingroup ScalarFxp
*/
int INV_EXPORT inv_icm20948_convert_test_limits_and_scale_fxp(long *x0_q30, int *pow);

/** Auxiliary function used by testLimitsAndScale()
* Find the highest nonzero bit in an unsigned 32 bit integer:
* @param[in] value operand Dimension is 1.
* @return highest bit position.
* \note This function performs the log2 of an interger as well. 
* \ingroup binary
**/
int16_t INV_EXPORT inv_icm20948_convert_get_highest_bit_position(uint32_t *value);

/**
 * \brief Converts a rotation matrix to a quaternion.
 * \param[in] Rcb_q30 Rotation matrix. Fixed point format is Q30. 
 * \param[out] Qcb_q30 quaternion related to provided rotation matrix. Vector size is 4. Fixed point format is Q30. 
 * \ingroup QuaternionFxp
 */
void INV_EXPORT inv_icm20948_convert_matrix_to_quat_fxp(long *Rcb_q30, long *Qcb_q30);

/** 
* \brief Calculates square-root of a fixed-point number
* \details This code calls 1/sqrt(x) and multiplies result with x, i.e. \f$ \sqrt{x}=x*(1/\sqrt{x}) \f$ .
* @param[in] x_q30 Input. Fixed point format is Q30
* @return square root of x0 in Q30.
\ingroup ScalarFxp
**/
long INV_EXPORT inv_icm20948_convert_sqrt_q30_fxp(long x_q30);

/** 
* \brief Calculates 1/square-root of a fixed-point number (30 bit mantissa, positive): Q1.30
* \details The number is scaled to lie between a range in which a Newton-Raphson iteration works best. 
*  Caller must scale final result by 2^rempow (while avoiding overflow).
* @param[in] x_q30 Input. The input must be positive. Fixed point format is Q30. 
* @param[out] pow2 Corresponding square root of the power of two is returned. length 1
* @return square root of x in Q30.
\ingroup ScalarFxp
*/
long INV_EXPORT inv_icm20948_convert_inv_sqrt_q30_fxp(long x_q30, int *pow2);

/** 
* \brief Inverse function based on Newton-Raphson 1/sqrt(x) calculation
 \details Note that upshifting c (the result) by pow2 right away will overflow q30 if b<0.5 in q30 (=536870912). \n
 So if you are doing some multiplication later on (like a/b), then it might be better
 to do <code>q30_mult(a,c)</code> first and then shift it up by pow2: <code>q30_mult(a,c)<<pow2</code> \n
 The result might still overflow in some cases (large a, small b: a=1073741824, b=1
 but precise limits of the overflow are tbd).
 
* @param[in] x_q30 the operand. Fixed point format is Q30
* @param[in] pow2 a power of 2 by which 1/b is downshifted to fit in q30.
* @return  the 1/x result in Q30 downshifted by pow2.
\ingroup ScalarFxp
**/
long INV_EXPORT inv_icm20948_convert_inverse_q30_fxp(long x_q30, int *pow2);

/**
 * Converts a rotation matrix to a quaternion in floating point.
 * \param[in] R Rotation matrix in floating point. The
 *             First 3 elements of the rotation matrix, represent
 *             the first row of the matrix. 
 * \param[out] q 4-element quaternion in floating point.
 * \warning This functions does not retrieve fixed point quaternion anymore. Use a conversion flt_to_fxp.
 * \ingroup QuaternionFlt
 */
// FIXME This function can be optimized in term of operation numbers. But a good test must be writtent first.
void INV_EXPORT inv_icm20948_convert_matrix_to_quat_flt(float *R, float *q);

/** 
 * \brief Performs a multiply and shift by shift. 
 * \details These are good functions to write in assembly on 
 * with devices with small memory where you want to get rid of
 * the long long which some assemblers don't handle well
 * @param[in] a First multicand
 * @param[in] b Second multicand
 * @param[in] shift Shift amount after multiplying
 * @return ((long long)a*b)>>shift
 * \warning Same function that invn_math_mult_qfix_fxp. 
 \ingroup ScalarFxp
*/
long INV_EXPORT inv_icm20948_convert_mult_qfix_fxp(long a, long b, unsigned char qfix);

/**
 * \brief Converts a quaternion to a rotation matrix in column major convention.
 * @param [in] quat_q30 4-element quaternion in fixed point. Fixed point format is Q30.
 * @param[ out] rot_q30 Rotation matrix in fixed point. One is 2^30. 
 * The Rotation matrix multiplied by a 3 element column vector transforms a vector from Body to World.
 * \warning output matrix storage is column major. 
 * \ref colmajor_convention
 * \ingroup MatrixFxp
 */
void INV_EXPORT inv_icm20948_convert_quat_to_col_major_matrix_fxp(const long *quat_q30, long *rot_q30);

/** \brief Seventh order Chebychev polynomial approximation in Q15.
 \details Chebychev 7th order polynomial approximation : <br />
 \li in fixed point : \f$ constA7 = \text{int32}(2^{15}*[0.999133448222780 -0.320533292381664 0.144982490144465,-0.038254464970299]); \f$ <br />
 \li in float : \f$ A = \begin{bmatrix}[0.999133 & -0.320533 & 0.144982 &-0.0382544 \end{bmatrix}); \f$ <br />
 
 The related formula is : <br />
 \f$ \xi = \begin{cases} |y|/|x| &&  \text{in }(0, \pi/4] \\ |x|/|y| &&  \text{in } (\pi/4, \pi/2) \end{cases} , \quad
 Cheb = A(1)*\xi + A(2)*\xi^3 + A(3)*\xi^5 + A(4)*\xi^7 \f$ 

7th Order Accuracy is +/-0.02 degrees (worst case) through entire range (accomplished with scaling). <br />
This code depends on: \ref reciprocal_fun_q15 , \ref inverse_sqrt_q15 , \ref inv_q15_mult

* @param [in] y_q15 first operand of atan2(y, x). Fixed point format is Q15.
* @param [in] x_q15 second operand of atan2(y, x). Fixed point format is Q15.
* @return output angle in radians. Fixed point format is Q15.
* \ingroup GeometryFxp
*/
long INV_EXPORT inv_icm20948_math_atan2_q15_fxp(long y_q15, long x_q15);

/** 
 * \brief Converts a 16-bit short to a big endian byte stream 
 * \param [in] x operand 
 * \param [out] big8 big endian byte stream
 * \return big8 pointer
 * \ingroup binary
 */
uint8_t INV_EXPORT *inv_icm20948_convert_int16_to_big8(int16_t x, uint8_t *big8);

/** 
 * \brief Converts a 32-bit long to a big endian byte stream 
 * \param [in] x operand 
 * \param [out] big8 big endian byte stream
 * \return big8 pointer
 * \ingroup binary
*/
uint8_t INV_EXPORT *inv_icm20948_convert_int32_to_big8(int32_t x, uint8_t *big8);

/** 
 * \brief Converts a big endian byte stream into a 32-bit long 
 * \param [in] big8 big endian byte stream
 * \return corresponding 32-bit integer.
 * \ingroup binary
 */
int32_t INV_EXPORT inv_icm20948_convert_big8_to_int32(const uint8_t *big8);

/** 
 * \brief Converts long values according to quat_30 matrix
 * \param [in] quat_30 mounting matrix to apply
 * \param [in] in long values to be converted
 * \param [out] out long values converted
 * \return void
 */
void INV_EXPORT inv_icm20948_convert_quat_rotate_fxp(const long *quat_q30, const long *in, long *out);
#ifdef __cplusplus
}
#endif
#endif	/* INV_ICM20948_DATA_CONVERTER_H__ */

/** @} */
