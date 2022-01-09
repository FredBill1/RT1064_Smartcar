/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
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

/** @defgroup InvError Error code
 *	@brief    Common error code
 *
 *	@ingroup EmbUtils
 *	@{
 */

#ifndef _INV_ERROR_H_
#define _INV_ERROR_H_

/** @brief Common error code definition
 */
enum inv_error
{
	INV_ERROR_SUCCESS      = 0,   /**< no error */
	INV_ERROR              = -1,  /**< unspecified error */
	INV_ERROR_NIMPL        = -2,  /**< function not implemented for given
	                                   arguments */
	INV_ERROR_TRANSPORT    = -3,  /**< error occured at transport level */
	INV_ERROR_TIMEOUT      = -4,  /**< action did not complete in the expected
	                                   time window */
	INV_ERROR_SIZE         = -5,  /**< size/length of given arguments is not
	                                   suitable to complete requested action */
	INV_ERROR_OS           = -6,  /**< error related to OS */
	INV_ERROR_IO           = -7,  /**< error related to IO operation */
	INV_ERROR_MEM          = -9,  /**< not enough memory to complete requested
	                                   action */
	INV_ERROR_HW           = -10, /**< error at HW level */
	INV_ERROR_BAD_ARG      = -11, /**< provided arguments are not good to
	                                   perform requestion action */
	INV_ERROR_UNEXPECTED   = -12, /**< something unexpected happened */
	INV_ERROR_FILE         = -13, /**< cannot access file or unexpected format */
	INV_ERROR_PATH         = -14, /**< invalid file path */
	INV_ERROR_IMAGE_TYPE   = -15, /**< error when image type is not managed */
	INV_ERROR_WATCHDOG     = -16, /**< error when device doesn't respond 
									   to ping */
	INV_ERROR_FIFO_OVERFLOW = -17, /**< FIFO overflow detected */
};

#endif /* _INV_ERROR_H_ */

/** @} */
