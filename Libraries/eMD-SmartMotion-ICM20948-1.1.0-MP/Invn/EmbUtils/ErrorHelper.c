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

#include "ErrorHelper.h"

const char * inv_error_str(int error)
{
	switch(error) {
	case INV_ERROR_SUCCESS:      return "Success";
	case INV_ERROR:              return "Unspecified error";
	case INV_ERROR_NIMPL:        return "Not implemented";
	case INV_ERROR_TRANSPORT:    return "Transport error";
	case INV_ERROR_TIMEOUT:      return "Timeout, action did not complete in time";
	case INV_ERROR_SIZE:         return "Wrong size error";
	case INV_ERROR_OS:           return "Operating system failure";
	case INV_ERROR_IO:           return "Input/Output error";
	case INV_ERROR_MEM: 		 return "Bad allocation";
	case INV_ERROR_HW:           return "Hardware error";
	case INV_ERROR_BAD_ARG:      return "Invalid arguments";
	case INV_ERROR_UNEXPECTED:   return "Unexpected error";
	case INV_ERROR_FILE:         return "Invalid file format";
	case INV_ERROR_PATH:         return "Invalid file path";
	case INV_ERROR_IMAGE_TYPE:   return "Unknown image type";
	case INV_ERROR_WATCHDOG:     return "Watchdog error";
	case INV_ERROR_FIFO_OVERFLOW: return "FIFO overflow error";
	default:                     return "Unknown error";
	}
}