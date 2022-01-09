/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
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

#include "Message.h"

#include <stdio.h>
#include <stdlib.h>

static int               msg_level;
static inv_msg_printer_t msg_printer;

void inv_msg_printer_default(int level, const char * str, va_list ap)
{
	(void)level, (void)str, (void)ap;
}

void inv_msg_setup(int level, inv_msg_printer_t printer)
{
	msg_level   = level;
	if (level < INV_MSG_LEVEL_OFF)
		msg_level = INV_MSG_LEVEL_OFF;
	else if (level > INV_MSG_LEVEL_MAX)
		msg_level = INV_MSG_LEVEL_MAX;
	msg_printer = printer;
}

void inv_msg(int level, const char * str, ...)
{
	if(level && level <= msg_level && msg_printer) {
		va_list ap;
		va_start(ap, str);
		msg_printer(level, str, ap);
		va_end(ap);
	}
}

int inv_msg_get_level(void)
{
	return msg_level;
}
