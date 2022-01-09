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

/** @defgroup InvAssert InvAssert
	@brief Custom runtime and compile-time assert macros definitions
    @ingroup EmbUtils
    @{
*/

#ifndef _InvAssert_h_
#define _InvAssert_h_

#ifdef __cplusplus
extern "C" {
#endif


/** @brief 	assert macro
*/
#ifdef NDEBUG
#undef 	ASSERT
#define ASSERT(...) (void)0
#else
#ifndef ASSERT
#include <assert.h>
#define ASSERT(...) assert(__VA_ARGS__)
#else
extern void InvAssert(const char *predicate, const char *file, unsigned line);
#undef ASSERT
#define ASSERT(...) \
		do {	\
			if(!(__VA_ARGS__))	\
				InvAssert(#__VA_ARGS__, __FILE__, __LINE__);	\
		} while(0)
#endif /* ASSERT */
#endif 	/* NDEBUG */


/** @brief 	Compile-time assert macro
*/
#ifndef CASSERT
#define _impl_PASTE(a,b) a##b
#define _impl_CASSERT_LINE(predicate, line) \
		typedef char _impl_PASTE(assertion_failed_,line)[2*!!(predicate)-1];
#define CASSERT(predicate) _impl_CASSERT_LINE(predicate, __LINE__)

/* Avoid some annoying warning under GCC when using CASSERT() */
#if defined(__GNUC__) && __GNUC__ > 4
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif

#endif 	/* CASSERT */

#ifdef __cplusplus
}
#endif

#endif /* _InvAssert_h_ */

/** @} */
