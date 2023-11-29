/* Copyright Jukka Jyl�nki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file myassert.h
	@author Jukka Jyl�nki
	@brief Control over assert() macro for MathGeoLib. */
#include "MathLog.h"
#include "assume.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <sstream>
#endif

#ifdef assert
#undef assert
#endif

#if defined(OPTIMIZED_RELEASE) || defined(NDEBUG)
#define RuntimeFailure(str) ((void)0)
#define RUNTIME_FAILURE_DISABLED
#else
#define RuntimeFailure(str) do { LOGE("%s", str); } while(0)
#endif

#ifdef RUNTIME_FAILURE_DISABLED

#define assert(x) ((void)0)
#define asserteq(x,y) ((void)0)
#define assertcmp(x, cmp, y) ((void)0)

// If defined in a compilation unit, MATH_IGNORE_UNUSED_VARS_WARNING will kill all subsequent warnings about variables going unused.
// This occurs commonly in unit tests that are also doubled as benchmarks - killing the use of assert() macro will leave variables
// that were otherwise not used, except when assert()ing a condition. As a simplest measure, MATH_IGNORE_UNUSED_VARS_WARNING will
// operate only when assert() is a no-op, so that in debug builds etc. real instances of unused variables are still caught.
#if defined(__GNUC__) && (__GNUC__*10000+__GNUC_MINOR*100) >= 40600
// GCC 4.6 introduced a new warning "-Wunused-but-set-variable", so suppress that on those compilers.
#define MATH_IGNORE_UNUSED_VARS_WARNING _Pragma("GCC diagnostic ignored \"-Wunused-variable\"") _Pragma("GCC diagnostic ignored \"-Wunused-but-set-variable\"")
#else
#define MATH_IGNORE_UNUSED_VARS_WARNING _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")
#endif

#else

#define MATH_IGNORE_UNUSED_VARS_WARNING

#define assert(x) \
	do { \
		if (!(x)) \
		{ \
			const char *error_ = #x " in " __FILE__ ":" STRINGIZE(__LINE__); \
			MARK_UNUSED(error_); /* Appease cppcheck to not complain that error is unused. */ \
			RuntimeFailure(error_); \
		} \
	} while(0)

#define asserteq(x,y) \
	do { \
		if ((x) != (y)) \
		{ \
			std::string str = std::string("Assertion '" #x "' == '" #y "' failed! (") + ObjToString(x) + " != " + ObjToString(y) + ("!) in " __FILE__ ":" STRINGIZE(__LINE__)); \
			RuntimeFailure(str.c_str()); \
		} \
	} while(0)

#define assertcmp(x, cmp, y) \
	do { \
		if (!((x) cmp (y))) \
		{ \
			std::string str = std::string("Assertion '" #x "' " #cmp " '" #y "' failed! (") + ObjToString(x) + " and " + ObjToString(y) + ("!) in " __FILE__ ":" STRINGIZE(__LINE__)); \
			RuntimeFailure(str.c_str()); \
		} \
	} while(0)

#endif
