if (CMAKE_C_COMPILER MATCHES ".*(clang|emcc).*" OR CMAKE_C_COMPILER_ID MATCHES ".*(Clang|emcc).*")
	set(COMPILER_IS_CLANG TRUE)
endif()

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER MATCHES ".*(gcc|clang|emcc).*" OR CMAKE_C_COMPILER_ID MATCHES ".*(GCC|Clang|emcc).*")
	set(IS_GCC_LIKE TRUE)
else()
	set(IS_GCC_LIKE FALSE)
endif()

if (IS_GCC_LIKE AND NOT COMPILER_IS_CLANG)
	set(COMPILER_IS_GCC TRUE)
endif()

option(GENERATE_ASM_LISTING "Generate assembly listing of all compiled code" FALSE)

if ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
	set(LINUX TRUE)
endif()

if (WIN32 AND IS_GCC_LIKE)
	add_definitions(-DWIN32)
endif()

# Add the global _DEBUG flag from WIN32 platform to all others, which is universally used in MGL to
# perform debug-mode-specific compilation.
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

set(optFlags "-DNDEBUG -DMATH_SILENT_ASSUME -DRELEASE -DOPTIMIZED_RELEASE")

set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${optFlags}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${optFlags}")
set(CMAKE_C_FLAGS_RELWITHDEBINFO     "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${optFlags}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${optFlags}")

if (IS_GCC_LIKE AND GENERATE_ASM_LISTING)
	# -fkeep-inline-functions: Inline everything, but also keep a separate inlined copy for asm outputting purposes.
	# Add -ffast-math and -fno-math-errno
	set(outputAsmCodeFlags "-S -fkeep-inline-functions -fverbose-asm -g")

	# Prefer outputting Intel syntax for assembly.
	if (COMPILER_IS_CLANG)
		set(outputAsmCodeFlags "${outputAsmCodeFlags} -mllvm --x86-asm-syntax=intel")
	else()
		set(outputAsmCodeFlags "${outputAsmCodeFlags} -masm=intel -Wa,-alnd") # GCC
	endif()

	# To interleave source code, run 'as -alhnd file.s > file.lst'
	set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${outputAsmCodeFlags}")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${outputAsmCodeFlags}")
endif()

if (LINUX)
	add_definitions(-DLINUX)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "PathScale")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER MATCHES ".*(gcc|clang|emcc).*" OR CMAKE_C_COMPILER_ID MATCHES ".*(GCC|Clang|emcc).*")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -Wno-variadic-macros")
else()
  message(WARNING "Unknown compiler '${CMAKE_C_COMPILER}'/'${CMAKE_C_COMPILER_ID}' used! Cannot set up max warning level.")
endif()

if (COMPILER_IS_GCC)
	if (MATH_SSE OR MATH_SSE2 OR MATH_SSE3 OR MATH_SSE41 OR MATH_AVX)
		add_definitions(-mfpmath=sse)
	endif()
endif()

if (MATH_FMA4 OR MATH_FMA3)
	# Between FMA3 and FMA4, the intrinsics are the same so C code doesn't need to know which to call,
	# it can just call _mm_fmadd_ps(), so this passed #define doesn't need to distinguish.
	add_definitions(-DMATH_FMA)
	# However for GCC codegen, it needs to know which instruction set to target:
	if (IS_GCC_LIKE)
		# http://gcc.gnu.org/onlinedocs/gcc-4.8.2/gcc/i386-and-x86-64-Options.html#i386-and-x86-64-Options
		if (MATH_FMA4)
			add_definitions(-mfma4)
		else()
			add_definitions(-mfma)
		endif()
	endif()
endif()

if (MATH_AVX)
	add_definitions(-DMATH_AVX)
	if (IS_GCC_LIKE)
		# http://gcc.gnu.org/onlinedocs/gcc-4.8.2/gcc/i386-and-x86-64-Options.html#i386-and-x86-64-Options
		add_definitions(-mavx -march=corei7-avx -mtune=corei7-avx)
	endif()
elseif (MATH_SSE41)
	add_definitions(-DMATH_SSE41)
	if (IS_GCC_LIKE)
		add_definitions(-msse4.1)
		add_definitions(-march=corei7 -mtune=corei7)
	endif()
elseif (MATH_SSE3)
	add_definitions(-DMATH_SSE3)
	if (IS_GCC_LIKE)
		add_definitions(-msse3)
		add_definitions(-march=core2 -mtune=core2)
	endif()
elseif (MATH_SSE2)
	add_definitions(-DMATH_SSE2)
	if (IS_GCC_LIKE)
		add_definitions(-msse2)
		add_definitions(-march=pentium4 -mtune=pentium4)
	endif()
elseif (MATH_SSE)
	add_definitions(-DMATH_SSE)
	if (IS_GCC_LIKE)
		add_definitions(-msse)
		add_definitions(-march=pentium3 -mtune=pentium3)
	endif()
endif()

if (MATH_ENABLE_UNCOMMON_OPERATIONS)
	add_definitions(-DMATH_ENABLE_UNCOMMON_OPERATIONS)
endif()

if (MATH_NEON)
	add_definitions(-DMATH_NEON)
	if (IS_GCC_LIKE)
		add_definitions(-mfpu=neon)
	endif()
endif()

if (MATH_AUTOMATIC_SSE)
	add_definitions(-DMATH_AUTOMATIC_SSE)
endif()

if (FAIL_USING_EXCEPTIONS)
	set(CMAKE_C_FLAGS_DEBUG     "${CMAKE_C_FLAGS_DEBUG} -DFAIL_USING_EXCEPTIONS")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DFAIL_USING_EXCEPTIONS")
endif()

if (MATH_TESTS_EXECUTABLE)
	add_definitions(-DMATH_TESTS_EXECUTABLE)

	if (BUILD_FOR_GCOV)
		if (IS_GCC_LIKE)
			add_definitions(-fprofile-arcs -ftest-coverage)
			set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs")
			set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fprofile-arcs")
			set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fprofile-arcs")
		endif()
	endif()
endif()
