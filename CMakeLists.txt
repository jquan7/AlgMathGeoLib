cmake_minimum_required(VERSION 3.0)

project(AlgMathGeoLib)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

if (MATH_TESTS_EXECUTABLE)
	add_definitions(-DMATH_TESTS_EXECUTABLE)
endif()

file(GLOB_RECURSE sourceFiles ./src/*.cpp ./src/*.c)
file(GLOB_RECURSE headerFiles ./src/*.h)

if (MATH_TESTS_EXECUTABLE)
	# Tests use C++11 syntax. (at least for now the main library does not need to)
	set (CMAKE_CXX_STANDARD 11)
	file(GLOB_RECURSE testSourceFiles ./tests/*.cpp)
	file(GLOB_RECURSE testHeaderFiles ./tests/*.h)
	set(sourceFiles ${sourceFiles} ${testSourceFiles})
	set(headerFiles ${headerFiles} ${testHeaderFiles})
	add_executable(AlgMathGeoLib ${sourceFiles} ${headerFiles})
else()
	add_library(AlgMathGeoLib STATIC ${sourceFiles} ${headerFiles})
endif()

if (LINUX)
	# clock_gettime() is found from the library librt on linux.
	target_link_libraries(AlgMathGeoLib rt)
endif()

