cmake_minimum_required(VERSION 3.0)

project(MathGeoLib)

include(CommonOptions.cmake)

file(GLOB_RECURSE sourceFiles ./src/*.cpp ./src/*.c)
file(GLOB_RECURSE headerFiles ./src/*.h)

if (MATH_TESTS_EXECUTABLE)
	file(GLOB_RECURSE testSourceFiles ./tests/*.cpp)
	file(GLOB_RECURSE testHeaderFiles ./tests/*.h)
	set(sourceFiles ${sourceFiles} ${testSourceFiles})
	set(headerFiles ${headerFiles} ${testHeaderFiles})

	# Tests use C++11 syntax.
	set (CMAKE_CXX_STANDARD 11)
	add_executable(MathGeoLib ${sourceFiles} ${headerFiles})
else()
	add_library(MathGeoLib STATIC ${sourceFiles} ${headerFiles})
endif()

if (LINUX)
	# clock_gettime() is found from the library librt on linux.
	target_link_libraries(MathGeoLib rt)
endif()
