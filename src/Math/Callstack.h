#pragma once

#include "../MathGeoLibFwd.h"
#include <string>

// Sometimes we want an explicit function call to occur, especially when related to operating with callstack fetching.

#define NOINLINE __attribute__((noinline))

std::string GetCallstack(const char *indent = "", const char *ignoreFilter = 0);
