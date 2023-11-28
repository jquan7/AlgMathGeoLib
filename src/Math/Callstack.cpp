#include "Callstack.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#endif
#include "assume.h"

#if defined(LINUX)

#include <stdlib.h>
#include <execinfo.h>
#include <string.h>

StringT NOINLINE GetCallstack(const char *indent, const char *ignoreFilter)
{
	const int N = 128;
	void *callstack[N];
	int n = backtrace(callstack, N);
	char **strs = backtrace_symbols(callstack, n);
	StringT stack;
	for(int i = 0; i < n; ++i)
	{
		if (strstr(strs[i], "_Z12GetCallstackPK") != 0)
			continue;
		if (!ignoreFilter || strstr(strs[i], ignoreFilter) != 0)
		{
			stack += indent;
			stack += strs[i];
			stack += '\n';
		}
	}
	free(strs);
	return stack;
}

#else

StringT GetCallstack(const char *indent, const char *ignoreFilter)
{
	// Not implemented on this platform.
	return StringT();
}

#endif
