/* Copyright 2010 Jukka Jyl�nki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Clock.cpp
	@brief */

#if defined(__unix__) || defined (__CYGWIN__)
#include <time.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#endif

#ifdef WIN32
#include "../Math/InclWindows.h"
#endif

#include "Clock.h"
#include "../Math/myassert.h"
#include "../Math/assume.h"

MATH_BEGIN_NAMESPACE

#ifdef WIN32
u64 Clock::ddwTimerFrequency;
#endif

tick_t Clock::appStartTime = 0;

Clock impl;

void Clock::InitClockData()
{
	if (appStartTime == 0)
		appStartTime = Tick();

#ifdef WIN32
	if (!QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&ddwTimerFrequency)))
	{
		LOGE("The system doesn't support high-resolution timers!");
		ddwTimerFrequency = (u64)-1;
	}

	if (appStartTime == 0)
	{
		appStartTime = (tick_t)GetTickCount64();
	}
#endif
}

Clock::Clock()
{
	InitClockData();
}

void Clock::Sleep(int milliseconds)
{
	timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = (milliseconds - ts.tv_sec * 1000) * 1000 * 1000;
	int ret = nanosleep(&ts, NULL);
	if (ret == -1)
		LOGI("nanosleep returned -1! Reason: %s(%d).", strerror(errno), (int)errno);
}

int Clock::Year()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wYear;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Month()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wMonth;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Day()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wDay;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Hour()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wHour;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Min()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wMinute;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Sec()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wSecond;
#else
	///\todo.
	return 0;
#endif
}

unsigned long Clock::SystemTime()
{
#ifdef WIN32
	return (unsigned long)GetTickCount64();
#else
	return TickU32();
#endif
}
/*
tick_t Clock::ApplicationStartupTick()
{
	return appStartTime;
}
*/
unsigned long Clock::Time()
{
	return (unsigned long)((Tick() - appStartTime) * 1000 / Clock::TicksPerSec());
}

tick_t Clock::Tick()
{
	timeval t;
	gettimeofday(&t, NULL);
	return (tick_t)t.tv_sec * 1000 * 1000 + (tick_t)t.tv_usec;
}

unsigned long Clock::TickU32()
{
#ifdef WIN32
	LARGE_INTEGER ddwTimer;
	BOOL success = QueryPerformanceCounter(&ddwTimer);
	assume(success != 0);
	MARK_UNUSED(success);
	return ddwTimer.LowPart;
#else
	return (unsigned long)Tick();
#endif
}

tick_t Clock::TicksPerSec()
{
	return 1000 * 1000;
}

unsigned long long Clock::Rdtsc()
{
#if defined(__x86_64__)
	unsigned hi, lo;
	__asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
	return ((unsigned long long)lo) | (((unsigned long long)hi) << 32);
#else
	return Clock::Tick();
#endif
}

MATH_END_NAMESPACE
