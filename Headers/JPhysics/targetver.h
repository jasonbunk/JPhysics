#pragma once

// Including SDKDDKVer.h defines the highest available Windows platform.

// If you wish to build your application for a previous Windows platform, include WinSDKVer.h and
// set the _WIN32_WINNT macro to the platform you wish to support before including SDKDDKVer.h.


// platform detection

#define PLATFORM_WINDOWS  1
#define PLATFORM_MAC      2
#define PLATFORM_LINUX     3
#define PLATFORM_UNSUPPORTED 4

#if defined(_WIN32)
#define THISPLATFORM PLATFORM_WINDOWS
#elif defined(__APPLE__)
#define THISPLATFORM PLATFORM_MAC
#elif defined(__linux__)
#define THISPLATFORM PLATFORM_LINUX
#else
#define THISPLATFORM PLATFORM_UNSUPPORTED
#endif

//#if THISPLATFORM == PLATFORM_WINDOWS
//#include <SDKDDKVer.h>
//#endif

#if THISPLATFORM == PLATFORM_LINUX
#ifndef INLINE
#define INLINE 
#endif
#else
#ifndef INLINE
#define INLINE 
#endif
#endif
