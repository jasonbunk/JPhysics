
#pragma once

// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the JPHYSICS_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// JPHYSICS_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#if 0

#ifdef JPHYSICS_EXPORTS
#define JPHYSICS_API __declspec(dllexport)
#else
#define JPHYSICS_API __declspec(dllimport)
#endif

#else

#define JPHYSICS_API

#endif



#include "FloatUnits.h"
