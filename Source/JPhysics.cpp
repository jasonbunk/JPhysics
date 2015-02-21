// JPhysics.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "JPhysics.h"


// This is an example of an exported variable
JPHYSICS_API int nJPhysics=0;

// This is an example of an exported function.
JPHYSICS_API int fnJPhysics(void)
{
	return 42;
}

/*
// This is the constructor of a class that has been exported.
// see JPhysics.h for the class definition
CPhysics::CPhysics()
{
	return;
}
*/