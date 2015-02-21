
#ifndef __JPHYSICS_DLL__
#define __JPHYSICS_DLL__


#include "exports.h"
#include "FloatUnits.h"


/*// This class is exported from the JPhysics.dll
class JPHYSICS_API CPhysics
{
public:
	CPhysics(void);
	// TODO: add your methods here.
};*/

extern JPHYSICS_API int nJPhysics;

JPHYSICS_API int fnJPhysics(void);


//---------------------------------------
//-- Entities

#include "Dot.h"
#include "Robotics/PendulumOnCart.h"
#include "Robotics/PendulumOnCart_WithRotInertia.h"
#include "Robotics/DoublePendulumOnCart.h"
#include "Robotics/DCMotor_Cart.h"
#include "Robotics/DCMotor_PendCart.h"
#include "Robotics/DCMotor2_PendCart.h"
#include "Fluid.h"
#include "Fluid_Old.h"
//#include "FluidField.h"
#include "Rectangle.h"
#include "RectangleNew.h"
#include "RectangleWithTorque.h"
#include "TestDiffEq.h"

//-- Fundamentals

#include "Entities.h"
#include "EntitySystem.h"
#include "Hit.h"
#include "Templates.h"
#include "Games/GameSystem.h"
#include "Games/EntityEdgeLine.h"
#include "Games/WorldLine.h"

//-- Tools

#include "Games/CollisionTools.h"
#include "Integrators.h"
#include "mathTools.h"
#include "AdvancedMiscMath.h"
#include "GLVertexes.h"
#include "FiniteDifferenceMethod.h"


#include "clamp.h"
#include "string_to_int.h"
#include "Timer.h"

#endif
