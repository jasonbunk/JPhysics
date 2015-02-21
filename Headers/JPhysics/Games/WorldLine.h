//-----------------------------------------------------------------
// September 12, 2012
// Jason Bunk
// WorldLine.h
//
// A mostly static environmental line, for others to collide with.
// It contains "links" to connected lines, so that if the user wants
// to move this line, it has the possibility to move all of its connected lines too.
//-----------------------------------------------------------------

#ifndef __JPHYSICS_WORLD_LINE_H__
#define __JPHYSICS_WORLD_LINE_H__

#include "exports.h"
#include "EntitySystem.h"
#include "Templates.h"
#include "mathTools.h"
#include <list>

namespace phys
{
	class WorldLine : public line	//inherits from line, so that collision functions still work with this class
	{
	protected:
		std::list<WorldLine*> linkedLines;
		bool hasBeenMovedThisFrame;

		EntitySystem* myEntSys;
	public:

		inline bool CanMove() {return (!hasBeenMovedThisFrame);}
		
		WorldLine(EntitySystem* EntSys, JPHYS_FLOAT_UNIT X1, JPHYS_FLOAT_UNIT Y1, JPHYS_FLOAT_UNIT X2, JPHYS_FLOAT_UNIT Y2);
		WorldLine(EntitySystem* EntSys, point P1, point P2);
		WorldLine(EntitySystem* EntSys, line oldLine);

		void UpdateLinks();
		void draw(unsigned char * colors);
		void Move(bool moveLinks, point offset, WorldLine* originator=NULL);

		void ResetMoveability();
	};
}

#endif
