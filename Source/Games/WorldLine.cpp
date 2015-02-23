/*
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: The MIT License (MIT) (Expat)
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "Games/CollisionTools.h"
#include "Games/WorldLine.h"
#include "GLVertexes.h"
#include "Entities.h"
#include "Fluid.h"

namespace phys
{
using namespace drawing; //from GLVertexes.h

	WorldLine::WorldLine(EntitySystem* EntSys, JPHYS_FLOAT_UNIT X1, JPHYS_FLOAT_UNIT Y1, JPHYS_FLOAT_UNIT X2, JPHYS_FLOAT_UNIT Y2)
	{
		p1.x = X1;
		p1.y = Y1;
		p2.x = X2;
		p2.y = Y2;
		myEntSys = EntSys;
	}
	WorldLine::WorldLine(EntitySystem* EntSys, point P1, point P2)
	{
		p1=P1;
		p2=P2;
		myEntSys = EntSys;
	}
	WorldLine::WorldLine(EntitySystem* EntSys, line oldLine)
	{
		p1=oldLine.p1;
		p2=oldLine.p2;
		myEntSys = EntSys;
	}

	void WorldLine::UpdateLinks()
	{
		linkedLines.clear();

		if(!myEntSys->allLines.empty())
		{
			std::list<WorldLine*>::iterator LineIter = myEntSys->allLines.begin();
			for( ; LineIter!=myEntSys->allLines.end(); LineIter++)
			{
				if((*LineIter) != this)
				{
					if(cTools::checkIntersect(line(p1,p2), *(*LineIter)))
					{
						linkedLines.push_back(*LineIter);
					}
				}
			}
		}
	}

	void WorldLine::Move(bool moveLinks, point offset, WorldLine* originator)
	{	
		hasBeenMovedThisFrame = true;

		myEntSys->checkCollisions((*this), offset);
		
		p1 += offset;
		p2 += offset;

		if(moveLinks)
		{
			std::list<WorldLine*>::iterator LineIter = linkedLines.begin();
			for( ; LineIter!=linkedLines.end(); LineIter++)
			{
				if(((*LineIter) != originator) && ((*LineIter)->CanMove()))
					(*LineIter)->Move(true, offset, this);
			}
		}
	}

	void WorldLine::ResetMoveability()
	{
		hasBeenMovedThisFrame = false;
	}

	void WorldLine::draw(unsigned char * colors)
	{
		glBegin(GL_LINES);
		glColor3ub(colors[0], colors[1], colors[2]);
		GLVERT2(p1);
		GLVERT2(p2);
		glEnd();
	}
}
