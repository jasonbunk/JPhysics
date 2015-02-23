/*
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: The MIT License (MIT) (Expat)
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "Games/GameSystem.h"
#include "Entities.h"
#include "Games/CollisionTools.h"
#include "Fluid.h"

#include "GLVertexes.h"

class EntDistSorter
{
public:
	phys::IEntity* myEnt;
	phys::WorldLine* myLine;
	JPHYS_FLOAT_UNIT myDist;

	EntDistSorter() {myEnt=NULL; myLine=NULL; myDist = 0.0;}

	EntDistSorter(phys::IEntity* ent, phys::WorldLine* LINE, JPHYS_FLOAT_UNIT dist) {myEnt=ent; myLine = LINE; myDist = dist;}
	
	//allows stl sorting algorithms
	INLINE bool operator< (const EntDistSorter &rhs) const {return ((*this).myDist < rhs.myDist);}
};

namespace phys
{
using namespace drawing; //from GLVertexes.h

	GameSystem::GameSystem()
	{
		MouseDown_L = false;
		MouseDown_R = false;
		drawMouseCursor = false;
		OnMouseL_Down = NULL;
		OnMouseR_Down = NULL;
		OnMouseL_Raised = NULL;
		OnMouseR_Raised = NULL;
		movingWorldLine = NULL;
		drawFluidOnUpdate = true;
	}

	GameSystem::~GameSystem()
	{
	}

	void GameSystem::deleteNearestEntity(JPHYS_FLOAT_UNIT maxDistance)
	{
		if(allEntities.empty())
			return;

		std::list<IEntity*>::iterator Iter = allEntities.begin();

		std::list<EntDistSorter> distSorter;

		for( ; Iter!=allEntities.end(); )
		{
			distSorter.push_back(EntDistSorter(*Iter, NULL, cTools::getDist(MousePos, (*Iter)->CenterPos)));
			Iter++;
		}
		distSorter.sort();

		if(distSorter.begin()->myDist >= maxDistance)
			return;
		
		for(Iter = allEntities.begin(); Iter!=allEntities.end(); )
		{
			if((*Iter) == distSorter.begin()->myEnt)
			{
				delete (*Iter);
				allEntities.erase(Iter);
				return;
			}
			else
				Iter++;
		}
	}

	void GameSystem::linkWorldLineToMouse(JPHYS_FLOAT_UNIT maxDistance)
	{
		if(allLines.empty())
			return;
		
		movingWorldLine = NULL; //if nothing found nearby
		linkedLastPos = MousePos;
		
		std::list<WorldLine*>::iterator Iter = allLines.begin();

		std::list<EntDistSorter> distSorter;

		for( ; Iter!=allLines.end(); )
		{
			(*Iter)->UpdateLinks();
			distSorter.push_back(EntDistSorter(NULL, *Iter, cTools::getDist(MousePos, (*Iter)->getCenter() )));
			Iter++;
		}
		distSorter.sort();

		if(distSorter.begin()->myDist >= maxDistance)
			return;
		
		for(Iter = allLines.begin(); Iter!=allLines.end(); )
		{
			if((*Iter) == distSorter.begin()->myLine)
			{
				movingWorldLine = (*Iter);
				return;
			}
			else
				Iter++;
		}
	}

	void GameSystem::updateLinkedToMouse()
	{
		if(movingWorldLine != NULL)
		{
			std::list<WorldLine*>::iterator Iter = allLines.begin();
			for( ; Iter!=allLines.end(); Iter++)
				(*Iter)->ResetMoveability();

			movingWorldLine->Move(true, (MousePos - linkedLastPos));

			linkedLastPos = MousePos;
		}
	}

	void GameSystem::unlinkFromMouse()
	{
		movingWorldLine = NULL;
	}

	void GameSystem::deleteNearestLine(JPHYS_FLOAT_UNIT maxDistance)
	{
		if(allLines.empty())
			return;

		std::list<WorldLine*>::iterator Iter = allLines.begin();

		std::list<EntDistSorter> distSorter;

		for( ; Iter!=allLines.end(); )
		{
			distSorter.push_back(EntDistSorter(NULL, *Iter, cTools::getDist(MousePos, (*Iter)->getCenter() )));
			Iter++;
		}
		distSorter.sort();

		if(distSorter.begin()->myDist >= maxDistance)
			return;
		
		for(Iter = allLines.begin(); Iter!=allLines.end(); )
		{
			if((*Iter) == distSorter.begin()->myLine)
			{
				delete (*Iter);
				allLines.erase(Iter); //we have to iterate again so we know which line to erase from game system
				return;
			}
			else
				Iter++;
		}
	}
	
	void GameSystem::SetMouseL(bool down)
	{
		if(down != MouseDown_L)
		{
			if(down)
			{
				MouseStartPosL = MousePos;
				if(OnMouseL_Down != NULL)
					OnMouseL_Down();
			}
			else if(OnMouseL_Raised != NULL)
			{
				OnMouseL_Raised();
			}
		}
		MouseDown_L = down;
	}

	void GameSystem::SetMouseR(bool down)
	{
		if(down != MouseDown_R)
		{
			if(down)
			{
				MouseStartPosR = MousePos;
				if(OnMouseR_Down != NULL)
					OnMouseR_Down();
			}
			else if(OnMouseR_Raised != NULL)
			{
				OnMouseR_Raised();
			}
		}
		MouseDown_R = down;
	}

	void GameSystem::updateAll(JPHYS_FLOAT_UNIT time)
	{
		updateLinkedToMouse();

		EntitySystem::updateAll(time);
	}
	
	void GameSystem::drawAll()
	{
		std::list<IEntity*>::iterator Iter = allEntities.begin();
		for( ; Iter!=allEntities.end(); Iter++)
		{
			(*Iter)->draw();
		}
		std::list<IEntityOld*>::iterator OldIter = allOldEntities.begin();
		for( ; OldIter!=allOldEntities.end(); OldIter++)
		{
			(*OldIter)->draw();
		}
		std::list<I3DEntityOld*>::iterator Old3DIter = allOld3DEntities.begin();
		for( ; Old3DIter!=allOld3DEntities.end(); Old3DIter++)
		{
			(*Old3DIter)->draw();
		}
		std::list<WorldLine*>::iterator LineIter = allLines.begin();
		for( ; LineIter!=allLines.end(); LineIter++)
		{
			(*LineIter)->draw(lineColors);
		}
		
		if(myFluid != NULL && drawFluidOnUpdate)
			myFluid->draw();
		
		if(drawMouseCursor)
		{
			// Draw the interpolated points second.
			glBegin( GL_POINTS );
			if(MouseDown_L)
				glColor3ub(255-lineColors[0],255-lineColors[1],255-lineColors[2]);
			else if(MouseDown_R)
				glColor3ub(255-lineColors[0],lineColors[1],lineColors[2]);
			else
				glColor3ub(lineColors[0],lineColors[1],lineColors[2]);
			glVertex2f( MousePos.x, MousePos.y );
			glEnd();
		}
	}
	
	void GameSystem::drawAllWithOffset(point offset, point aspectStretch)
	{
		std::list<IEntity*>::iterator Iter = allEntities.begin();
		for( ; Iter!=allEntities.end(); Iter++)
		{
			(*Iter)->drawWithOffset(offset, aspectStretch);
		}
		std::list<IEntityOld*>::iterator OldIter = allOldEntities.begin();
		for( ; OldIter!=allOldEntities.end(); OldIter++)
		{
			(*OldIter)->drawWithOffset(offset, aspectStretch);
		}
		std::list<I3DEntityOld*>::iterator Old3DIter = allOld3DEntities.begin();
		for( ; Old3DIter!=allOld3DEntities.end(); Old3DIter++)
		{
			(*Old3DIter)->drawWithOffset(offset, aspectStretch);
		}
		std::list<WorldLine*>::iterator LineIter = allLines.begin();
		for( ; LineIter!=allLines.end(); LineIter++)
		{
			glBegin(GL_LINES);
			glColor3ub(lineColors[0],lineColors[1],lineColors[2]);
			GLVERT2((*(*LineIter)).p1, offset, aspectStretch);
			GLVERT2((*(*LineIter)).p2, offset, aspectStretch);
			glEnd();
		}
		
		if(myFluid != NULL)
			myFluid->drawWithOffset(offset, aspectStretch);
		
		if(drawMouseCursor)
		{
			// Draw the interpolated points second.
			glBegin( GL_POINTS );
			if(MouseDown_L)
				glColor3ub(255-lineColors[0],255-lineColors[1],255-lineColors[2]);
			else if(MouseDown_R)
				glColor3ub(255-lineColors[0],lineColors[1],lineColors[2]);
			else
				glColor3ub(lineColors[0],lineColors[1],lineColors[2]);

			GLVERT2( MousePos, offset, aspectStretch );

			glEnd();
		}
	}
}
