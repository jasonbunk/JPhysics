
#include "stdafx.h"
#include "EntitySystem.h"
#include <list>
#include "Fluid.h"

#include "GLVertexes.h"

namespace phys
{
using namespace drawing; //from GLVertexes.h
	
	EntitySystem::EntitySystem() :
		stop_entities_that_exit_level_boundaries(true),
		INTEGRATOR(1),
		myFluid(nullptr),
		my_FluidField(nullptr)
	{
		gravity.y = -9.8;

		drawScale.x = 1.0;
		drawScale.y = 1.0;

		universeLimits.x = 1000.0;
		universeLimits.y = 1000.0;
		universeLimits.z = 1000.0;

		lineColors[0] = 254;
		lineColors[1] = 1;
		lineColors[2] = 1;
	}

	EntitySystem::~EntitySystem()
	{
		deleteAll();
	}

	void EntitySystem::checkCollisions(const line &hitSurface, const point& offset)
	{
		if(myFluid != NULL)
			myFluid->checkCollisions(hitSurface, offset);

		return;
		
		/*
		std::list<IEntity*>::iterator EntityIter = allEntities.begin();
		for( ; EntityIter!=allEntities.end(); )
		{
			//IEntity:
			//std::vector<EntityEdgeLine> DangerLines;

			std::vector<EntityEdgeLine>::iterator DangerIter = (*EntityIter)->DangerLines.begin();
			for(; DangerIter != (*EntityIter)->DangerLines.end(); DangerIter++)
			{
			}
		}
		*/
	}

	void EntitySystem::updateAll(JPHYS_FLOAT_UNIT time)
	{
		curFrameTime = time;
		
		JPHYS_FLOAT_UNIT tempFriction(0.0);

		std::list<IEntity*>::iterator Iter = allEntities.begin();

		for( ; Iter!=allEntities.end(); )
		{
			if(stop_entities_that_exit_level_boundaries &&
			  ( ((*Iter)->CenterPos.x >   universeLimits.x)
			||	((*Iter)->CenterPos.x < -(universeLimits.x))
			||	((*Iter)->CenterPos.y >   universeLimits.y)
			||	((*Iter)->CenterPos.y < -(universeLimits.y)) ))
			{
				delete (*Iter);
				Iter = allEntities.erase(Iter);
			}
			else
			{
				tempFriction = (*Iter)->friction;
				(*Iter)->friction *= (200.0 * time);
				(*Iter)->update(time);
				(*Iter)->friction = tempFriction;

				Iter++;
			}
		}

		std::list<IEntityOld*>::iterator OldIter = allOldEntities.begin();
		for( ; OldIter != allOldEntities.end(); )
		{
			if(stop_entities_that_exit_level_boundaries &&
			  ( ((*OldIter)->CenterPos.x >   universeLimits.x)
			||	((*OldIter)->CenterPos.x < -(universeLimits.x))
			||	((*OldIter)->CenterPos.y >   universeLimits.y)
			||	((*OldIter)->CenterPos.y < -(universeLimits.y)) ))
			{
				if((*OldIter)->delete_when_outside_screen)
				{
					delete (*OldIter);
					OldIter = allOldEntities.erase(OldIter);
				}
				else
				{
					(*OldIter)->i_have_finished_my_drawing_of_my_path = true;
					OldIter++;
				}
			}
			else
			{
				if((*OldIter)->update(time))
				{
					delete (*OldIter);
					OldIter = allOldEntities.erase(OldIter);
				}
				else
					OldIter++;
			}
		}
		std::list<I3DEntityOld*>::iterator Old3DIter = allOld3DEntities.begin();
		for( ; Old3DIter != allOld3DEntities.end(); )
		{
			if(stop_entities_that_exit_level_boundaries &&
			  ( ((*Old3DIter)->CenterPos.x >   universeLimits.x)
			||	((*Old3DIter)->CenterPos.x < -(universeLimits.x))
			||	((*Old3DIter)->CenterPos.y >   universeLimits.y)
			||	((*Old3DIter)->CenterPos.y < -(universeLimits.y))
			||	((*Old3DIter)->CenterPos.z >   universeLimits.z)
			||	((*Old3DIter)->CenterPos.z < -(universeLimits.z)) ))
			{
				if((*Old3DIter)->delete_when_outside_screen)
				{
					delete (*Old3DIter);
					Old3DIter = allOld3DEntities.erase(Old3DIter);
				}
				else
				{
					(*Old3DIter)->i_have_finished_my_drawing_of_my_path = true;
					Old3DIter++;
				}
			}
			else
			{
				if((*Old3DIter)->update(time))
				{
					delete (*Old3DIter);
					Old3DIter = allOld3DEntities.erase(Old3DIter);
				}
				else
					Old3DIter++;
			}
		}
		
		if(myFluid != NULL)
		{
			tempFriction = myFluid->friction;
			myFluid->friction *= (200.0 * time);
			myFluid->update(time);
			myFluid->friction = tempFriction;
		}
	}

	void EntitySystem::drawAll()
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
		
		if(myFluid != NULL)
			myFluid->draw();
	}
	
	void EntitySystem::drawAllWithOffset(point offset, point aspectStretch)
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
	}

	void EntitySystem::deleteAll()
	{
		std::list<IEntity*>::iterator Iter = allEntities.begin();
		for( ; Iter!=allEntities.end(); )
		{
			delete (*Iter);
			Iter = allEntities.erase(Iter);
		}
		std::list<WorldLine*>::iterator LineIter = allLines.begin();
		for( ; LineIter!=allLines.end(); )
		{
			delete (*LineIter);
			LineIter = allLines.erase(LineIter);
		}
		std::list<IEntityOld*>::iterator OldIter = allOldEntities.begin();
		for( ; OldIter!=allOldEntities.end(); )
		{
			delete (*OldIter);
			OldIter = allOldEntities.erase(OldIter);
		}
		std::list<I3DEntityOld*>::iterator Old3DIter = allOld3DEntities.begin();
		for( ; Old3DIter!=allOld3DEntities.end(); )
		{
			delete (*Old3DIter);
			Old3DIter = allOld3DEntities.erase(Old3DIter);
		}
		
		if(myFluid != NULL)
			myFluid->deleteAllElements();

	}

	void EntitySystem::addNewLine(point start, point end)
	{
		WorldLine* newL = new WorldLine(this, start, end);

		if(newL != NULL)
			allLines.push_back(newL);
	}

	void EntitySystem::initFluidSystem(ent_fluid* newFluid)
	{
		if(myFluid != NULL)
		{
			delete myFluid; myFluid=NULL;
		}
		myFluid = newFluid;
		if(myFluid != NULL)
			myFluid->myEntSystem = this;
	}

	void EntitySystem::initFluidSystem(SpawnParams_Fluid fluidParams)
	{
		if(myFluid == NULL)
		{
			myFluid = new ent_fluid(fluidParams);
			if(myFluid!=NULL)
				myFluid->myEntSystem = this;
			//else
			//	big error message
		}
	}

	void EntitySystem::addNewFluidPoint(SpawnParams pointParams)
	{
		if(myFluid != NULL)
		{
			myFluid->add_element(pointParams);
		}
	}

	void EntitySystem::addNewEntity(IEntity* newEnt)
	{
		newEnt->myEntSystem = this;
		allEntities.push_back(newEnt);
	}

	void EntitySystem::addNewEntity(IEntityOld* newEnt)
	{
		newEnt->myEntSystem = this;
		allOldEntities.push_back(newEnt);
	}
	
	void EntitySystem::addNewEntity(I3DEntityOld* newEnt)
	{
		newEnt->myEntSystem = this;
		allOld3DEntities.push_back(newEnt);
	}

	IEntity* EntitySystem::GetEntAtOffset(int offset)
	{
		if(allEntities.empty()) return NULL;

		std::list<IEntity*>::iterator Iter = allEntities.begin();
		int a = 0;
		for( ; Iter!=allEntities.end(); )	//ouch, this is the cost of using a list instead of a vector
		{
			if(a == offset)
			{
				return (*Iter);
			}
			a++;
			Iter++;
		}
		return NULL;
	}

	void EntitySystem::applyImpulseToAll(point Impulse)
	{
		std::list<IEntity*>::iterator Iter = allEntities.begin();
		
		for( ; Iter!=allEntities.end(); Iter++)
		{
			(*Iter)->applyImpulse((*Iter)->CenterPos, Impulse);
		}
	}
	
	void EntitySystem::applyImpulse(point Location, point Impulse)
	{
		std::list<IEntity*>::iterator Iter = allEntities.begin();
		
		for( ; Iter!=allEntities.end(); Iter++)
		{
			(*Iter)->applyImpulse(Location, Impulse);
		}
	}
}
