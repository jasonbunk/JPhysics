
#ifndef __JPHYS_ENTITY_SYS_H__
#define __JPHYS_ENTITY_SYS_H__

#include "exports.h"
#include "Entities.h"
//#include "Fluid_Old.h" //use one fluid "entity" to handle all quantities of fluid in a game
#include <list>
#include "Games/WorldLine.h"

namespace phys
{
	class ent_fluid;
	class ent_fluid_old;
	class WorldLine;
	class FluidField;

	class EntitySystem
	{
	protected:

		ent_fluid* myFluid;

	public:

		FluidField* my_FluidField;
		
		std::list<IEntityOld*> allOldEntities;
		std::list<I3DEntityOld*> allOld3DEntities;

		bool stop_entities_that_exit_level_boundaries;

		std::list<IEntity*> allEntities;

		std::list<WorldLine*> allLines;
		unsigned char lineColors[3];

		unsigned char INTEGRATOR;

		JPHYS_FLOAT_UNIT curFrameTime;
		
		point drawScale;
		vec3 gravity;
		vec3 universeLimits;

//----------------
		
		JPHYSICS_API EntitySystem();
		JPHYSICS_API virtual ~EntitySystem();


		JPHYSICS_API virtual void updateAll(JPHYS_FLOAT_UNIT time);
		
		JPHYSICS_API virtual void drawAll();
		JPHYSICS_API virtual void drawAllWithOffset(point offset, point aspectStretch);
		
		JPHYSICS_API virtual void deleteAll();
		
		JPHYSICS_API void addNewLine(point start, point end);

		JPHYSICS_API void addNewEntity(IEntity* newEnt);
		JPHYSICS_API void addNewEntity(IEntityOld* newEnt);
		JPHYSICS_API void addNewEntity(I3DEntityOld* newEnt);
		
		JPHYSICS_API ent_fluid* GetFluidSys() {return myFluid;}

		JPHYSICS_API void checkCollisions(const line &hitSurface, const point& offset);

		JPHYSICS_API virtual void initFluidSystem(SpawnParams_Fluid fluidParams);
		
		JPHYSICS_API virtual void initFluidSystem(ent_fluid* newFluid);
		
		JPHYSICS_API void addNewFluidPoint(SpawnParams pointParams);

		JPHYSICS_API INLINE int GetEntCount() {return allEntities.size();}

		JPHYSICS_API IEntity* GetEntAtOffset(int offset);

		JPHYSICS_API INLINE int GetLinesCount() {return allLines.size();}

		JPHYSICS_API void applyImpulseToAll(point Impulse);
		
		JPHYSICS_API void applyImpulse(point Location, point Impulse);
	};

}

#endif
