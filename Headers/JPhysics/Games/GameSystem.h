
#ifndef __JPHYS_GAME_SYS_H__
#define __JPHYS_GAME_SYS_H__

#include "exports.h"
#include "EntitySystem.h"

namespace phys
{
	class GameSystem : public EntitySystem
	{
	protected:
		bool MouseDown_L;
		bool MouseDown_R;

		WorldLine* movingWorldLine;

		void updateLinkedToMouse();

	public:
		bool drawMouseCursor;
		bool drawFluidOnUpdate;

		point linkedLastPos;
		point MousePos;
		point MouseStartPosL;
		point MouseStartPosR;

		INLINE bool isLinkedToSomething() {return (movingWorldLine!=NULL);}

		INLINE bool GetMouseDown_L() {return MouseDown_L;}
		INLINE bool GetMouseDown_R() {return MouseDown_R;}

		void (*OnMouseL_Down)(void);
		void (*OnMouseR_Down)(void);

		void (*OnMouseL_Raised)(void);
		void (*OnMouseR_Raised)(void);

		JPHYSICS_API void SetMouseL(bool down);
		JPHYSICS_API void SetMouseR(bool down);

		JPHYSICS_API void linkWorldLineToMouse(JPHYS_FLOAT_UNIT maxDistance);
		JPHYSICS_API void unlinkFromMouse();

		JPHYSICS_API GameSystem();
		JPHYSICS_API virtual ~GameSystem();
		
		JPHYSICS_API virtual void updateAll(JPHYS_FLOAT_UNIT time);

		JPHYSICS_API virtual void drawAll();
		JPHYSICS_API virtual void drawAllWithOffset(point offset, point aspectStretch);
		
		JPHYSICS_API void deleteNearestEntity(JPHYS_FLOAT_UNIT maxDistance=1.0);
		JPHYSICS_API void deleteNearestLine(JPHYS_FLOAT_UNIT maxDistance=1.0);
	};
}

#endif
