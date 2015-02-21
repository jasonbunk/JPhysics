//-----------------------------------------------------------------
// September 23, 2012
// Jason Bunk
// EntityEdgeLine.h
//
// Like a "templine", a line formed by pointers to vertices,
// used for collisions with the outer surface of an entity,
// useful because you can apply forces to this and it will apply to
// both of the attached vertices of the entity edge.
//-----------------------------------------------------------------

#ifndef __ENTITY_EDGE_LINE_H__
#define __ENTITY_EDGE_LINE_H__

#include "exports.h"
#include "Templates.h"

namespace phys
{
	class JPHYSICS_API EntityEdgeLine
	{
	public:
			point* pos1; point* vel1;
			point* pos2; point* vel2;

		EntityEdgeLine() {pos1=0; pos2=0;
						  vel1=0; vel2=0;}

		EntityEdgeLine(point* start_pos, point* start_vel, point* end_pos, point* end_vel)
		{
			pos1 = start_pos;	vel1 = start_vel;
			pos2 =   end_pos;	vel2 =   end_vel;
		}

		point GetP1() const {return pos1 ? (*pos1) : point(0.0, 0.0);}
		point GetP2() const {return pos2 ? (*pos2) : point(0.0, 0.0);}

		line GetLine() const {return ((pos1 && pos2) ? line(*pos1, *pos2) : line(0,0,1,1));}

		JPHYS_FLOAT_UNIT GetLength() const;

		void CollideWith(bool collisionKnown, point & other_pos_start, point & other_pos_end, point & other_vel);

		void checkCollisions(const point &hitParticle, const point& particleOffset);

		void Push(const point& origin, const point & deltaV);

		void draw(unsigned char * colors);
	};
}


#endif
