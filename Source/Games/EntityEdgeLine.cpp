
#include "stdafx.h"
#include "Games/CollisionTools.h"
#include "Games/EntityEdgeLine.h"
#include "GLVertexes.h"


namespace phys
{
using namespace drawing; //from GLVertexes.h

	//to really do this, we need to know the masses of both! edge lines should have a mass proportional to entity's total mass

	void EntityEdgeLine::CollideWith(bool collisionKnown, point & other_pos_start, point & other_pos_end, point & other_vel)
	{
		if(pos1==NULL || pos2==NULL)
			return;

		if(collisionKnown == true)
		{
			other_pos_end = other_pos_start;
			
			*vel1 = ((*vel1 + other_vel)*0.5);
			*vel2 = ((*vel2 + other_vel)*0.5);

			cTools::collisionRotation(other_vel, line(*pos1, *pos2));
		}
	}

	void EntityEdgeLine::checkCollisions(const point &hitParticle, const point& particleOffset)
	{
	}

	void EntityEdgeLine::Push(const point& origin, const point & deltaV)
	{
		if(pos1==NULL || pos2==NULL)
			return;

		phys::point distances;
		distances.x = cTools::getDist(origin,*pos1);
		distances.y = cTools::getDist(origin,*pos2);

		distances.Normalize();

		*vel1 += (deltaV*distances.x);
		*vel2 += (deltaV*distances.y);
	}

	JPHYS_FLOAT_UNIT EntityEdgeLine::GetLength() const
	{
		return ((pos1 && pos2) ? cTools::getDist(*pos1, *pos2) : 0.0);
	}

	void EntityEdgeLine::draw(unsigned char * colors)
	{
		if(pos1 && pos2)
		{
			glBegin(GL_LINES);
			glColor3ub(colors[0], colors[1], colors[2]);
			GLVERT2(*pos1);
			GLVERT2(*pos2);
			glEnd();
		}
	}
}
