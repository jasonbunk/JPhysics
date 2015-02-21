
#include "stdafx.h"
#include "Games/CollisionTools.h"
#include "mathTools.h"

#define AA LINE1.p1
#define CC LINE2.p1
#define CCEDGE EDGELINE.GetP1()

#define BB LINE1.p2
#define DD LINE2.p2
#define DDEDGE EDGELINE.GetP2()

namespace phys
{
namespace cTools
{
	//faster (no sqrt)
	JPHYS_FLOAT_UNIT getDistSquared( JPHYS_FLOAT_UNIT x1, JPHYS_FLOAT_UNIT y1, JPHYS_FLOAT_UNIT x2, JPHYS_FLOAT_UNIT y2 )
	{
		return ((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1));
	}
	JPHYS_FLOAT_UNIT getDistSquared( const point& p1, const point& p2 )
	{
		return ((p2.x-p1.x)*(p2.x-p1.x)) + ((p2.y-p1.y)*(p2.y-p1.y));
	}
	JPHYS_FLOAT_UNIT getDistSquared( const vec3& p1, const vec3& p2 )
	{
		return ((p2.x-p1.x)*(p2.x-p1.x)) + ((p2.y-p1.y)*(p2.y-p1.y)) + ((p2.z-p1.z)*(p2.z-p1.z));
	}


	JPHYS_FLOAT_UNIT getDist( const point& p1, const point& p2 )
	{
		return sqrt(((p2.x-p1.x)*(p2.x-p1.x)) + ((p2.y-p1.y)*(p2.y-p1.y)));
	}

	JPHYS_FLOAT_UNIT getDist( const vec3& p1, const vec3& p2 )
	{
		return sqrt(((p2.x-p1.x)*(p2.x-p1.x)) + ((p2.y-p1.y)*(p2.y-p1.y)) + ((p2.z-p1.z)*(p2.z-p1.z)));
	}


	JPHYS_FLOAT_UNIT getDistToInfiniteLine( const point& pP, const line& LINE)
	{
		return fabs((pP.x-LINE.p1.x)*(LINE.p2.y - LINE.p1.y) - (pP.y - LINE.p1.y)*(LINE.p2.x - LINE.p1.x)) / LINE.getLength();
	}

	JPHYS_FLOAT_UNIT getDist( const point& pP, const line& LINE)
	{
		// Return minimum distance between line segment vw and point p
		const JPHYS_FLOAT_UNIT l2 = LINE.getLengthSquared();  // i.e. |w-v|^2 -  avoid a sqrt

		if(mathTools::cmpFloatToZero(l2)) return getDist(pP, LINE.p1);   // v == w case

		// Consider the line extending the segment, parameterized as v + t (w - v).
		// We find projection of point p onto the line. 
		// It falls where t = [(p-v) . (w-v)] / |w-v|^2
		const JPHYS_FLOAT_UNIT t = point::dot(pP - LINE.p1, LINE.p2 - LINE.p1) / l2;

		if (t < 0.0)
			return getDist(pP, LINE.p1);       // Beyond the 'v' end of the segment
		else if (t > 1.0)
			return getDist(pP, LINE.p2);  // Beyond the 'w' end of the segment

		const point projection = LINE.p1 + ((LINE.p2 - LINE.p1)*t);  // Projection falls on the segment

		return getDist(pP, projection);
	}

	JPHYS_FLOAT_UNIT getDist( const point& pP, const point& LINE_P1, const point& LINE_P2)
	{
		// Return minimum distance between line segment vw and point p
		const JPHYS_FLOAT_UNIT l2 = getDistSquared(LINE_P1, LINE_P2);  // i.e. |w-v|^2 -  avoid a sqrt

		if(mathTools::cmpFloatToZero(l2)) return getDist(pP, LINE_P1);   // v == w case

		// Consider the line extending the segment, parameterized as v + t (w - v).
		// We find projection of point p onto the line. 
		// It falls where t = [(p-v) . (w-v)] / |w-v|^2
		const JPHYS_FLOAT_UNIT t = point::dot(pP - LINE_P1, LINE_P2 - LINE_P1) / l2;

		if (t < 0.0)
			return getDist(pP, LINE_P1);       // Beyond the 'v' end of the segment
		else if (t > 1.0)
			return getDist(pP, LINE_P2);  // Beyond the 'w' end of the segment

		const point projection = LINE_P1 + ((LINE_P2 - LINE_P1)*t);  // Projection falls on the segment

		return getDist(pP, projection);
	}

	Hit nearestHitAlongLine( const point& pP, const line& LINE)
	{
		// Return minimum distance between line segment p1,p2 and point pP
		const JPHYS_FLOAT_UNIT l2 = LINE.getLengthSquared();  // i.e. |p2-p1|^2 -  avoid a sqrt

		if(mathTools::cmpFloatToZero(l2)) return Hit(false, getDist(pP, LINE.p1), LINE.p1);   // p1 == p2 case

		// Consider the line extending the segment, parameterized as p1 + t (p2 - p1).
		// We find projection of point pP onto the line. 
		// It falls where t = [(pP-p1) . (p2-p1)] / |p2-p1|^2
		const JPHYS_FLOAT_UNIT t = point::dot(pP - LINE.p1, LINE.p2 - LINE.p1) / l2;

		if (t < 0.0)
			return Hit(false, getDist(pP, LINE.p1), LINE.p1); // Beyond the p1 end of the segment
		else if (t > 1.0)
			return Hit(false, getDist(pP, LINE.p2), LINE.p2); // Beyond the p2 end of the segment

		const point projection = LINE.p1 + ((LINE.p2 - LINE.p1)*t);  // Projection falls on the segment

		return Hit(false, getDist(pP, projection), projection);
	}

	Hit getHitOrDist( const line& LINE1, const line& LINE2 )
	{
		point s1;
		JPHYS_FLOAT_UNIT s2_x, s2_y;
		s1.x = BB.x - AA.x;     s1.y = BB.y - AA.y;
		s2_x = DD.x - CC.x;     s2_y = DD.y - CC.y;

		JPHYS_FLOAT_UNIT s, t;
		s = (-s1.y * (AA.x - CC.x) + s1.x * (AA.y - CC.y)) / (-s2_x * s1.y + s1.x * s2_y);
		t = ( s2_x * (AA.y - CC.y) - s2_y * (AA.x - CC.x)) / (-s2_x * s1.y + s1.x * s2_y);

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		{
			// Collision detected
			//if (i_x != NULL)
			//	*i_x = AA.x + (t * s1_x);
			//if (i_y != NULL)
			//	*i_y = AA.y + (t * s1_y);

			point temP = AA + (s1*t);

			return Hit(true, getDist(LINE2.p1, temP), temP);
		}

		return nearestHitAlongLine(LINE1.p1, LINE2);
	}

	Hit getHitOrDist( const line& LINE1, const EntityEdgeLine& EDGELINE )
	{
		point s1;
		JPHYS_FLOAT_UNIT s2_x, s2_y;
		s1.x = BB.x - AA.x;     s1.y = BB.y - AA.y;
		s2_x = DDEDGE.x - CCEDGE.x;     s2_y = DDEDGE.y - CCEDGE.y;

		JPHYS_FLOAT_UNIT s, t;
		s = (-s1.y * (AA.x - CCEDGE.x) + s1.x * (AA.y - CCEDGE.y)) / (-s2_x * s1.y + s1.x * s2_y);
		t = ( s2_x * (AA.y - CCEDGE.y) - s2_y * (AA.x - CCEDGE.x)) / (-s2_x * s1.y + s1.x * s2_y);

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		{
			// Collision detected
			//if (i_x != NULL)
			//	*i_x = AA.x + (t * s1_x);
			//if (i_y != NULL)
			//	*i_y = AA.y + (t * s1_y);

			point temP = AA + (s1*t);

			return Hit(true, getDist(EDGELINE.GetP1(), temP), temP);
		}

		return nearestHitAlongLine(LINE1.p1, EDGELINE.GetLine());
	}

	Hit getHit( const line& LINE1, const line& LINE2 )
	{
		point s1;
		JPHYS_FLOAT_UNIT s2_x, s2_y;
		s1.x = BB.x - AA.x;     s1.y = BB.y - AA.y;
		s2_x = DD.x - CC.x;     s2_y = DD.y - CC.y;

		JPHYS_FLOAT_UNIT s, t;
		s = (-s1.y * (AA.x - CC.x) + s1.x * (AA.y - CC.y)) / (-s2_x * s1.y + s1.x * s2_y);
		t = ( s2_x * (AA.y - CC.y) - s2_y * (AA.x - CC.x)) / (-s2_x * s1.y + s1.x * s2_y);

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		{
			// Collision detected
			//if (i_x != NULL)
			//	*i_x = AA.x + (t * s1_x);
			//if (i_y != NULL)
			//	*i_y = AA.y + (t * s1_y);

			point temP = AA + (s1*t);

			return Hit(true, getDist(LINE2.p1, temP), temP);
		}

		return false; // No collision
	}

	void collisionRotation(point& velocity, const line& surface)
	{
		JPHYS_FLOAT_UNIT c1 = atan2(velocity.y, velocity.x);

		JPHYS_FLOAT_UNIT s2 = atan2((surface.p2.y-surface.p1.y), (surface.p2.x-surface.p1.x));

		s2 = (s2 - c1)*2.0; //this is the angle we will rotate by: the difference between them, times two

		c1 = cos(s2);
		s2 = sin(s2);

		JPHYS_FLOAT_UNIT tx = velocity.x;

		velocity.x = tx * c1 - velocity.y * s2;
		velocity.y = tx * s2 + velocity.y * c1;
	}
	
	//the rectangle is defined by two corners, start and end
	bool checkPointInRectangle(const point &a, const point &start, const point &end)
	{
		if(		((start.x <= a.x && a.x <= end.x) && (start.y <= a.y && a.y <= end.y))
			||	((start.x >= a.x && a.x >= end.x) && (start.y <= a.y && a.y <= end.y))
			||	((start.x >= a.x && a.x >= end.x) && (start.y >= a.y && a.y >= end.y))
			||	((start.x <= a.x && a.x <= end.x) && (start.y >= a.y && a.y >= end.y))	)
		{
			return true;
		}
		return false;
	}
	
	//JPHYSICS_API bool      ___IsOnSegment(JPHYS_FLOAT_UNIT xi, JPHYS_FLOAT_UNIT yi, JPHYS_FLOAT_UNIT xj, JPHYS_FLOAT_UNIT yj, JPHYS_FLOAT_UNIT xk, JPHYS_FLOAT_UNIT yk);
	//JPHYSICS_API char ___ComputeDirection(JPHYS_FLOAT_UNIT xi, JPHYS_FLOAT_UNIT yi, JPHYS_FLOAT_UNIT xj, JPHYS_FLOAT_UNIT yj, JPHYS_FLOAT_UNIT xk, JPHYS_FLOAT_UNIT yk);

	static INLINE bool ___IsOnSegment(JPHYS_FLOAT_UNIT xi, JPHYS_FLOAT_UNIT yi, JPHYS_FLOAT_UNIT xj, JPHYS_FLOAT_UNIT yj, JPHYS_FLOAT_UNIT xk, JPHYS_FLOAT_UNIT yk)
	{
		return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) &&
			   (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
	}
	static char ___ComputeDirection(JPHYS_FLOAT_UNIT xi, JPHYS_FLOAT_UNIT yi, JPHYS_FLOAT_UNIT xj, JPHYS_FLOAT_UNIT yj, JPHYS_FLOAT_UNIT xk, JPHYS_FLOAT_UNIT yk)
	{
		JPHYS_FLOAT_UNIT a = (xk - xi) * (yj - yi);
		JPHYS_FLOAT_UNIT b = (xj - xi) * (yk - yi);
		return a < b ? -1 : a > b ? 1 : 0;
	}
	bool checkIntersect( const line& LINE1, const line& LINE2 )
	{
		char d1 = ___ComputeDirection(LINE2.p1.x, LINE2.p1.y, LINE2.p2.x, LINE2.p2.y, LINE1.p1.x, LINE1.p1.y);
		char d2 = ___ComputeDirection(LINE2.p1.x, LINE2.p1.y, LINE2.p2.x, LINE2.p2.y, LINE1.p2.x, LINE1.p2.y);
		char d3 = ___ComputeDirection(LINE1.p1.x, LINE1.p1.y, LINE1.p2.x, LINE1.p2.y, LINE2.p1.x, LINE2.p1.y);
		char d4 = ___ComputeDirection(LINE1.p1.x, LINE1.p1.y, LINE1.p2.x, LINE1.p2.y, LINE2.p2.x, LINE2.p2.y);
		return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
				((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) ||
				(d1 == 0 && ___IsOnSegment(LINE2.p1.x, LINE2.p1.y, LINE2.p2.x, LINE2.p2.y, LINE1.p1.x, LINE1.p1.y)) ||
				(d2 == 0 && ___IsOnSegment(LINE2.p1.x, LINE2.p1.y, LINE2.p2.x, LINE2.p2.y, LINE1.p2.x, LINE1.p2.y)) ||
				(d3 == 0 && ___IsOnSegment(LINE1.p1.x, LINE1.p1.y, LINE1.p2.x, LINE1.p2.y, LINE2.p1.x, LINE2.p1.y)) ||
				(d4 == 0 && ___IsOnSegment(LINE1.p1.x, LINE1.p1.y, LINE1.p2.x, LINE1.p2.y, LINE2.p2.x, LINE2.p2.y));
	}

	bool checkPointIsToTheLeftOfLine(const point& pCheck, const point& LINE_P1, const point& LINE_P2)
	{
		if(((LINE_P2.x - LINE_P1.x)*(pCheck.y - LINE_P1.y) - (LINE_P2.y - LINE_P1.y)*(pCheck.x - LINE_P1.x)) > 0.0) //only difference left/right is > or <
		{
			if(	(pCheck.y > LINE_P1.y && pCheck.y < LINE_P2.y)
			||	(pCheck.y > LINE_P2.y && pCheck.y < LINE_P1.y))
				return true;
		}
		return false;
	}
	bool checkPointIsToTheLeftOfLine(const point& pCheck, const line& LINE)
	{
		return checkPointIsToTheLeftOfLine(pCheck, LINE.p1, LINE.p2);
	}
	
	bool checkPointIsToTheRightOfLine(const point& pCheck, const point& LINE_P1, const point& LINE_P2)
	{
		if(((LINE_P2.x - LINE_P1.x)*(pCheck.y - LINE_P1.y) - (LINE_P2.y - LINE_P1.y)*(pCheck.x - LINE_P1.x)) < 0.0) //only difference left/rightis > or <
		{
			if(	(pCheck.y > LINE_P1.y && pCheck.y < LINE_P2.y)
			||	(pCheck.y > LINE_P2.y && pCheck.y < LINE_P1.y))
				return true;
		}
		return false;
	}
	bool checkPointIsToTheRightOfLine(const point& pCheck, const line& LINE)
	{
		return checkPointIsToTheRightOfLine(pCheck, LINE.p1, LINE.p2);
	}

	JPHYS_FLOAT_UNIT CosineBetweenTwoLineSegments(const phys::vec3 & pstart, const phys::vec3 & pmiddle, const phys::vec3 & pend)
	{
		JPHYS_FLOAT_UNIT Asqrd = getDistSquared(pstart, pmiddle);
		JPHYS_FLOAT_UNIT Bsqrd = getDistSquared(pmiddle, pend);
		JPHYS_FLOAT_UNIT Csqrd = getDistSquared(pstart, pend);

		return (Asqrd + Bsqrd - Csqrd) / (2 * sqrt(Asqrd) * sqrt(Bsqrd));
	}
}
}
