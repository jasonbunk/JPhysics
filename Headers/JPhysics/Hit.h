//-----------------------------------------------------------------
// July 27, 2012
// Jason Bunk
// Hits.h
//
// Hit class, used & returned by various collision functions.
//-----------------------------------------------------------------

#ifndef __JPHYSICS_HIT_H__
#define __JPHYSICS_HIT_H__

#include "exports.h"
#include "Templates.h"

namespace phys
{

class Hit
{
	public:
		JPHYS_FLOAT_UNIT dist;
		point pt;
		bool b;

		inline operator bool() {return b;}

		//allows stl sorting algorithms for comparing different hits (to choose the first hit along a trajectory)
		inline bool operator< (const Hit &rhs) const {return ((*this).dist < rhs.dist);}

		inline bool operator<= (const Hit &rhs) const {return ((*this).dist <= rhs.dist);} //not used for stl but here for consistency
		inline bool operator>= (const Hit &rhs) const {return ((*this).dist >= rhs.dist);}
		inline bool operator>  (const Hit &rhs) const {return ((*this).dist >  rhs.dist);}


		//constructors
		Hit(bool B, JPHYS_FLOAT_UNIT D=0, point P=point(0,0))
		{
			pt = P;
			dist = D;
			b = B;
		}
		Hit()
		{
			//pt = point(0,0);
			dist = 0.0;
			b = false;
		}
};

}

#endif
