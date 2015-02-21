
//extern JPHYSICS_API Hit WINAPI getHit( line, line );    //FLOATING POINT

#include "exports.h"
#include "Templates.h"
#include "EntityEdgeLine.h"
#include "Hit.h"

namespace phys
{
namespace cTools
{
	//The "distance" return value of Hit assumes that LINE2 represents the start and endpoints of the motion of a particle;
	//then Hit.dist is the distance it travels to reach the hit position.
	//This allows sorting multiple hits for the one that occurs first along a trajectory.
	JPHYSICS_API Hit getHit( const line& LINE1, const line& LINE2 );
	
	JPHYSICS_API Hit getHitOrDist( const line& LINE1, const line& LINE2 );
	JPHYSICS_API Hit getHitOrDist( const line& LINE1, const EntityEdgeLine& EDGELINE );
	

	
	//faster (no sqrt)
	JPHYSICS_API JPHYS_FLOAT_UNIT getDistSquared( JPHYS_FLOAT_UNIT x1, JPHYS_FLOAT_UNIT y1, JPHYS_FLOAT_UNIT x2, JPHYS_FLOAT_UNIT y2 );
	JPHYSICS_API JPHYS_FLOAT_UNIT getDistSquared( const point& p1, const point& p2 );
	JPHYSICS_API JPHYS_FLOAT_UNIT getDistSquared( const vec3& p1, const vec3& p2 );


	JPHYSICS_API JPHYS_FLOAT_UNIT getDist( const vec3& p1, const vec3& p2 );
	JPHYSICS_API JPHYS_FLOAT_UNIT getDist( const point& p1, const point& p2 );
	JPHYSICS_API JPHYS_FLOAT_UNIT getDistToInfiniteLine( const point& pP, const line& LINE);
	JPHYSICS_API JPHYS_FLOAT_UNIT getDist( const point& pP, const line& LINE);
	JPHYSICS_API JPHYS_FLOAT_UNIT getDist( const point& pP, const point& LINE_P1, const point& LINE_P2);

	JPHYSICS_API		Hit nearestHitAlongLine( const point& pP, const line& LINE);

	JPHYSICS_API void collisionRotation(point& velocity, const line& surface);

	//the rectangle is defined by two corners, start and end
	JPHYSICS_API bool checkPointInRectangle(const point &a, const point &start, const point &end);

	JPHYSICS_API bool checkIntersect( const line& LINE1, const line& LINE2 );
	
	JPHYSICS_API bool checkPointIsToTheLeftOfLine(const point& pCheck, const point& LINE_P1, const point& LINE_P2);
	JPHYSICS_API bool checkPointIsToTheLeftOfLine(const point& pCheck, const line& LINE);
	
	JPHYSICS_API bool checkPointIsToTheRightOfLine(const point& pCheck, const point& LINE_P1, const point& LINE_P2);
	JPHYSICS_API bool checkPointIsToTheRightOfLine(const point& pCheck, const line& LINE);
	
		//two line segments that meet in the middle.... forming a triangle
	JPHYSICS_API JPHYS_FLOAT_UNIT CosineBetweenTwoLineSegments(const phys::vec3 & pstart, const phys::vec3 & pmiddle, const phys::vec3 & pend);
}
}
