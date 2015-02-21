
#ifndef __JPHYS_ENTITIES_H__
#define __JPHYS_ENTITIES_H__

#include "Templates.h"
#include "Games/EntityEdgeLine.h"
#include <vector>
#include <deque>

#include "exports.h"

namespace phys
{

	class EntitySystem; //to be defined later in compiliation

class I1DPhysStateObj
{
private:
	virtual void f( std::vector<double> *STATE, JPHYS_FLOAT_UNIT deltaT ) = 0;
	bool bVerlet;
	friend class integrate; //only integrators (during the integration process) are allowed to touch this
public:
	std::vector<double> state;
	I1DPhysStateObj() {bVerlet = false;}
	std::deque< std::vector<double> > FoPState;
};

class IPhysObject
{
private:
	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT ) = 0;

	bool bVerlet;

	friend class integrate; //only integrators (during the integration process) are allowed to touch this

public:
	std::vector<point> positions;
	std::vector<point> velocities;

	IPhysObject() {bVerlet = false;}
	
	std::deque< std::vector<point> > FoPPos;
	std::deque< std::vector<point> > FoPVel;
};

class I3DPhysObject
{
private:
	virtual void f( std::vector<vec3> *POS, std::vector<vec3> *VEL, JPHYS_FLOAT_UNIT deltaT ) = 0;

	bool bVerlet;	

	friend class integrate; //only integrators (during the integration process) are allowed to touch this

public:
	std::vector<vec3> positions;
	std::vector<vec3> velocities;

	I3DPhysObject() {bVerlet = false;}
	
	std::deque< std::vector<vec3> > FoPPos;
	std::deque< std::vector<vec3> > FoPVel;
};

class IEntity
{
public:
	//JPHYS_FLOAT_UNIT lifeTimer;
	
	std::vector<EntityEdgeLine> DangerLines;

	std::vector<point> positions;
	std::vector<point> velocities;
	JPHYS_FLOAT_UNIT friction;

	unsigned char color[3];
	
	EntitySystem* myEntSystem;	//they hold a pointer to their own system

	point CenterPos;

	//updates positions and velocities
	JPHYSICS_API virtual void update(JPHYS_FLOAT_UNIT frametime) = 0;

	//draws object using OpenGl
	JPHYSICS_API virtual void draw() = 0;

	//draws positions with an offset and stretch. since it has to calculate the offset each time, this is less efficient.
	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch) = 0;

	//applies an impulse to any nearby objects (e.g. for mouse/finger touches)
	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse) = 0;
	
	//Check collisions, esp. against another moving object nearby
	JPHYSICS_API virtual void checkCollisions(const line &hitSurface, const point& offset) = 0;

	IEntity() : myEntSystem(NULL) {color[0]=255; color[1]=0; color[2]=0;}
};

class IEntityOld : public IPhysObject
{
protected:
	bool i_have_finished_my_drawing_of_my_path;
	friend class EntitySystem;
public:
	JPHYS_FLOAT_UNIT lifeTimer;
	unsigned char color[3];
	bool delete_when_outside_screen;
	
	EntitySystem* myEntSystem;	//they hold a pointer to their own system

	point CenterPos;

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime) = 0;
	JPHYSICS_API virtual void draw() = 0;
	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch) = 0;
	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse) = 0;

	IEntityOld() : myEntSystem(NULL), delete_when_outside_screen(true) {lifeTimer=0.0; color[0]=255; color[1]=0; color[2]=0; i_have_finished_my_drawing_of_my_path=false;}
};

class I3DEntityOld : public I3DPhysObject
{
protected:
	bool i_have_finished_my_drawing_of_my_path;
	friend class EntitySystem;
public:
	JPHYS_FLOAT_UNIT lifeTimer;
	unsigned char color[3];
	bool delete_when_outside_screen;
	
	EntitySystem* myEntSystem;	//they hold a pointer to their own system

	vec3 CenterPos;

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime) = 0;
	JPHYSICS_API virtual void draw() = 0;
	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch) = 0;
	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse) = 0;

	I3DEntityOld() : myEntSystem(NULL), delete_when_outside_screen(true) {lifeTimer=0.0; color[0]=255; color[1]=0; color[2]=0; i_have_finished_my_drawing_of_my_path=false;}
};

class JPHYSICS_API SpawnParams
{
public:
	JPHYS_FLOAT_UNIT air_resistance;
	JPHYS_FLOAT_UNIT mass_total;
	unsigned char color[3];

	point pos_center;
	point velocity;

	SpawnParams() : air_resistance(0.0), mass_total(1.0) {color[0]=255; color[1]=0; color[2]=0;}
};

class JPHYSICS_API SpawnParams_Fluid
{
public:
	JPHYS_FLOAT_UNIT friction;
	JPHYS_FLOAT_UNIT kCohesion;
	JPHYS_FLOAT_UNIT kRepulsion;

	JPHYS_FLOAT_UNIT distanceFactor;

	JPHYS_FLOAT_UNIT mass_each;
	unsigned char color[3];

	SpawnParams_Fluid() : friction(0.0), mass_each(1.0), kCohesion(1.0), kRepulsion(1.0) {color[0]=255; color[1]=0; color[2]=0;}
};

class JPHYSICS_API SpawnParams_Soft
{
public:

	SpawnParams entParams;

	JPHYS_FLOAT_UNIT vertex_equilibrium_distance;
	JPHYS_FLOAT_UNIT vertex_equilibrium_angle;
	JPHYS_FLOAT_UNIT springconst_linear;
	JPHYS_FLOAT_UNIT springconst_angular;

	SpawnParams_Soft() : vertex_equilibrium_distance(1.0), vertex_equilibrium_angle(90.0), springconst_linear(0.0), springconst_angular(0.0) {}
};

class JPHYSICS_API SpawnParams_3D
{
public:
	JPHYS_FLOAT_UNIT air_resistance;
	JPHYS_FLOAT_UNIT mass_total;
	unsigned char color[3];

	vec3 pos_center;
	vec3 velocity;

	SpawnParams_3D() : air_resistance(0.0), mass_total(1.0) {color[0]=255; color[1]=0; color[2]=0;}
};

}

#endif
