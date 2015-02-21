#ifndef __JPHYSICS_DOT_H__
#define __JPHYSICS_DOT_H__

#include "exports.h"
#include "Templates.h"
#include "Entities.h"
#include "EntitySystem.h"
#include <vector>


namespace phys
{

class ent_dot : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
public:

	JPHYS_FLOAT_UNIT mass;
	JPHYS_FLOAT_UNIT kvel;

	JPHYSICS_API ent_dot();

	JPHYSICS_API ent_dot(SpawnParams params);

//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();

	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );

};

//==========================================================================

class ent3d_dot : public I3DEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
public:

	JPHYS_FLOAT_UNIT mass;
	JPHYS_FLOAT_UNIT kvel;

	JPHYSICS_API ent3d_dot();

	JPHYSICS_API ent3d_dot(SpawnParams_3D params);

//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();

	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<vec3> *POS, std::vector<vec3> *VEL, JPHYS_FLOAT_UNIT deltaT );

};



class ent3d_dotsystem : public I3DEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
protected:
	std::vector<JPHYS_FLOAT_UNIT> masses;
	std::vector<JPHYS_FLOAT_UNIT> kvel;


	std::vector< std::vector<unsigned char> > colors_vec;

public:

	JPHYSICS_API ent3d_dotsystem();

	JPHYSICS_API ent3d_dotsystem(SpawnParams_3D params);


	JPHYSICS_API void add_point_particle(vec3 new_position, vec3 new_velocity, JPHYS_FLOAT_UNIT new_mass, JPHYS_FLOAT_UNIT new_air_resistance_constant=0.0, std::vector<unsigned char> new_colors = std::vector<unsigned char>());
	JPHYSICS_API void add_point_particle(JPHYS_FLOAT_UNIT new_mass, vec3 new_position, vec3 new_velocity, JPHYS_FLOAT_UNIT new_air_resistance_constant=0.0, std::vector<unsigned char> new_colors = std::vector<unsigned char>());

//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();

	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<vec3> *POS, std::vector<vec3> *VEL, JPHYS_FLOAT_UNIT deltaT );

};



}

#endif
