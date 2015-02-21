#ifndef ___JPHYSICS_FINITE_DIFFERENCE_METHOD_H____
#define ___JPHYSICS_FINITE_DIFFERENCE_METHOD_H____

#include <deque>
#include "FloatUnits.h"


namespace phys
{
	class FiniteDifferenceSolver_1D_HeatEquation
	{
	protected:
		int num_grid_points;
		int last_grid_point;
		JPHYS_FLOAT_UNIT delta_x;
		JPHYS_FLOAT_UNIT delta_x_squared;
		
		JPHYS_FLOAT_UNIT L;
		std::deque< std::deque<JPHYS_FLOAT_UNIT> > grid; //1D space, 1D time

	public:
		JPHYS_FLOAT_UNIT chi;


		int GetNumGridPoints() const {return num_grid_points;}
		JPHYS_FLOAT_UNIT GetGridValue(int m, int n) const {return grid[n][m];}
		JPHYS_FLOAT_UNIT GetLatestGridValues(int m) const {return (*grid.rbegin())[m];}

		JPHYS_FLOAT_UNIT Convert_m_To_x(int m) const {return (static_cast<JPHYS_FLOAT_UNIT>(m) * delta_x);}

		//--------------
		
		void InitializeGrid2(JPHYS_FLOAT_UNIT xmax_equals_L, JPHYS_FLOAT_UNIT deltax);
		void InitializeGrid(JPHYS_FLOAT_UNIT xmax_equals_L, int NUM_GRID_POINTS);


		virtual JPHYS_FLOAT_UNIT Func_InitialCondition(int m);

		virtual JPHYS_FLOAT_UNIT Func_Source(int m, int n);
		
		virtual bool PointIsABoundaryCondition(int m);
		virtual JPHYS_FLOAT_UNIT Func_BoundaryConditions(int m, int n);
		
		virtual JPHYS_FLOAT_UNIT SpaceSecondDerivative(int m, int n);
		virtual JPHYS_FLOAT_UNIT SpaceCalcFunc(int m, int n);

		void UpdateGrid(JPHYS_FLOAT_UNIT delta_t);


		FiniteDifferenceSolver_1D_HeatEquation();
	};
}

#endif
