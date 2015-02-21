#include "stdafx.h"
#include "FiniteDifferenceMethod.h"
#include "mathTools.h"


namespace phys
{

	FiniteDifferenceSolver_1D_HeatEquation::FiniteDifferenceSolver_1D_HeatEquation() : num_grid_points(0), last_grid_point(0), delta_x_squared(0.1), chi(1.0), L(1.0) {}
	
	void FiniteDifferenceSolver_1D_HeatEquation::InitializeGrid2(JPHYS_FLOAT_UNIT xmax_equals_L, JPHYS_FLOAT_UNIT deltax)
	{
		int num_grid_points_rounded = mathTools::RoundDoubleToInt(xmax_equals_L / deltax);
		InitializeGrid(xmax_equals_L, num_grid_points_rounded + 1);
	}

	void FiniteDifferenceSolver_1D_HeatEquation::InitializeGrid(JPHYS_FLOAT_UNIT xmax_equals_L, int NUM_GRID_POINTS)
	{
		num_grid_points = NUM_GRID_POINTS;
		last_grid_point = (NUM_GRID_POINTS - 1);

		delta_x = (xmax_equals_L / static_cast<JPHYS_FLOAT_UNIT>(last_grid_point));
		delta_x_squared = (delta_x * delta_x);

		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));
		for(int m=0; m <= last_grid_point; m++)
		{
			grid[0][m] = Func_InitialCondition(m);
		}
	}

//=============================================================================================

	JPHYS_FLOAT_UNIT FiniteDifferenceSolver_1D_HeatEquation::Func_InitialCondition(int m)
	{
		/*
		if(m == (num_grid_points / 2))
			return 10.0;
		return 0.0;
		*/

		return fabs(20.0*sin((static_cast<JPHYS_FLOAT_UNIT>(m)/static_cast<JPHYS_FLOAT_UNIT>(last_grid_point)) * (mathTools::PI * 4.0 / L)));
	}

	JPHYS_FLOAT_UNIT FiniteDifferenceSolver_1D_HeatEquation::Func_Source(int m, int n)
	{
		return 0.0;
	}

	bool FiniteDifferenceSolver_1D_HeatEquation::PointIsABoundaryCondition(int m)
	{
		return (m==0 || m==last_grid_point);
	}

	JPHYS_FLOAT_UNIT FiniteDifferenceSolver_1D_HeatEquation::Func_BoundaryConditions(int m, int n)
	{
		if(m == 0)				{return 0.0;}
		if(m == last_grid_point){return 0.0;}

		return 1.0e9;
	}
	
//=============================================================================================

	JPHYS_FLOAT_UNIT FiniteDifferenceSolver_1D_HeatEquation::SpaceSecondDerivative(int m, int n)
	{
		return (1.0 / delta_x_squared)*(

			grid[n][m-1] - (2.0 * grid[n][m]) + grid[n][m+1]
		);
	}

	JPHYS_FLOAT_UNIT FiniteDifferenceSolver_1D_HeatEquation::SpaceCalcFunc(int m, int n)
	{
		if(PointIsABoundaryCondition(m))
			return Func_BoundaryConditions(m,n);

		return Func_Source(m,n) + (chi * SpaceSecondDerivative(m,n));
	}

	void FiniteDifferenceSolver_1D_HeatEquation::UpdateGrid(JPHYS_FLOAT_UNIT delta_t)
	{
		int m=0;

#if 0
		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));

		int n = (grid.size() - 1);

		for(int m=0; m <= last_grid_point; m++)
		{
			grid[n][m] = grid[n-1][m] + (SpaceCalcFunc(m,n-1) * delta_t);
		}

#elif THIS_METHOD_SEEMS_TO_HAVE_150pct_BETTER_STABILITY_THAN_PLAIN_EULER

		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));
		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));

		int n0 = (grid.size() - 3); //the data from last frame == n0
		int n1 = (grid.size() - 2);
		int n2 = (grid.size() - 1); //temporarily used; will be pop_back()'d
		
		for(m=0; m <= last_grid_point; m++) //predictor
		{
			grid[n1][m] = grid[n0][m] + (SpaceCalcFunc(m,n0) * delta_t);
		}
		
		for(m=0; m <= last_grid_point; m++) //corrector
		{
			grid[n2][m] = grid[n1][m] + (SpaceCalcFunc(m,n1) * delta_t);
		}
		
		for(m=0; m <= last_grid_point; m++) //now average the prediction and correction
		{
			grid[n1][m] = (0.5 * (grid[n1][m] + grid[n2][m]));
		}

		grid.pop_back();

#elif 1
		
		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));
		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));
		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));
		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));

		int n0 = (grid.size() - 5); //the data from last frame == n0
		int n1 = (grid.size() - 4);
		int n2 = (grid.size() - 3);
		int n3 = (grid.size() - 2);
		int n4 = (grid.size() - 1);
		
		for(m=0; m <= last_grid_point; m++) //predict based on the starting point (SPn0)
		{
			grid[n1][m] = grid[n0][m] + (SpaceCalcFunc(m,n0) * delta_t*0.5);

			//Tm,n1 == Tm,n0 + (SPno * dt/2) == Tm,n0 + (k1/2)
		}
		
		for(m=0; m <= last_grid_point; m++) //we went halfway, now we predict from halfway (SPn1)
		{
			grid[n2][m] = grid[n0][m] + (SpaceCalcFunc(m,n1) * delta_t*0.5);
			
			//Tm,n2 == Tm,n0 + (SPn1 * dt/2) == Tm,n0 + (k2/2)
		}
		
		for(m=0; m <= last_grid_point; m++) //predict from halfway again, based on previous prediction-from-halfway (SPn2)
		{
			grid[n3][m] = grid[n0][m] + (SpaceCalcFunc(m,n2) * delta_t);

			//Tm,n3 == Tm,n0 + (SPn2 * dt) == Tm,n0 + (k3)
		}
		
		for(m=0; m <= last_grid_point; m++) //use prediction from the end, based on the latest prediction-from-halfway (SPn3)
		{
			grid[n4][m] = (SpaceCalcFunc(m,n3) * delta_t);

			//Tm,n4 - Tm,n0 == (SPn3 * dt)
		}
		
		for(m=0; m <= last_grid_point; m++) //now add all predictions together using the method's weighted sum
		{
			grid[n1][m] = grid[n0][m] + ((1.0 / 6.0) * (

	-2.0*grid[n0][m] + 2.0*grid[n1][m] + 4.0*grid[n2][m] + 2.0*grid[n3][m] + grid[n4][m]
				
				));
		}

		grid.pop_back();
		grid.pop_back();
		grid.pop_back();

#elif THIS_METHOD_IS_NO_BETTER_THAN_THE_PLAIN_EULER_PREDICTOR

		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));
		grid.push_back(std::deque<JPHYS_FLOAT_UNIT>(num_grid_points, 0.0));

		int n0 = (grid.size() - 3); //the data from last frame == n0
		int n1 = (grid.size() - 2);
		int n2 = (grid.size() - 1); //temporarily used; will be pop_back()'d
		
		for(m=0; m <= last_grid_point; m++) //predictor
		{
			grid[n2][m] = grid[n0][m] + (SpaceCalcFunc(m,n0) * delta_t);
		}
		
		for(m=0; m <= last_grid_point; m++) //corrector
		{
			grid[n1][m] = grid[n2][m] + (SpaceCalcFunc(m,n2) * delta_t);
		}

		grid.pop_back();
#endif


		if(grid.size() > 5)
			grid.pop_front();
	}

}