/*
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: The MIT License (MIT) (Expat)
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "Templates.h"
#include "AdvancedMiscMath.h"
#include "mathTools.h"


#define TRY_ADDING_FASTER_WAY 0


namespace phys
{
	using namespace mathTools;

	
	void ComplexPhasor::SetToCartesianFormat(JPHYS_FLOAT_UNIT x, JPHYS_FLOAT_UNIT y) //x+iy
	{
		A = sqrt(x*x + y*y);

		theta = atan2(y,x);
	}
	

	ComplexPhasor& ComplexPhasor::operator+=(const ComplexPhasor &rhs)
	{
#if TRY_ADDING_FASTER_WAY
		A		= sqrt(A*A + rhs.A*rhs.A - 2.0*A*rhs.A*cos(theta - rhs.theta));
		theta	= //(A*sin(theta) + rhs.A*sin(rhs.theta));
#else
		JPHYS_FLOAT_UNIT t_x = (   X() + rhs.X()   );
		JPHYS_FLOAT_UNIT t_y = (   Y() + rhs.Y()   );

		A = sqrt(t_x*t_x + t_y*t_y);
		theta = atan2(t_y, t_x);
#endif
		return *this;
	}
	ComplexPhasor& ComplexPhasor::operator-=(const ComplexPhasor &rhs)
	{
#if TRY_ADDING_FASTER_WAY
		A		= sqrt(A*A + rhs.A*rhs.A + 2.0*A*rhs.A*cos(theta - rhs.theta));
		theta	= (A*sin(theta) - rhs.A*sin(rhs.theta));
#else
		JPHYS_FLOAT_UNIT t_x = (   X() - rhs.X()   );
		JPHYS_FLOAT_UNIT t_y = (   Y() - rhs.Y()   );

		A = sqrt(t_x*t_x + t_y*t_y);
		theta = atan2(t_y, t_x);
#endif
		return *this;
	}
	ComplexPhasor& ComplexPhasor::operator*=(const ComplexPhasor &rhs)
	{
		A *= rhs.A;
		theta += rhs.theta;
		return *this;
	}
	ComplexPhasor& ComplexPhasor::operator/=(const ComplexPhasor &rhs)
	{
		A /= rhs.A;
		theta -= rhs.theta;
		return *this;
	}

//--------------
	
	ComplexPhasor& ComplexPhasor::operator+=(const JPHYS_FLOAT_UNIT &rhs)
	{
#if TRY_ADDING_FASTER_WAY
		A		= (A*cos(theta) + rhs);
		theta	= (A*sin(theta));
#else
		JPHYS_FLOAT_UNIT t_x = (   X() + rhs   );
		JPHYS_FLOAT_UNIT t_y = (   Y()         );

		A = sqrt(t_x*t_x + t_y*t_y);
		theta = atan2(t_y, t_x);
#endif

		return *this;
	}
	ComplexPhasor& ComplexPhasor::operator-=(const JPHYS_FLOAT_UNIT &rhs)
	{
#if TRY_ADDING_FASTER_WAY
		A		= (A*cos(theta) - rhs);
		theta	= (A*sin(theta));
#else
		JPHYS_FLOAT_UNIT t_x = (   X() + rhs   );
		JPHYS_FLOAT_UNIT t_y = (   Y()         );

		A = sqrt(t_x*t_x + t_y*t_y);
		theta = atan2(t_y, t_x);
#endif
		return *this;
	}
	ComplexPhasor& ComplexPhasor::operator*=(const JPHYS_FLOAT_UNIT &rhs)
	{
		A *= rhs;
		return *this;
	}
	ComplexPhasor& ComplexPhasor::operator/=(const JPHYS_FLOAT_UNIT &rhs)
	{
		A /= rhs;
		return *this;
	}

//=========================================================================================================================

namespace mathToolsAdvanced
{

	//these correspond to a Fourier Transform of the form:  Integrate[ f(t)*e^(i*w*t) ... dt ] ... from -infinity to infinity

#if 0
ComplexPhasor DiscretizedFourierTransform(const std::vector<vec2>& input_data__x_is_time__y_is_f_of_t, JPHYS_FLOAT_UNIT omega)
{
	ComplexPhasor retval;

	if(input_data__x_is_time__y_is_f_of_t.size() > 1)
	{
		//inpit->x == time.... inpit->y == data

		std::vector<vec2>::const_iterator inp, inp_next;
		inp_next = inp = input_data__x_is_time__y_is_f_of_t.cbegin();
		inp_next++;

		while(inp_next != input_data__x_is_time__y_is_f_of_t.cend())
		{
			retval += (
				(ComplexPhasor(inp_next->y, omega*inp_next->x) + ComplexPhasor(inp->y, omega*inp->x))
			* (inp_next->x - inp->x)
				);

			inp = inp_next;
			inp_next++;
		}
	}
	return (retval * 0.5);
}
#endif

ComplexPhasor DiscretizedFourierTransform(const std::deque <vec2>& input_data__x_is_time__y_is_f_of_t, JPHYS_FLOAT_UNIT omega)
{
	ComplexPhasor retval;

	if(input_data__x_is_time__y_is_f_of_t.size() > 1)
	{
		//inpit->x == time.... inpit->y == data

		std::deque<vec2>::const_iterator inp, inp_next;
		inp_next = inp = input_data__x_is_time__y_is_f_of_t.cbegin();
		inp_next++;

		while(inp_next != input_data__x_is_time__y_is_f_of_t.cend())
		{
			retval += (
				(ComplexPhasor(inp_next->y, omega*inp_next->x) + ComplexPhasor(inp->y, omega*inp->x))
			* (inp_next->x - inp->x)
				);

			inp = inp_next;
			inp_next++;
		}
	}
	return (retval * 0.5);
}


std::deque<ComplexPhasor> DiscreteFourierTransform(const std::deque<ComplexPhasor>& input_x)
{
	int N = input_x.size();
	double N_dbl = static_cast<double>(N);
	std::deque<ComplexPhasor> retval(N);

	if(N > 1)
	{
		int n=0;
		int k=0;

		for(k=0; k<N; k++) //k from 0 to N-1
		{
			retval[k].Nullify();

			for(n=0; n<N; n++) //n from 0 to N-1
			{
				retval[k] += (input_x[n] * phys::ComplexPhasor(1.0, TWO_PI * static_cast<double>(n*k) / N_dbl));
			}
		}
	}

	return retval;
}

std::deque<ComplexPhasor> InverseDiscreteFourierTransform(const std::deque<ComplexPhasor>& input_x)
{
	int N = input_x.size();
	double N_dbl = static_cast<double>(N);
	std::deque<ComplexPhasor> retval(N);

	if(N > 1)
	{
		int n=0;
		int k=0;

		for(k=0; k<N; k++) //k from 0 to N-1
		{
			retval[k].Nullify();

			for(n=0; n<N; n++) //n from 0 to N-1
			{
				retval[k] += (input_x[n] * phys::ComplexPhasor(1.0, NEGATIVE_TWO_PI * static_cast<double>(n*k) / N_dbl));
			}
			
			retval[k] /= N_dbl;
		}
	}

	return retval;
}


//===========================================================================================================================


std::deque<std::deque<ComplexPhasor>> DiscreteFourierTransform__2D(const std::deque<std::deque<ComplexPhasor>>& input_xy)
{
	int N_y = input_xy.size();
	int N_x = input_xy[0].size();
	double N_y_dbl = static_cast<double>(N_y);
	double N_x_dbl = static_cast<double>(N_x);

	std::deque<std::deque<ComplexPhasor>> retval(N_y, std::deque<ComplexPhasor>(N_x, 0.0));

	if(N_x > 1 && N_y > 1)
	{
		int n_y=0;
		int k_y=0;
		
		int n_x=0;
		int k_x=0;

		for(k_y=0; k_y<N_y; k_y++) //ky from 0 to Ny-1
		{
			for(k_x=0; k_x<N_x; k_x++) //kx from 0 to Nx-1
			{
				retval[k_y][k_x].Nullify();

//----------------------------------------------------------
				for(n_y=0; n_y<N_y; n_y++) //ny from 0 to Ny-1
				{
					for(n_x=0; n_x<N_x; n_x++) //nx from 0 to Nx-1
					{
						retval[k_y][k_x] += (input_xy[n_y][n_x] * phys::ComplexPhasor(1.0, TWO_PI *

							((static_cast<double>(n_y*k_y)/N_y_dbl) + (static_cast<double>(n_x*k_x)/N_x_dbl))
							
							));
					}
				}
//----------------------------------------------------------
			}
		}
	}

	return retval;
}


std::deque<std::deque<ComplexPhasor>> InverseDiscreteFourierTransform__2D(const std::deque<std::deque<ComplexPhasor>>& input_xy)
{
	int N_y = input_xy.size();
	int N_x = input_xy[0].size();
	double N_y_dbl = static_cast<double>(N_y);
	double N_x_dbl = static_cast<double>(N_x);

	std::deque<std::deque<ComplexPhasor>> retval(N_y, std::deque<ComplexPhasor>(N_x, 0.0));

	if(N_x > 1 && N_y > 1)
	{
		int n_y=0;
		int k_y=0;
		
		int n_x=0;
		int k_x=0;

		for(k_y=0; k_y<N_y; k_y++) //ky from 0 to Ny-1
		{
			for(k_x=0; k_x<N_x; k_x++) //kx from 0 to Nx-1
			{
				retval[k_y][k_x].Nullify();

//----------------------------------------------------------
				for(n_y=0; n_y<N_y; n_y++) //ny from 0 to Ny-1
				{
					for(n_x=0; n_x<N_x; n_x++) //nx from 0 to Nx-1
					{
						retval[k_y][k_x] += (input_xy[n_y][n_x] * phys::ComplexPhasor(1.0, NEGATIVE_TWO_PI *

							((static_cast<double>(n_y*k_y)/N_y_dbl) + (static_cast<double>(n_x*k_x)/N_x_dbl))
							
							));
					}
				}
//----------------------------------------------------------
				
				retval[k_y][k_x] /= (N_x_dbl * N_y_dbl);
			}
		}
	}

	return retval;
}

//###################################################################################################################
//###################################################################################################################

std::vector<std::vector<JPHYS_FLOAT_UNIT>> DiscreteCosineTransform__2D(const std::vector<std::vector<JPHYS_FLOAT_UNIT>>& input_xy)
{
	int N_y = input_xy.size();
	int N_x = input_xy[0].size();
	double N_y_dbl = static_cast<double>(N_y);
	double N_x_dbl = static_cast<double>(N_x);
	double N_y_dbl_littleless = (N_y_dbl - 0.001); //just for comparing ints; 2.000000 ~= 1.9999999 is still definitely > 1.999
	double N_x_dbl_littleless = (N_x_dbl - 0.001);

	std::vector<std::vector<JPHYS_FLOAT_UNIT>> retval(N_y, std::vector<JPHYS_FLOAT_UNIT>(N_x, 0.0));

	if(N_x > 1 && N_y > 1)
	{
		double n_y=0.0;
		double k_y=0.0;
		
		double n_x=0.0;
		double k_x=0.0;
		
		double current_row_cosinefactor = 0.0;

		for(k_y=0.0; k_y<N_y_dbl_littleless; k_y++) //ky from 0 to Ny-1
		{
			for(k_x=0.0; k_x<N_x_dbl_littleless; k_x++) //kx from 0 to Nx-1
			{
				retval[k_y][k_x] = 0.0;

//----------------------------------------------------------
				for(n_y=0.0; n_y<N_y_dbl_littleless; n_y++) //ny from 0 to Ny-1
				{
					//current_row_cosinefactor = 0.0;

					for(n_x=0.0; n_x<N_x_dbl_littleless; n_x++) //nx from 0 to Nx-1
					{
						//current_row_cosinefactor += (input_xy[n_y][n_x]

						retval[k_y][k_x] += (input_xy[n_y][n_x]
													* cos(phys::mathTools::PI*(n_y+0.5)*k_y/N_y_dbl)
													* cos(phys::mathTools::PI*(n_x+0.5)*k_x/N_x_dbl)
											);
					}
				}
//----------------------------------------------------------
			}
		}
	}

	return retval;
}


std::vector<std::vector<JPHYS_FLOAT_UNIT>> InverseDiscreteCosineTransform__2D(const std::vector<std::vector<JPHYS_FLOAT_UNIT>>& input_xy)
{
	int N_y = input_xy.size();
	int N_x = input_xy[0].size();
	double N_y_dbl = static_cast<double>(N_y);
	double N_x_dbl = static_cast<double>(N_x);
	double N_y_dbl_littleless = (N_y_dbl - 0.001); //just for comparing ints; 2.000000 ~= 1.9999999 is still definitely > 1.999
	double N_x_dbl_littleless = (N_x_dbl - 0.001);

	std::vector<std::vector<JPHYS_FLOAT_UNIT>> retval(N_y, std::vector<JPHYS_FLOAT_UNIT>(N_x, 0.0));

	if(N_x > 1 && N_y > 1)
	{
		double n_y=0.0;
		double k_y=0.0;
		
		double n_x=0.0;
		double k_x=0.0;

		double current_row_cosinefactor = 0.0;

		for(k_y=0.0; k_y<N_y_dbl_littleless; k_y++) //ky from 0 to Ny-1
		{
			for(k_x=0.0; k_x<N_x_dbl_littleless; k_x++) //kx from 0 to Nx-1
			{
				retval[k_y][k_x] = 0.0;

//----------------------------------------------------------
				for(n_y=0.0; n_y<N_y_dbl_littleless; n_y++) //ny from 0 to Ny-1
				{
					for(n_x=0.0; n_x<N_x_dbl_littleless; n_x++) //nx from 0 to Nx-1
					{
						retval[k_y][k_x] += (input_xy[n_y][n_x]
													* cos(phys::mathTools::PI*n_y*(k_y+0.5)/N_y_dbl)
													* cos(phys::mathTools::PI*n_x*(k_x+0.5)/N_x_dbl)
											);
					}
				}
//----------------------------------------------------------
				
				retval[k_y][k_x] *= (4.0 / (N_x_dbl * N_y_dbl));
			}
		}
	}

	return retval;
}


//##################################################################################################################

std::vector<std::vector<JPHYS_FLOAT_UNIT>> DiscreteCosineTransform__2D_SYMMETRIC(const std::vector<std::vector<JPHYS_FLOAT_UNIT>>& input_xy)
{
	int N_y = input_xy.size();
	int N_x = input_xy[0].size();
	double N_y_dbl = static_cast<double>(N_y);
	double N_x_dbl = static_cast<double>(N_x);
	double N_y_dbl_littleless = (N_y_dbl - 0.001); //just for comparing ints; 2.000000 ~= 1.9999999 is still definitely > 1.999
	double N_x_dbl_littleless = (N_x_dbl - 0.001);

	std::vector<std::vector<JPHYS_FLOAT_UNIT>> retval(N_y, std::vector<JPHYS_FLOAT_UNIT>(N_x, 0.0));

	if(N_x > 1 && N_y > 1)
	{
		double n_y=0.0;
		double k_y=0.0;
		
		double n_x=0.0;
		double k_x=0.0;
		
		double current_row_cosinefactor = 0.0;

		for(k_y=0.0; k_y<N_y_dbl_littleless; k_y++) //ky from 0 to Ny-1
		{
			for(k_x=0.0; k_x<N_x_dbl_littleless; k_x++) //kx from 0 to Nx-1
			{
				//retval[k_y][k_x] = 0.0;

//----------------------------------------------------------
				for(n_y=0.0; n_y<N_y_dbl_littleless; n_y++) //ny from 0 to Ny-1
				{
					for(n_x=0.0; n_x<N_x_dbl_littleless; n_x++) //nx from 0 to Nx-1
					{
						retval[k_y][k_x] += (input_xy[n_y][n_x]
													* cos(phys::mathTools::PI*(n_y+0.5)*(k_y+0.5)/N_y_dbl)
													* cos(phys::mathTools::PI*(n_x+0.5)*(k_x+0.5)/N_x_dbl)
											);
					}
				}
//----------------------------------------------------------

				retval[k_y][k_x] *= sqrt(4.0 / (N_x_dbl*N_y_dbl));
			}
		}
	}

	return retval;
}




}
}
