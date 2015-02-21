#ifndef ___JPHYSICS__ADVANCED_MISC_MATH_H______
#define ___JPHYSICS__ADVANCED_MISC_MATH_H______

#include <vector>
#include <deque>


namespace phys
{
	//lets you write (A*Exp(i*theta)) in terms of A and theta, so you don't have to deal with the imaginary number

	class JPHYSICS_API ComplexPhasor
	{
	public:
		JPHYS_FLOAT_UNIT	A,theta;
	
		ComplexPhasor() : A(0.0), theta(0.0) {}
		ComplexPhasor(JPHYS_FLOAT_UNIT real_value) : A(real_value), theta(0.0) {}
		ComplexPhasor(JPHYS_FLOAT_UNIT amplitude, JPHYS_FLOAT_UNIT angletheta) : A(amplitude), theta(angletheta) {}

		void SetToCartesianFormat(JPHYS_FLOAT_UNIT x, JPHYS_FLOAT_UNIT y); //x+iy
		void Nullify() {A=0.0; theta=0.0;}

		INLINE JPHYS_FLOAT_UNIT X()		const {return A*cos(theta);}
		INLINE JPHYS_FLOAT_UNIT Re()	const {return A*cos(theta);}

		INLINE JPHYS_FLOAT_UNIT Y()		const {return A*sin(theta);}
		INLINE JPHYS_FLOAT_UNIT Im()	const {return A*sin(theta);}
	
		ComplexPhasor& operator+=(const ComplexPhasor &rhs);
		ComplexPhasor& operator-=(const ComplexPhasor &rhs);
		ComplexPhasor& operator*=(const ComplexPhasor &rhs);
		ComplexPhasor& operator/=(const ComplexPhasor &rhs);
		
		ComplexPhasor& operator+=(const JPHYS_FLOAT_UNIT &rhs);
		ComplexPhasor& operator-=(const JPHYS_FLOAT_UNIT &rhs);
		ComplexPhasor& operator*=(const JPHYS_FLOAT_UNIT &rhs);
		ComplexPhasor& operator/=(const JPHYS_FLOAT_UNIT &rhs);
		
		const ComplexPhasor operator+(const ComplexPhasor &other) const
		{
			return ComplexPhasor(*this) += other;
		}
		const ComplexPhasor operator-(const ComplexPhasor &other) const
		{
			return ComplexPhasor(*this) -= other;
		}
		const ComplexPhasor operator*(const ComplexPhasor &other) const
		{
			return ComplexPhasor(*this) *= other;
		}
		const ComplexPhasor operator/(const ComplexPhasor &other) const
		{
			return ComplexPhasor(*this) /= other;
		}
		
		const ComplexPhasor operator+(const JPHYS_FLOAT_UNIT &other) const
		{
			return ComplexPhasor(*this) += other;
		}
		const ComplexPhasor operator-(const JPHYS_FLOAT_UNIT &other) const
		{
			return ComplexPhasor(*this) -= other;
		}
		const ComplexPhasor operator*(const JPHYS_FLOAT_UNIT &other) const
		{
			return ComplexPhasor(*this) *= other;
		}
		const ComplexPhasor operator/(const JPHYS_FLOAT_UNIT &other) const
		{
			return ComplexPhasor(*this) /= other;
		}
	};

//=========================================================================================================================

namespace mathToolsAdvanced
{

	//JPHYSICS_API ComplexPhasor DiscretizedFourierTransform(const std::vector<vec2>& input_data__x_is_time__y_is_f_of_t, JPHYS_FLOAT_UNIT desired_omega);
	JPHYSICS_API ComplexPhasor DiscretizedFourierTransform(const std::deque <vec2>& input_data__x_is_time__y_is_f_of_t, JPHYS_FLOAT_UNIT desired_omega);

	
	JPHYSICS_API std::deque<ComplexPhasor> DiscreteFourierTransform(const std::deque<ComplexPhasor>& input_x);
	JPHYSICS_API std::deque<ComplexPhasor> InverseDiscreteFourierTransform(const std::deque<ComplexPhasor>& input_x);

	
	JPHYSICS_API std::deque< std::deque<ComplexPhasor> > DiscreteFourierTransform__2D(const std::deque< std::deque<ComplexPhasor> >& input_xy);
	JPHYSICS_API std::deque< std::deque<ComplexPhasor> > InverseDiscreteFourierTransform__2D(const std::deque< std::deque<ComplexPhasor> >& input_xy);


	JPHYSICS_API std::vector<std::vector<JPHYS_FLOAT_UNIT> > DiscreteCosineTransform__2D_SYMMETRIC(const std::vector< std::vector<JPHYS_FLOAT_UNIT> >& input_xy);

	
	JPHYSICS_API std::vector< std::vector<JPHYS_FLOAT_UNIT> > DiscreteCosineTransform__2D(const std::vector< std::vector<JPHYS_FLOAT_UNIT> >& input_xy);
	JPHYSICS_API std::vector< std::vector<JPHYS_FLOAT_UNIT> > InverseDiscreteCosineTransform__2D(const std::vector< std::vector<JPHYS_FLOAT_UNIT> >& input_xy);

	//JPHYSICS_API std::deque<ComplexPhasor> ConvolutionIntegral(const std::deque<ComplexPhasor>& input_x, const double integration_limits);
}
}
#endif
