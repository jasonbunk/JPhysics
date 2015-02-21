#include "stdafx.h"
#include "Templates.h"
#include "mathTools.h"
#include <string>
#include <iomanip>


namespace phys
{
		//normalized
		void point::GenerateRandomDirection()
		{
			x = mathTools::RandomF(-1.0,1.0);
			y = mathTools::RandomF(-1.0,1.0);

			JPHYS_FLOAT_UNIT LENG = getLength(); //normalize

			if(mathTools::cmpFloatToZero(LENG)) //unlikely, but check!
			{
					x = 0.0;
					y = 1.0;
			}
			else
			{
				x /= LENG;
				y /= LENG;
			}
		}
		point point::GetNormalized() const
		{
			JPHYS_FLOAT_UNIT LENG = getLength();

			if(mathTools::cmpFloatToZero(LENG)) //possible, so check!
				return phys::point(0.0, 1.0);
			else
				return ((*this) / LENG);
		}
		JPHYS_FLOAT_UNIT point::Normalize()
		{
			JPHYS_FLOAT_UNIT LENG = getLength();

			if(mathTools::cmpFloatToZero(LENG)) //possible, so check!
			{
					x = 0.0;
					y = 1.0;
			}
			else
			{
				x /= LENG;
				y /= LENG;
			}

			return LENG;
		}
		void point::LengthClamp(JPHYS_FLOAT_UNIT max_length)
		{
			JPHYS_FLOAT_UNIT LENG = getLength();

			if(mathTools::cmpFloatToZero(LENG) == false && LENG > max_length)
			{
				x *= (max_length/LENG);
				y *= (max_length/LENG);
			}
		}
		bool point::operator==(const point &other) const
		{
			return (mathTools::cmpFloats((*this).x, other.x) && mathTools::cmpFloats((*this).y, other.y));
		}
		bool point::operator!=(const point &other) const
		{
			return ((!mathTools::cmpFloats((*this).x, other.x)) || (!mathTools::cmpFloats((*this).y, other.y)));
		}

//===================================================================================
		//normalized
		void vec3::GenerateRandomDirection()
		{
			x = mathTools::RandomF(-1.0,1.0);
			y = mathTools::RandomF(-1.0,1.0);
			z = mathTools::RandomF(-1.0,1.0);

			JPHYS_FLOAT_UNIT LENG = getLength(); //normalize

			if(mathTools::cmpFloatToZero(LENG)) //unlikely, but check!
			{
				 x = 0.0;
				 y = 1.0;
				 z = 0.0;
			}
			else
			{
				x /= LENG;
				y /= LENG;
				z /= LENG;
			}
		}
		vec3 vec3::GetNormalized() const
		{
			JPHYS_FLOAT_UNIT LENG = getLength();

			if(mathTools::cmpFloatToZero(LENG)) //possible, so check!
				return phys::vec3(0.0, 1.0, 0.0);
			else
				return ((*this) / LENG);
		}

		JPHYS_FLOAT_UNIT vec3::Normalize()
		{
			JPHYS_FLOAT_UNIT LENG = getLength();

			if(mathTools::cmpFloatToZero(LENG)) //possible, so check!
			{
				 x = 0.0;
				 y = 1.0;
				 z = 0.0;
			}
			else
			{
				x /= LENG;
				y /= LENG;
				z /= LENG;
			}

			return LENG;
		}

		void vec3::LengthClamp(JPHYS_FLOAT_UNIT max_length)
		{
			JPHYS_FLOAT_UNIT LENG = getLength();

			if(mathTools::cmpFloatToZero(LENG) == false && LENG > max_length)
			{
				x *= (max_length/LENG);
				y *= (max_length/LENG);
				z *= (max_length/LENG);
			}
		}

		bool vec3::operator==(const vec3 &other) const
		{
			return (mathTools::cmpFloats((*this).x, other.x) && mathTools::cmpFloats((*this).y, other.y) && mathTools::cmpFloats((*this).z, other.z));
		}
		bool vec3::operator!=(const vec3 &other) const
		{
			return ((!mathTools::cmpFloats((*this).x, other.x)) || (!mathTools::cmpFloats((*this).y, other.y)) || (!mathTools::cmpFloats((*this).z, other.z)));
		}
//===================================================================================
//===================================================================================
		
		mat2x2::mat2x2()
		{
			m[0][0] = 0.0;
			m[1][0] = 0.0;
			m[0][1] = 0.0;
			m[1][1] = 0.0;
		}

		mat2x2::mat2x2(const mat2x2 & other)
		{
			m[0][0] = other.m[0][0];
			m[1][0] = other.m[1][0];
			m[0][1] = other.m[0][1];
			m[1][1] = other.m[1][1];
		}

		mat2x2::mat2x2(std::string call_this_the_identity)
		{
			if(call_this_the_identity == std::string("Identity")
			|| call_this_the_identity == std::string("identity")
			|| call_this_the_identity == std::string("I"))
			{
				MakeMeIdentity();
			}
			else
			{
				m[0][0] = 0.0;
				m[1][0] = 0.0;
				m[0][1] = 0.0;
				m[1][1] = 0.0;
			}
		}
		
		double mat2x2::GetDet() const
		{
			return (m[0][0] * m[1][1]) - (m[0][1] * m[1][0]);
		}
		
		/*static*/ mat2x2 mat2x2::mult(const mat2x2 & m1, const mat2x2 & m2)
		{
			mat2x2 retval;
			
			char i,j;
			char nnn;
			
			for(i=0; i<2; i++)
			{
				for(j=0; j<2; j++)
				{
					for(nnn=0; nnn<2; nnn++)
					{
						retval.m[i][j] += (m1.m[i][nnn] * m2.m[nnn][j]);
					}
				}
			}

			return retval;
		}
		
		void mat2x2::print(std::ostream & sout) const
		{
			for(char i=0; i<2; i++)
			{
				sout << std::left << "| " << std::setw(11) << m[i][0] << std::setw(10) << m[i][1] << "|" << std::endl;
			}
		}
		
//===================================================================================
		
		mat3x3::mat3x3(	double a11, double a12, double a13,
				double a21, double a22, double a23,
				double a31, double a32, double a33)
		{
			m[0][0] = a11;	m[0][1] = a12;	m[0][2] = a13;
			m[1][0] = a21;	m[1][1] = a22;	m[1][2] = a23;
			m[2][0] = a31;	m[2][1] = a32;	m[2][2] = a33;
		}

		mat3x3::mat3x3()
		{
			char j;
			for(char i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					m[i][j] = 0.0;
				}
			}
		}

		mat3x3::mat3x3(const mat3x3 & other)
		{
			char n=0;
			char i,j;

			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					m[i][j] = other.m[i][j];
				}
			}
		}

		mat3x3::mat3x3(std::string call_this_the_identity)
		{
			if(call_this_the_identity == std::string("Identity")
			|| call_this_the_identity == std::string("identity")
			|| call_this_the_identity == std::string("I"))
			{
				MakeMeIdentity();
			}
			else
			{
				char i,j;

				for(i=0; i<3; i++)
				{
					for(j=0; j<3; j++)
					{
						m[i][j] = 0.0;
					}
				}
			}
		}

		/*static*/ mat3x3 mat3x3::mult(const mat3x3 & m1, const mat3x3 & m2)
		{
			mat3x3 retval;
			
			char i,j;
			char nnn;
			
			for(i=0; i<3; i++)
			{
				for(j=0; j<3; j++)
				{
					for(nnn=0; nnn<3; nnn++)
					{
						retval.m[i][j] += (m1.m[i][nnn] * m2.m[nnn][j]);
					}
				}
			}

			return retval;
		}

		/*
		const phys::vec3 mat3x3::operator*(const phys::vec3 & other) const
		{
			phys::vec3 retval;
			
			for(char i=0; i<3; i++)
			{
				retval[i] += (m[i][0] + m[i][1] + m[i][2]);
			}
			
			return retval;
		}
		*/

		void mat3x3::print(std::ostream & sout) const
		{
			for(char i=0; i<3; i++)
			{
				sout << std::left << "| " << std::setw(14) << m[i][0] << std::setw(14) << m[i][1] << std::setw(13) << m[i][2] << "|" << std::endl;
			}
		}


		mat2x2 mat3x3::get_mat2x2_by_excluding_one(int i, int j) const
		{
			mat2x2 retval;

			if(i <= 0 || j <= 0 || i > 3 || j > 3)
			{
				std::cout << std::endl << "ERROR: mat3x3::get_mat2x2_by_excluding_one --- called with wrong calling convention - call with (1...3) not [0...2]" << std::endl;
				return retval;
			}

			i -= 1;
			j -= 1;

			char II,JJ;
			
			for(II=0; II<3; II++)
			{
				for(JJ=0; JJ<3; JJ++)
				{
					if(II != i && JJ != j)
					{
						retval.m[ (i == 0) ? (II-1) : ((i == 2) ? II : (II == 2 ? 1 : 0))
								]
								[ (j == 0) ? (JJ-1) : ((j == 2) ? JJ : (JJ == 2 ? 1 : 0))
								]
								= m[II][JJ];
					}
				}
			}

			return retval;
		}


		double mat3x3::GetDet() const
		{
			return m[0][0]*(get_mat2x2_by_excluding_one(1,1).GetDet())
				-  m[0][1]*(get_mat2x2_by_excluding_one(1,2).GetDet())
				+  m[0][2]*(get_mat2x2_by_excluding_one(1,3).GetDet());
		}

		mat3x3 mat3x3::GetInverse() const
		{
			double A,B,C,D,E,F,G,H,I;

			A =  (get_mat2x2_by_excluding_one(1,1).GetDet());
			B = -(get_mat2x2_by_excluding_one(1,2).GetDet());
			C =  (get_mat2x2_by_excluding_one(1,3).GetDet());
			
			D = -(get_mat2x2_by_excluding_one(2,1).GetDet());
			E =  (get_mat2x2_by_excluding_one(2,2).GetDet());
			F = -(get_mat2x2_by_excluding_one(2,3).GetDet());
			
			G =  (get_mat2x2_by_excluding_one(3,1).GetDet());
			H = -(get_mat2x2_by_excluding_one(3,2).GetDet());
			I =  (get_mat2x2_by_excluding_one(3,3).GetDet());

			double DetA = m[0][0]*A + m[0][1]*B + m[0][2]*C;

			if(mathTools::cmpDoubleToZero(DetA))
				return mat3x3(*this);

			mat3x3 inverse(	A,D,G,
						B,E,H,
						C,F,I);

			inverse /= DetA;
			
			return inverse;
		}
		
		void mat3x3::MakePassiveRotation_X_axis(double theta)
		{
			MakeMeZero();

			m[0][0] = 1.0;
									m[1][1] = cos(theta);	m[1][2] = sin(theta);
									m[2][1] = -m[1][2];		m[2][2] = m[1][1];
		}

		void mat3x3::MakePassiveRotation_Y_axis(double theta)
		{
			MakeMeZero();
			
			m[0][0] = cos(theta);							m[0][2] = -sin(theta);
									m[1][1] = 1.0;
			m[2][0] = -m[0][2];								m[2][2] = m[0][0];
		}

		void mat3x3::MakePassiveRotation_Z_axis(double theta)
		{
			MakeMeZero();
			
			m[0][0] = cos(theta);	m[0][1] = sin(theta);
			m[1][0] = -m[0][1];		m[1][1] = m[0][0];
															m[2][2] = 1.0;
		}

//===================================================================================
//===================================================================================
	
	vec2_polar point::GetEquivalent_polar() const
	{
		return vec2_polar(getLength(), atan2(y,x));
	}
  //vec3_spherical vec3::GetEquivalent_spherical() const;
	vec3_spherical vec3::GetEquivalent_spherical() const
	{
		return vec3_spherical(getLength(), atan2(y,x), atan2(sqrt(x*x+y*y), z));
	}

	vec3 vec3_spherical::to_cartesian(const vec3_spherical & spherical_source)
	{
		return vec3(spherical_source.r*sin(spherical_source.theta)*cos(spherical_source.phi),
					spherical_source.r*sin(spherical_source.theta)*sin(spherical_source.phi),
					spherical_source.r*cos(spherical_source.theta));
	}

	vec3 vec3_spherical::convert_spherical_vel_to_cartesian_vel(const vec3_spherical & sphericalsource_pos, const vec3_spherical & sphericalsource_vel)
	{
		const vec3_spherical & ssp(sphericalsource_pos);
		const vec3_spherical & ssv(sphericalsource_vel);

		vec3 rhat(		sin(ssp.theta)*cos(ssp.phi), sin(ssp.theta)*sin(ssp.phi),  cos(ssp.theta));
		vec3 thetahat(	cos(ssp.theta)*cos(ssp.phi), cos(ssp.theta)*sin(ssp.phi), -sin(ssp.theta));
		vec3 phihat(	-sin(ssp.phi),               cos(ssp.phi), 0.0);

		return (rhat * ssv.r) + (thetahat * (ssp.r * ssv.theta)) + (phihat * (ssp.r * sin(ssp.theta) * ssv.phi));
	}


	
	void vec2_polar::print(std::ostream & sout) const
	{
		sout << std::left << "| r:   " << std::setw(11) << r << "|" << std::endl;
		sout << std::left << "| phi: " << std::setw(11) << phi << "|" << std::endl;
	}

	void vec3_spherical::print(std::ostream & sout) const
	{
		sout << std::left << "| r:     " << std::setw(11) << r << "|" << std::endl;
		sout << std::left << "| phi:   " << std::setw(11) << phi << "|" << std::endl;
		sout << std::left << "| theta: " << std::setw(11) << theta << "|" << std::endl;
	}

	
	void point::print(std::ostream & sout) const
	{
		sout << std::left << "| " << std::setw(11) << x << "|" << std::endl;
		sout << std::left << "| " << std::setw(11) << y << "|" << std::endl;
	}
	void vec3::print(std::ostream & sout) const
	{
		sout << std::left << "| " << std::setw(11) << x << "|" << std::endl;
		sout << std::left << "| " << std::setw(11) << y << "|" << std::endl;
		sout << std::left << "| " << std::setw(11) << z << "|" << std::endl;
	}



}