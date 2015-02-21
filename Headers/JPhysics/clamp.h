
#ifndef __PHYS_MATH_CLAMP_TEMPLATE_TOOL__
#define __PHYS_MATH_CLAMP_TEMPLATE_TOOL__

namespace phys
{
	template <typename T>
	static T clamp(const T& value, const T& min, const T& max) 
	{
	  return value < min ? min : (value > max ? max : value); 
	}
	
	template <typename T>
	static T clamper(T& value, const T& min, const T& max) 
	{
	  return value = (value < min ? min : (value > max ? max : value)); 
	}
	
	template <typename T>
	static T minimum(const T& value, const T& minn) 
	{
	  return value > minn ? minn : value; 
	}
	
	template <typename T>
	static T maximum(const T& value, const T& maxx) 
	{
	  return value < maxx ? maxx : value; 
	}
}

#endif