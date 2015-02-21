
#ifndef __JPHYS_INT_TO_STRING_TOOL__
#define __JPHYS_INT_TO_STRING_TOOL__

#include <string>
#include <sstream>

namespace phys
{
	template <class T>
	inline std::string to_string (const T& t)
	{
		std::stringstream ss;
		ss << t;
		return ss.str();
	}

	template <class T>
	inline std::string to_istring (const T& t)
	{
		std::stringstream ss;
		ss << static_cast<int>(t);
		return ss.str();
	}
	
	template <class T>
	inline std::string to_ilstring (const T& t)
	{
		std::stringstream ss;
		ss << static_cast<long>(t);
		return ss.str();
	}
}

#endif
