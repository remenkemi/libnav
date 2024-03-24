/*
	Author: discord/bruh4096#4512
*/

#pragma once

#include <iomanip>
#include <string>
#include <sstream>
#include "geo_utils.hpp"


namespace libnav
{
	inline std::string double_to_str(double num, uint8_t precision)
	{
		std::stringstream s;
		s << std::fixed << std::setprecision(precision) << num;
		return s.str();
	}

	inline int clamp(int val, int upper, int lower)
	{
		if (val > upper)
			return upper;
		else if (val < lower)
			return lower;
		return val;
	}
}
