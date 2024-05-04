/*
	Author: discord/bruh4096#4512
*/

#pragma once

#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include "geo_utils.hpp"


namespace libnav
{
	enum XPLM_fix_navaid_types // For fixes in earth_awy.dat and earth_hold.dat
    {
        XP_FIX_WPT = 11,
        XP_FIX_NDB = 2,
        XP_FIX_VHF = 3
    };

	enum class DbErr
	{
		ERR_NONE,
		SUCCESS,
		FILE_NOT_FOUND,
		DATA_BASE_ERROR,
		BAD_ALLOC,
		PARTIAL_LOAD
	};

	enum class NavaidType 
	{
		NONE = 0,
		WAYPOINT = 1,
		NDB = 2,
		VOR = 4,
		ILS_LOC = 8,
		ILS_LOC_ONLY = 16,
		ILS_GS = 32,
		ILS_FULL = 64,
		DME = 128,
		DME_ONLY = 256,
		VOR_DME = 512,
		ILS_DME = 1024,
		VHF_NAVAID = VOR + DME + DME_ONLY + VOR_DME,
		ILS = ILS_LOC + ILS_LOC_ONLY + ILS_GS + ILS_FULL + ILS_DME,
		NAVAID = 2047,
		OUTER_MARKER = 2048,
		MIDDLE_MARKER = 4096,
		INNER_MARKER = 8192,
		MARKER = OUTER_MARKER + MIDDLE_MARKER + INNER_MARKER,
		// Used only by CIFP parser:
		RWY = 16384,
		APT = 32768
	};

	// N_COL_AIRAC is the Number of columns in a line declaring the airac cycle
	// of earth_*.dat file
	constexpr int N_COL_AIRAC = 16;  
	constexpr int AIRAC_CYCLE_WORD = 6;
	constexpr int AIRAC_CYCLE_LINE = 2;
	constexpr char AIRAC_WORD_SEP = ' ';

	typedef uint16_t navaid_type_t;


	struct earth_data_line_t  // General variables to describe a line of earth*.dat
	{
		int airac_cycle;
        bool is_parsed=false, is_last=false, is_airac=false;
	};

	/*
		Function: xp_fix_type_to_libnav
		Description:
		Converts x-plane navaid type used in earth_awy.dat and earth_hold.dat
		to libnav type.
	*/

	inline NavaidType xp_fix_type_to_libnav(navaid_type_t type) 
	{
		switch (type)
        {
        case XP_FIX_WPT:
            return NavaidType::WAYPOINT;
        case XP_FIX_NDB:
            return NavaidType::NDB;
        case XP_FIX_VHF:
            return NavaidType::VHF_NAVAID;
        default:
            return NavaidType::NONE;
        }
	}

	inline navaid_type_t libnav_to_xp_fix(NavaidType type)
	{
		if(type == NavaidType::WAYPOINT)
			return XP_FIX_WPT;
		if(type == NavaidType::NDB)
			return XP_FIX_NDB;
		if(static_cast<navaid_type_t>(type) & 
			static_cast<navaid_type_t>(NavaidType::VHF_NAVAID))
			return XP_FIX_VHF;
		return 0;
	}

	inline bool does_file_exist(std::string name)
	{
		std::ifstream file(name, std::ifstream::in);
		if (file.is_open())
		{
			file.close();
			return true;
		}
		file.close();
		return false;
	}

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
}; // namespace libnav
