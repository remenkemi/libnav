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
	enum class DbErr
	{
		ERR_NONE,
		SUCCESS,
		FILE_NOT_FOUND,
		DATA_BASE_ERROR,
		BAD_ALLOC,
		PARTIAL_LOAD
	};

	constexpr int AIRAC_CYCLE_WORD = 6;
	constexpr int AIRAC_CYCLE_LINE = 2;
	constexpr char AIRAC_WORD_SEP = ' ';


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

	inline int get_airac_cycle(std::string s)
	{
		int curr_word = 1;
		std::stringstream ss(s);
		std::string word;

		while(std::getline(ss, word, AIRAC_WORD_SEP) && curr_word <= AIRAC_CYCLE_WORD)
		{
			if(curr_word == AIRAC_CYCLE_WORD)
			{
				return std::stoi(word);
			}
			curr_word++;
		}
		
		return 0;
	}
}; // namespace libnav
