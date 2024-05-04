/*
	Author: discord/bruh4096#4512
*/

#include "libnav/nav_db.hpp"


namespace libnav
{
	NavDB::NavDB(std::shared_ptr<libnav::ArptDB> arpt_ptr, std::shared_ptr<libnav::NavaidDB> navaid_ptr)
	{
		navaid_db = navaid_ptr;
		arpt_db = arpt_ptr;
	}

	DbErr NavDB::is_arpt_loaded()
	{
		return arpt_db->is_loaded();
	}

	DbErr NavDB::is_wpt_loaded()
	{
		if(navaid_db->err_code != DbErr::ERR_NONE)
		{
			return navaid_db->err_code;
		}
		return navaid_db->wpt_loaded();
	}

	DbErr NavDB::is_navaid_loaded()
	{
		if(navaid_db->err_code != DbErr::ERR_NONE)
		{
			return navaid_db->err_code;
		}
		return navaid_db->navaids_loaded();
	}

	// Wrappers around ArptDB member functions.

	int NavDB::get_airport_data(std::string icao_code, airport_data_t* out)
	{
		return arpt_db->get_airport_data(icao_code, out);
	}

	int NavDB::get_apt_rwys(std::string icao_code, runway_data* out)
	{
		return arpt_db->get_apt_rwys(icao_code, out);
	}

	int NavDB::get_rnw_data(std::string apt_icao, std::string rnw_id, runway_entry_t* out)
	{
		return arpt_db->get_rnw_data(apt_icao, rnw_id, out);
	}

	// Wrappers around NavaidDB member functions.

	int NavDB::get_wpt_data(std::string id, std::vector<waypoint_entry_t>* out, 
			std::string area_code, std::string country_code, NavaidType type)
	{
		return navaid_db->get_wpt_data(id, out, area_code, country_code, type);
	}

	bool NavDB::is_navaid_of_type(std::string id, NavaidType type)
	{
		return navaid_db->is_navaid_of_type(id, type);
	}

	std::string NavDB::get_fix_desc(waypoint_t& fix)
	{
		return navaid_db->get_fix_desc(fix);
	}
}; // namespace libnav
