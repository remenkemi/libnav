/*
	This project is licensed under 
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	Author: discord/bruh4096#4512
*/

#pragma once

#include "arpt_db.hpp"
#include "navaid_db.hpp"
#include "common.hpp"


enum POI_types
{
	POI_NULL = 0,
	POI_WAYPOINT = 2,
	POI_NAVAID = 3,
	POI_AIRPORT = 5,
	POI_RWY = 7
};


namespace libnav
{
	struct POI_t
	{
		std::string id;
		std::vector<geo::point> wpt;
		std::vector<navaid_entry_t> navaid;
		airport_entry_t arpt;
		uint8_t type;
	};


	/*
		NavDB provides is unified interface for all nav data bases.
	*/

	class NavDB
	{
	public:
		NavDB(std::shared_ptr<libnav::ArptDB> arpt_ptr, std::shared_ptr<libnav::NavaidDB> navaid_ptr);

		DbErr is_arpt_loaded();

		DbErr is_wpt_loaded();

		DbErr is_navaid_loaded();

		//These member functions are just wrappers around ArptDB member functions.

		int get_airport_data(std::string icao_code, airport_data_t* out);

		int get_apt_rwys(std::string icao_code, runway_data* out);

		int get_rnw_data(std::string apt_icao, std::string rnw_id, runway_entry_t* out);

		//These member functions are just wrappers around NavaidDB member functions.

		int get_wpt_data(std::string id, std::vector<waypoint_entry_t>* out, 
			std::string area_code="", std::string country_code="", 
			NavaidType type=NavaidType::NAVAID, 
			navaid_filter_t filt_func=default_navaid_filter, void* ref=NULL);

		std::string get_fix_desc(waypoint_t& fix);

		bool is_navaid_of_type(std::string id, NavaidType type);

	private:
		std::shared_ptr<libnav::ArptDB> arpt_db;
		std::shared_ptr<libnav::NavaidDB> navaid_db;
	};
}; // namespace libnav
