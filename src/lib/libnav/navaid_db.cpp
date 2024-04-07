/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	Author(s): discord/bruh4096#4512

	This file contains definitions of member functions used within the REF NAV DATA page.
*/

#include "libnav/navaid_db.hpp"
#include <assert.h>


namespace libnav
{
	NavaidType xp_type_to_libnav(navaid_type_t tp)
	{
		switch(tp)
		{
		case XP_NAV_NDB:
			return NavaidType::NAV_NDB;
		case XP_NAV_VOR:
			return NavaidType::NAV_VOR;
		case XP_NAV_ILS_LOC_ONLY:
			return NavaidType::NAV_ILS_LOC_ONLY;
		case XP_NAV_ILS_LOC:
			return NavaidType::NAV_ILS_LOC;
		case XP_NAV_ILS_GS:
			return NavaidType::NAV_ILS_GS;
		case XP_NAV_ILS_FULL:
			return NavaidType::NAV_ILS_FULL;
		case XP_NAV_DME:
			return NavaidType::NAV_DME;
		case XP_NAV_DME_ONLY:
			return NavaidType::NAV_DME_ONLY;
		case XP_NAV_VOR_DME:
			return NavaidType::NAV_VOR_DME;
		case XP_NAV_ILS_DME:
			return NavaidType::NAV_ILS_DME;
		default:
			return NavaidType::NAV_NONE;
		}
	}

	NavaidType make_composite(NavaidType tp1, NavaidType tp2)
	{
		int v1 = static_cast<int>(tp1), v2 = static_cast<int>(tp2);
		int tp_sum = v1 + v2;

		if((tp_sum & static_cast<int>(NavaidType::NAV_ILS_LOC)) && 
			(tp_sum & static_cast<int>(NavaidType::NAV_ILS_GS)))
		{
			return NavaidType::NAV_ILS_FULL;
		}
		if((tp_sum & static_cast<int>(NavaidType::NAV_VOR)) && 
			(tp_sum & static_cast<int>(NavaidType::NAV_DME)))
		{
			return NavaidType::NAV_VOR_DME;
		}
		if((tp_sum & static_cast<int>(NavaidType::NAV_ILS_FULL)) && 
			(tp_sum & static_cast<int>(NavaidType::NAV_DME)))
		{
			return NavaidType::NAV_ILS_DME;
		}
		if((tp_sum & static_cast<int>(NavaidType::NAV_ILS_GS)) && 
			(tp_sum & static_cast<int>(NavaidType::NAV_DME)))
		{
			return NavaidType::NAV_ILS_DME;
		}
		if((tp_sum & static_cast<int>(NavaidType::NAV_ILS_LOC)) && 
			(tp_sum & static_cast<int>(NavaidType::NAV_DME)))
		{
			return NavaidType::NAV_ILS_DME;
		}
		if((tp_sum & static_cast<int>(NavaidType::NAV_ILS_LOC_ONLY)) && 
			(tp_sum & static_cast<int>(NavaidType::NAV_DME)))
		{
			return NavaidType::NAV_ILS_DME;
		}

		return NavaidType::NAV_NONE;
	}


	bool WaypointEntryCompare::operator()(waypoint_entry_t w1, waypoint_entry_t w2)
	{
		double d1 = w1.pos.get_great_circle_distance_nm(ac_pos);
		double d2 = w2.pos.get_great_circle_distance_nm(ac_pos);
		return d1 < d2;
	}

	bool WaypointCompare::operator()(waypoint_t w1, waypoint_t w2)
	{
		double d1 = w1.data.pos.get_great_circle_distance_nm(ac_pos);
		double d2 = w2.data.pos.get_great_circle_distance_nm(ac_pos);
		return d1 < d2;
	}

	NavaidDB::NavaidDB(std::string wpt_path, std::string navaid_path)
	{
		// Pre-defined stuff

		err_code = DbErr::ERR_NONE;

		// Paths

		sim_wpt_db_path = wpt_path;
		sim_navaid_db_path = navaid_path;


		navaid_entries = new navaid_entry_t[NAVAID_ENTRY_CACHE_SZ];
		n_navaid_entries = 0;

		if(navaid_entries == nullptr)
		{
			err_code = DbErr::BAD_ALLOC;
		}
		else
		{
			wpt_loaded = std::async(std::launch::async, [](NavaidDB* db) -> bool {return db->load_waypoints(); }, this);
			navaid_loaded = std::async(std::launch::async, [](NavaidDB* db) -> bool {return db->load_navaids(); }, this);
		}
	}

	// Public member functions:

	DbErr NavaidDB::is_loaded()
	{
		if(err_code == DbErr::ERR_NONE && (wpt_loaded.get() && navaid_loaded.get()))
		{
			err_code = DbErr::SUCCESS;
		}
		else
		{
			err_code = DbErr::FILE_NOT_FOUND;
		}

		return err_code;
	}

	int NavaidDB::get_wpt_cycle()
	{
		return wpt_airac_cycle;
	}

	int NavaidDB::get_navaid_cycle()
	{
		return navaid_airac_cycle;
	}

	void NavaidDB::reset()
	{
		delete[] navaid_entries;
		n_navaid_entries = 0;
	}

	NavaidDB::~NavaidDB()
	{

	}

	bool NavaidDB::load_waypoints()
	{
		std::ifstream file(sim_wpt_db_path);
		if (file.is_open())
		{
			std::string line;
			int i = 0;
			int limit = N_NAVAID_LINES_IGNORE;
			while (getline(file, line) && line != "99")
			{
				if (i >= limit)
				{
					// Construct a waypoint_t entry.
					std::stringstream s(line);
					std::string desc;
					waypoint_t wpt = {};
					wpt.data.type = NavaidType::NAV_WAYPOINT;
					s >> wpt.data.pos.lat_deg >> wpt.data.pos.lon_deg >> wpt.id >> 
						wpt.data.area_code >> wpt.data.country_code >> 
						wpt.data.arinc_type;

					getline(s, desc);
					if(desc.length())
					{
						desc = desc.substr(1, desc.length() - 1);
					}

					std::string unique_ident = get_fix_unique_ident(wpt);

					add_to_map_with_mutex(unique_ident, desc, wpt_desc_mutex, wpt_desc_db);
					add_to_wpt_cache(wpt);
				}
				else if(i+1 == AIRAC_CYCLE_LINE)
				{
					wpt_airac_cycle = get_airac_cycle(line);
				}
				i++;
			}
			file.close();
			return true;
		}
		return false;
	}

	bool NavaidDB::load_navaids()
	{
		std::ifstream file(sim_navaid_db_path);
		if (file.is_open())
		{
			std::string line;
			int i = 0;
			while (getline(file, line))
			{
				std::string check_val;
				std::stringstream s(line);
				s >> check_val;
				if (i >= N_NAVAID_LINES_IGNORE && check_val != "99")
				{
					// Construct a navaid entry.
					std::stringstream s(line);
					uint16_t type, max_recv;
					double lat, lon, elevation, mag_var;
					uint32_t freq;
					std::string id;
					std::string area_code;
					std::string desc;  // Spoken name of the navaid

					waypoint_t wpt;
					navaid_entry_t navaid;

					s >> type >> lat >> lon >> elevation >> freq >> max_recv >> mag_var >> 
						id >> wpt.data.area_code >> wpt.data.country_code;

					getline(s, desc);
					if(desc.length())
					{
						desc = desc.substr(1, desc.length() - 1);
					}

					wpt.id = id;
					wpt.data.type = xp_type_to_libnav(type);
					navaid.max_recv = max_recv;
					wpt.data.pos.lat_deg = lat;
					wpt.data.pos.lon_deg = lon;
					navaid.elevation = elevation;
					navaid.freq = freq;

					std::string unique_ident = get_fix_unique_ident(wpt);

					add_to_map_with_mutex(unique_ident, desc, navaid_desc_mutex, navaid_desc_db);
					add_to_navaid_cache(wpt, navaid);
				}
				else if(i+1 == AIRAC_CYCLE_LINE)
				{
					navaid_airac_cycle = get_airac_cycle(line);
				}
				else if (check_val == "99")
				{
					break;
				}
				i++;
			}
			file.close();
			return true;
		}
		return false;
	}

	bool NavaidDB::is_wpt(std::string id) 
	{
		std::lock_guard<std::mutex> lock(wpt_db_mutex);
		return wpt_cache.find(id) != wpt_cache.end();
	}

	bool NavaidDB::is_navaid_of_type(std::string id, NavaidType type)
	{
		if (is_wpt(id))
		{
			std::lock_guard<std::mutex> lock(wpt_db_mutex);
			for(int i = 0; i < int(wpt_cache[id].size()); i++)
			{
				NavaidType curr_type = wpt_cache[id][i].type;
				if((static_cast<int>(curr_type) & static_cast<int>(type)) == 
					static_cast<int>(curr_type))
				{
					return true;
				}
			}
		}
		return false;
	}

	int NavaidDB::get_wpt_data(std::string id, std::vector<waypoint_entry_t>* out, 
		std::string area_code, std::string country_code, NavaidType type)
	{
		if (is_wpt(id))
		{
			std::lock_guard<std::mutex> lock(wpt_db_mutex);
			std::vector<waypoint_entry_t>* waypoints = &wpt_cache.at(id);
			int n_waypoints = int(waypoints->size());
			for (int i = 0; i < n_waypoints; i++)
			{
				waypoint_entry_t wpt_curr = waypoints->at(i);

				bool is_fine = true;
				if(area_code != "" && wpt_curr.area_code != area_code)
				{
					is_fine = false;
				}
				if(country_code != "" && wpt_curr.country_code != country_code)
				{
					is_fine = false;
				}
				else if(type != NavaidType::NAV_NONE && 
					(static_cast<int>(wpt_curr.type) & static_cast<int>(type)) == 0)
				{
					is_fine = false;
				}
				if(is_fine)
				{
					out->push_back(wpt_curr);
				}
			}
		}
		return out->size();
	}

	std::string NavaidDB::get_fix_desc(waypoint_t& fix)
	{
		std::string unique_ident = get_fix_unique_ident(fix);
		if(fix.data.navaid != nullptr)
		{
			return get_map_val_with_mutex(unique_ident, navaid_desc_mutex, 
				navaid_desc_db);
		}

		return get_map_val_with_mutex(unique_ident, wpt_desc_mutex, wpt_desc_db);
	}

	// Private member functions:

	navaid_entry_t* NavaidDB::navaid_entries_add(navaid_entry_t data)
	{
		navaid_entries[n_navaid_entries] = data;
		n_navaid_entries++;

		assert(n_navaid_entries < NAVAID_ENTRY_CACHE_SZ);

		return &navaid_entries[n_navaid_entries-1];
	}

	void NavaidDB::add_to_wpt_cache(waypoint_t wpt)
	{
		// Find the navaid in the database by name.
		if (is_wpt(wpt.id))
		{
			std::lock_guard<std::mutex> lock(wpt_db_mutex);
			// If there is a waypoint with the same name in the database,
			// add new entry to the vector.
			wpt_cache.at(wpt.id).push_back(wpt.data);
		}
		else
		{
			std::lock_guard<std::mutex> lock(wpt_db_mutex);
			// If there is no waypoint with the same name in the database,
			// add a vector with tmp
			std::pair<std::string, std::vector<waypoint_entry_t>> p;
			p = std::make_pair(wpt.id, std::vector<waypoint_entry_t>{wpt.data});
			wpt_cache.insert(p);
		}
	}

	void NavaidDB::add_to_navaid_cache(waypoint_t wpt, navaid_entry_t data)
	{
		// Find the navaid in the database by name.
		if (is_wpt(wpt.id))
		{
			// If there is a navaid with the same name in the database,
			// add new entry to the vector.
			bool is_colocated = false;
			bool is_duplicate = false;
			std::vector<waypoint_entry_t>* entries = &wpt_cache.at(wpt.id);
			for (int i = 0; i < int(entries->size()); i++)
			{
				if (entries->at(i).navaid != nullptr)
				{
					waypoint_entry_t tmp_wpt = entries->at(i);
					navaid_entry_t* tmp_navaid = tmp_wpt.navaid;

					bool is_wpt_equal = !bool(memcmp(&tmp_wpt.pos, &wpt.data.pos, sizeof(geo::point)));
					bool is_type_equal = tmp_wpt.type == wpt.data.type;
					bool is_nav_equal = !bool(memcmp(tmp_wpt.navaid, &data, sizeof(navaid_entry_t)));
					bool is_equal = is_wpt_equal && is_nav_equal && is_type_equal;

					if (is_equal)
					{
						is_duplicate = true;
						break;
					}

					double lat_dev = abs(wpt.data.pos.lat_deg - tmp_wpt.pos.lat_deg);
					double lon_dev = abs(wpt.data.pos.lon_deg - tmp_wpt.pos.lon_deg);
					double ang_dev = lat_dev + lon_dev;
					NavaidType type_sum = make_composite(wpt.data.type, tmp_wpt.type);
					bool is_comp = type_sum != NavaidType::NAV_NONE;
					if (ang_dev < 0.001 && is_comp && data.freq == tmp_navaid->freq)
					{
						entries->at(i).type = type_sum;
						is_colocated = true;
						break;
					}
				}
			}
			if (!is_colocated && !is_duplicate)
			{
				wpt.data.navaid = navaid_entries_add(data);
				
				std::lock_guard<std::mutex> lock(wpt_db_mutex);
				entries->push_back(wpt.data);
			}
		}
		else
		{
			// If there is no navaid with the same name in the database,
			// add a vector with tmp
			wpt.data.navaid = navaid_entries_add(data);

			add_to_wpt_cache(wpt);
		}
	}


	std::string NavaidDB::get_fix_unique_ident(waypoint_t& fix)
	{
		return fix.id + fix.data.country_code + fix.data.area_code;
	}

	void NavaidDB::add_to_map_with_mutex(std::string& id, std::string& desc,
		std::mutex& mtx, std::unordered_map<std::string, std::string>& umap)
	{
		std::lock_guard<std::mutex> lock(mtx);

		umap[id] = desc;
	}

	std::string NavaidDB::get_map_val_with_mutex(std::string& id,
		std::mutex& mtx, std::unordered_map<std::string, std::string>& umap)
	{
		std::lock_guard<std::mutex> lock(mtx);

		if(umap.find(id) != umap.end())
		{
			return umap[id];
		}

		return "";
	}


	std::string navaid_to_str(NavaidType navaid_type)
	{
		switch (navaid_type)
		{
		case NavaidType::NAV_WAYPOINT:
			return "WPT";
		case NavaidType::NAV_NDB:
			return "NDB";
		case NavaidType::NAV_DME:
			return "DME";
		case NavaidType::NAV_VOR:
			return "VOR";
		case NavaidType::NAV_ILS_LOC_ONLY:
			return "ILS";
		case NavaidType::NAV_ILS_LOC:
			return "ILS";
		case NavaidType::NAV_ILS_GS:
			return "ILS";
		case NavaidType::NAV_ILS_FULL:
			return "ILS";
		case NavaidType::NAV_DME_ONLY:
			return "DME";
		case NavaidType::NAV_VOR_DME:
			return "VORDME";
		case NavaidType::NAV_ILS_DME:
			return "ILSDME";
		default:
			return "";
		}
	}

	void sort_wpt_entry_by_dist(std::vector<waypoint_entry_t>* vec, geo::point p)
	{
		WaypointEntryCompare comp;
		comp.ac_pos = p;

		sort(vec->begin(), vec->end(), comp);
	}

	void sort_wpts_by_dist(std::vector<waypoint_t>* vec, geo::point p)
	{
		WaypointCompare comp;
		comp.ac_pos = p;

		sort(vec->begin(), vec->end(), comp);
	}
}; // namespace libnav


namespace radnav_util
{
	/*
		The following function returns a fom in nm for a DME using a formula
		from RTCA DO-236C appendix C-3. The only argument is the total distance to
		the station.
	*/

	double get_dme_fom(double dist_nm)
	{
		double max_val = std::pow(0.085, 2);
		double tmp_val = std::pow(0.00125 * dist_nm, 2);
		if (max_val < tmp_val)
		{
			max_val = tmp_val;
		}
		double variance = std::pow(0.05, 2) + max_val;
		// Now convert variance to FOM(standard_deviation * 2)
		return sqrt(variance) * 2;
	}

	/*
		The following function returns a fom in nm for a VOR using a formula
		from RTCA DO-236C appendix C-2.The only argument is the total distance to
		the station.
	*/

	double get_vor_fom(double dist_nm)
	{
		double variance = std::pow((0.0122 * dist_nm), 2) + std::pow((0.0175 * dist_nm), 2);
		return sqrt(variance) * 2;
	}

	/*
		The following function returns a fom in nm for a VOR DME station.
		It accepts the total distance to the station as its only argument.
	*/

	double get_vor_dme_fom(double dist_nm)
	{
		double dme_fom = get_dme_fom(dist_nm);
		double vor_fom = get_vor_fom(dist_nm);
		if (vor_fom > dme_fom)
		{
			return vor_fom;
		}
		return dme_fom;
	}

	/*
		Function: get_dme_dme_fom
		Description:
		This function calculates a FOM value for a pair of navaids given
		the encounter geometry angle and their respective distances.
		Param:
		dist1_nm: quality value of the first DME
		dist2_nm: quality value of the second DME
		phi_rad: encounter geometry angle between 2 DMEs
		Return:
		Returns a FOM value.
	*/

	double get_dme_dme_fom(double dist1_nm, double dist2_nm, double phi_rad)
	{
		double sin_phi = sin(phi_rad);
		if(sin_phi)
		{
			double dme1_fom = get_dme_fom(dist1_nm);
			double dme2_fom = get_dme_fom(dist2_nm);
			if (dme1_fom > dme2_fom)
			{
				return dme1_fom / sin_phi;
			}
			return dme2_fom / sin_phi;
		}
		return 0;
	}

	/*
		Function: get_dme_dme_qual
		Description:
		This function calculates a quality value for a pair of navaids given
		the encounter geometry angle and their respective qualities.
		Param:
		phi_deg: encounter geometry angle between 2 DMEs
		q1: quality value of the first DME
		q2: quality value of the second DME
		Return:
		Returns a quality value. The higher the quality value, the better.
	*/

	double get_dme_dme_qual(double phi_deg, double q1, double q2)
	{
		if (phi_deg > libnav::DME_DME_PHI_MIN_DEG && 
			phi_deg < libnav::DME_DME_PHI_MAX_DEG)
		{
			double min_qual = q1;
			if (q2 < min_qual)
			{
				min_qual = q2;
			}

			double qual = (min_qual + 1 - abs(90 - phi_deg) / 90) / 2;
			return qual;
		}
		return -1;
	}

	/*
		This function calculates the quality ratio for a navaid.
		Navaids are sorted by this ratio to determine the best 
		suitable candidate(s) for radio navigation.
	*/

	void navaid_t::calc_qual(geo::point3d ac_pos)
	{
		libnav::navaid_entry_t* nav_data = data.navaid;
		if (nav_data != nullptr)
		{
			double lat_dist_nm = ac_pos.p.get_great_circle_distance_nm(data.pos);

			if (lat_dist_nm)
			{
				
				double v_dist_nm = abs(ac_pos.alt_ft - nav_data->elevation) * geo::FT_TO_NM;
				double slant_deg = atan(v_dist_nm / lat_dist_nm) * geo::RAD_TO_DEG;

				if (slant_deg > 0 && slant_deg < libnav::VOR_MAX_SLANT_ANGLE_DEG)
				{
					double true_dist_nm = sqrt(lat_dist_nm * lat_dist_nm + v_dist_nm * v_dist_nm);

					double tmp = 1 - (true_dist_nm / nav_data->max_recv);
					if (tmp >= 0)
					{
						qual = tmp;
						return;
					}
				}
			}
		}
		qual = -1;
	}
	
	/*
		This function calculates a quality value for a pair of navaids.
		This is useful when picking candidates for DME/DME position calculation.
	*/

	void navaid_pair_t::calc_qual(geo::point ac_pos)
	{
		if (n1 != nullptr && n2 != nullptr)
		{
			double b1 = n1->data.pos.get_great_circle_bearing_deg(ac_pos);
			double b2 = n2->data.pos.get_great_circle_bearing_deg(ac_pos);
			double phi = abs(b1 - b2);
			if (phi > 180)
				phi = 360 - phi;

			qual = get_dme_dme_qual(phi, n1->qual, n2->qual);
			return;
		}
		qual = -1;
	}
}; // namespace radnav_util
