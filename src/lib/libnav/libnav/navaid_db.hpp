#pragma once

#include <fstream>
#include <future>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <algorithm>
#include <string>
#include <sstream>
#include "geo_utils.hpp"
#include "common.hpp"


namespace libnav
{
	constexpr double VOR_MAX_SLANT_ANGLE_DEG = 40;
	constexpr double DME_DME_PHI_MIN_DEG = 30;
	constexpr double DME_DME_PHI_MAX_DEG = 180 - DME_DME_PHI_MIN_DEG;
	constexpr double MAX_ANG_DEV_MERGE = 0.03;
	constexpr size_t NAVAID_ENTRY_CACHE_SZ = 300000;
	//Number of lines at the beginning of the .dat file to ignore
	constexpr int N_NAVAID_LINES_IGNORE = 3;

	typedef uint16_t navaid_type_t;


	enum XPLM_navaid_types
	{
		XP_NAV_NDB = 2,
		XP_NAV_VOR = 3,
		XP_NAV_ILS_LOC = 4,
		XP_NAV_ILS_LOC_ONLY = 5,
		XP_NAV_ILS_GS = 6,
		XP_NAV_OM = 7,  // Outer marker
		XP_NAV_MM = 8,  // Middle marker
		XP_NAV_IM = 9,  // Inner marker
		XP_NAV_ILS_FULL = 10,
		XP_NAV_DME = 12,
		XP_NAV_DME_ONLY = 13,
		XP_NAV_VOR_DME = 15,
		XP_NAV_ILS_DME = 18
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
		VHF_NAVAID = 2047 - WAYPOINT - NDB,
		NAVAID = 2047,
		OUTER_MARKER = 2048,
		MIDDLE_MARKER = 4096,
		INNER_MARKER = 8192,
		MARKER = OUTER_MARKER + MIDDLE_MARKER + INNER_MARKER,
		// Used only by CIFP parser:
		RWY = 16384,
		APT = 32768
	};


	NavaidType xp_type_to_libnav(navaid_type_t tp);

	NavaidType make_composite(NavaidType tp1, NavaidType tp2);


	struct navaid_entry_t
	{
		uint16_t max_recv;
		double elevation, freq;
	};

	struct waypoint_entry_t
	{
		NavaidType type;
		uint32_t arinc_type = 0;  // Ref: arinc424 spec, section 5.42
		geo::point pos;
		std::string area_code;
		std::string country_code;
		navaid_entry_t* navaid = nullptr;
	};

	struct waypoint_t
	{
		std::string id;
		waypoint_entry_t data;
	};

	typedef std::vector<libnav::waypoint_t> wpt_tile_t;

	typedef std::unordered_map<std::string, std::vector<libnav::waypoint_entry_t>> wpt_db_t;


	class WaypointEntryCompare
	{
	public:
		geo::point ac_pos; // Aircraft position
		bool operator()(waypoint_entry_t w1, waypoint_entry_t w2);
	};

	class WaypointCompare
	{
	public:
		geo::point ac_pos; // Aircraft position
		bool operator()(waypoint_t w1, waypoint_t w2);
	};

	class NavaidDB
	{
	public:

		DbErr err_code;


		NavaidDB(std::string wpt_path, std::string navaid_path);

		DbErr is_loaded();

		int get_wpt_cycle();

		int get_navaid_cycle();

		bool load_waypoints();

		bool load_navaids();

		bool is_wpt(std::string id);

		bool is_navaid_of_type(std::string id, NavaidType type);

		// get_wpt_data returns 0 if waypoint is not in the database. 
		// Otherwise, returns number of items written to out.
		int get_wpt_data(std::string id, std::vector<waypoint_entry_t>* out, 
			std::string area_code="", std::string country_code="", 
			NavaidType type=NavaidType::NAVAID);

		std::string get_fix_desc(waypoint_t& fix);

		void reset();

		~NavaidDB();

	private:
		int wpt_airac_cycle;
		int navaid_airac_cycle;

		std::string sim_wpt_db_path;
		std::string sim_navaid_db_path;

		std::future<bool> wpt_loaded;
		std::future<bool> navaid_loaded;

		std::mutex wpt_db_mutex;
		std::mutex navaid_db_mutex;

		std::mutex wpt_desc_mutex;
		std::mutex navaid_desc_mutex;

		wpt_db_t wpt_cache;
		navaid_entry_t* navaid_entries;
		size_t n_navaid_entries;

		std::unordered_map<std::string, std::string> wpt_desc_db;
		std::unordered_map<std::string, std::string> navaid_desc_db;


		navaid_entry_t* navaid_entries_add(navaid_entry_t data);

		void add_to_wpt_cache(waypoint_t wpt);

		void add_to_navaid_cache(waypoint_t wpt, navaid_entry_t data);


		static std::string get_fix_unique_ident(waypoint_t& fix);

		static void add_to_map_with_mutex(std::string& id, std::string& desc,
			std::mutex& mtx, std::unordered_map<std::string, std::string>& umap);

		static std::string get_map_val_with_mutex(std::string& id,
			std::mutex& mtx, std::unordered_map<std::string, std::string>& umap);
	};


	std::string navaid_to_str(NavaidType navaid_type);

	void sort_wpt_entry_by_dist(std::vector<waypoint_entry_t>* vec, geo::point p);

	void sort_wpts_by_dist(std::vector<waypoint_t>* vec, geo::point p);

}; // namespace libnav


namespace radnav_util
{
	/*
		The following function returns a fom in nm for a DME using a formula
		from RTCA DO-236C appendix C-3. The only argument is the total distance to 
		the station.
	*/

	double get_dme_fom(double dist_nm);

	/*
		The following function returns a fom in nm for a VOR using a formula
		from RTCA DO-236C appendix C-2.The only argument is the total distance to 
		the station.
	*/

	double get_vor_fom(double dist_nm);

	/*
		The following function returns a fom in nm for a VOR DME station.
		It accepts the total distance to the station as its only argument.
	*/

	double get_vor_dme_fom(double dist_nm);

	/*
		This function calculates a quality value for a pair of navaids given
		the encounter geometry angle and their respective qualities.
	*/

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

	double get_dme_dme_fom(double dist1_nm, double dist2_nm, double phi_rad);

	double get_dme_dme_qual(double phi_deg, double q1, double q2);


	struct navaid_t
	{
		std::string id;
		libnav::waypoint_entry_t data;
		double qual;

		/*
			This function calculates the quality ratio for a navaid.
			Navaids are sorted by this ratio to determine the best 
			suitable candidate(s) for radio navigation.
		*/

		void calc_qual(geo::point3d ac_pos);
	};

	struct navaid_pair_t
	{
		navaid_t* n1;
		navaid_t* n2;
		double qual;

		/*
			This function calculates a quality value for a pair of navaids.
			This is useful when picking candidates for DME/DME position calculation.
		*/

		void calc_qual(geo::point ac_pos);
	};
}; // namespace radnav_util