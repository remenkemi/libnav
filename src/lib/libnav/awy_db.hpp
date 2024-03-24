#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include "str_utils.hpp"
#include "navaid_db.hpp"


namespace libnav
{
    constexpr int N_AWY_LINES_IGNORE = 3;
    constexpr char AWY_NAME_SEP = '-';
    constexpr char AWY_RESTR_FWD = 'F';
    constexpr char AWY_RESTR_BWD = 'B';
    constexpr char AWY_RESTR_NONE = 'N';

    enum XPLM_awy_navaid_types
    {
        XP_AWY_WPT = 11,
        XP_AWY_NDB = 2,
        XP_AWY_VHF = 3
    };


    navaid_type_t xp_awy_type_to_libnav(navaid_type_t type);


    struct alt_restr_t
    {
        uint32_t lower, upper;
    };

    struct awy_entry_t
    {
        navaid_type_t type;
        std::string reg_code;  // Region code of navaid/fix
    };

    struct awy_point_t
    {
        std::string id;
        awy_entry_t data;
        alt_restr_t alt_restr;

        awy_point_t(std::string nm="", navaid_type_t tp=NAV_NONE, 
            std::string r_c="", uint32_t lower=0, uint32_t upper=0);
    };


    class AwyDB
    {
        typedef std::unordered_map<std::string, std::unordered_map<std::string, alt_restr_t>> graph_t;
        typedef std::unordered_map<std::string, graph_t> awy_db_t;
        typedef std::unordered_map<std::string, awy_entry_t> awy_data_db_t;

    public:
        AwyDB(std::string awy_path);

        bool is_in_awy(std::string awy, std::string point);

        int get_path(std::string awy, std::string start, 
            std::string end, std::vector<awy_point_t>* out);

        ~AwyDB();

    private:
        awy_db_t awy_db;
        std::unordered_map<std::string, awy_data_db_t> awy_data;


        void load_airways(std::string awy_path);

        void add_to_awy_db(awy_point_t p1, awy_point_t p2, std::string awy_nm, char restr);
    };    
}
