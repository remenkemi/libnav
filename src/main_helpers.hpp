#include <iostream>
#include <memory>
#include <string>
#include <libnav/nav_db.hpp>
#include <libnav/awy_db.hpp>
#include <libnav/cifp_parser.hpp>

#define UNUSED(x) (void)(x)


double AC_LAT_DEF = 45.588670483;
double AC_LON_DEF = -122.598150383;


namespace dbg
{
    class Avionics
    {
    public:
        double ac_lat;
        double ac_lon;

        std::shared_ptr<libnav::ArptDB> arpt_db_ptr;
        std::shared_ptr<libnav::NavaidDB> navaid_db_ptr;

        libnav::AwyDB* awy_db;
        std::shared_ptr<libnav::NavDB> db;

        std::unordered_map<std::string, std::string> env_vars;

        std::string cifp_dir_path;


        Avionics(std::string apt_dat, std::string custom_apt, std::string custom_rnw,
            std::string fix_data, std::string navaid_data, std::string awy_data, 
            std::string cifp_path, double def_lat=AC_LAT_DEF, 
            double def_lon=AC_LON_DEF)
        {
            env_vars["ac_lat"] = strutils::double_to_str(def_lat, 8);
            env_vars["ac_lon"] = strutils::double_to_str(def_lon, 8);

            cifp_dir_path = cifp_path;

            ac_lat = def_lat;
            ac_lon = def_lon;

            arpt_db_ptr = 
                std::make_shared<libnav::ArptDB>(apt_dat, custom_apt, custom_rnw);
	        navaid_db_ptr = 
                std::make_shared<libnav::NavaidDB>(fix_data, navaid_data);

            awy_db = new libnav::AwyDB(awy_data);
            db = std::make_shared<libnav::NavDB>(arpt_db_ptr, navaid_db_ptr);

            libnav::DbErr err_arpt = db->is_arpt_loaded();
            libnav::DbErr err_nav = db->is_navaid_loaded();

            std::cout << navaid_db_ptr->get_wpt_cycle() << " " <<
                navaid_db_ptr->get_navaid_cycle() << "\n";

            if(err_arpt != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load airport database\n";
            }
            if(err_nav != libnav::DbErr::SUCCESS)
            {
                std::cout << "Unable to load navaid database\n";
            }
        }

        void update()
        {
            update_pos();
        }

        ~Avionics()
        {
            delete awy_db;
            db.reset();
            navaid_db_ptr.reset();
            navaid_db_ptr.reset();
            arpt_db_ptr.reset();
        }

    private:
        void update_pos()
        {
            bool lat_valid = strutils::is_numeric(env_vars["ac_lat"]);
            bool lon_valid = strutils::is_numeric(env_vars["ac_lon"]);
            
            if(lon_valid && lat_valid)
            {
                ac_lat = std::stod(env_vars["ac_lat"]);
                ac_lon = std::stod(env_vars["ac_lon"]);
            }
        }
    };

    typedef void (*cmd)(Avionics*, std::vector<std::string>&);


    inline void set_var(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Command expects 2 arguments\n";
            return;
        }

        av->env_vars[in[0]] = in[1];
    }

    inline void name(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() < 1)
        {
            std::cout << "Too few arguments provided\n";
            return;
        }
        
        std::string poi_id = in[0];

        std::vector<libnav::waypoint_entry_t> found_wpts;

        size_t n_wpts_found = av->db->get_wpt_data(poi_id, &found_wpts);

        for(size_t i = 0; i < n_wpts_found; i++)
        {
            libnav::waypoint_t wpt = {poi_id, found_wpts[i]};

            std::cout << av->db->get_fix_desc(wpt) << "\n";
        }
    }

    inline void display_poi_info(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() < 1)
        {
            std::cout << "Too few arguments provided\n";
            return;
        }

        std::string poi_id = in[0];

        libnav::airport_data_t found_arpt;
        std::vector<libnav::waypoint_entry_t> found_wpts;

        std::shared_ptr<libnav::NavDB> db = av->db;

        size_t n_arpts_found = db->get_airport_data(poi_id, &found_arpt);
        size_t n_wpts_found = db->get_wpt_data(poi_id, &found_wpts);

        if (n_arpts_found)
        {
            std::string lat_str = strutils::lat_to_str(found_arpt.pos.lat_deg);
            std::string lon_str = strutils::lon_to_str(found_arpt.pos.lon_deg);
            std::cout << poi_id << " " << lat_str << " " << lon_str << "\n";
        }
        else if (n_wpts_found)
        {
            libnav::sort_wpt_entry_by_dist(&found_wpts, { av->ac_lat, av->ac_lon });

            for (size_t i = 0; i < n_wpts_found; i++)
            {
                std::string lat_str = strutils::lat_to_str(found_wpts[i].pos.lat_deg);
                std::string lon_str = strutils::lon_to_str(found_wpts[i].pos.lon_deg);
                libnav::NavaidType wpt_type = found_wpts[i].type;
                std::string type_str = libnav::navaid_to_str(wpt_type);
                if (wpt_type == libnav::NavaidType::WAYPOINT)
                {
                    std::cout << poi_id << " " << lat_str << " " << lon_str << " " << 
                        type_str << "\n";
                }
                else
                {
                    libnav::navaid_entry_t* navaid_data = found_wpts[i].navaid;
                    double freq = navaid_data->freq;
                    std::string lat_dms = strutils::lat_to_str(found_wpts[i].pos.lat_deg);
                    std::string lon_dms = strutils::lon_to_str(found_wpts[i].pos.lon_deg);
                    std::cout << poi_id << " " << lat_dms << " " << lon_dms << " " << type_str << " " << 
                        strutils::freq_to_str(freq) << "\n";
                }
            }
        }
        else
        {
            std::cout << "Not in data base\n";
        }
    }

    inline void get_path(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 3)
        {
            std::cout << "Command expects 3 arguments: <airway name> <entry point> <exit point>\n";
            return;
        }

        std::vector<libnav::awy_point_t> tmp;
        av->awy_db->get_path(in[0], in[1], in[2], &tmp);
        for(int i = 0; i < int(tmp.size()); i++)
        {
            std::cout << tmp[i].id << " " << tmp[i].alt_restr.lower 
                << " " << tmp[i].alt_restr.upper << "\n";
        }
    }

    inline void quit(Avionics* av, std::vector<std::string>& in)
    {
        UNUSED(av);

        if(in.size())
        {
            std::cout << "Too many arguments provided\n";
            return;
        }
        std::exit(0);
    }

    inline void allsid(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 1)
        {
            std::cout << "Invalid arguments provided\n";
            return;
        }
        
        libnav::Airport apt(in[0], av->db, av->cifp_dir_path);

        std::set<std::string> sids = apt.get_all_sids();

        for(auto i: sids)
        {
            std::cout << i << "\n";
        }
    }

    inline void getsid(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 3)
        {
            std::cout << "Invalid arguments provided\n";
            return;
        }
        
        libnav::Airport apt(in[0], av->db, av->cifp_dir_path);

        libnav::arinc_leg_seq_t sid_legs = apt.get_sid(in[1], in[2]);
        for(auto i: sid_legs)
        {
            std::cout << i.main_fix.id << " " << i.leg_type << "\n";
        }
    }

    inline void getstar(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 3)
        {
            std::cout << "Command expected 3 arguments: <airport icao> <runway id>\n";
            return;
        }
        
        libnav::Airport apt(in[0], av->db, av->cifp_dir_path);

        if(apt.err_code != libnav::DbErr::SUCCESS &&
            apt.err_code != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid airport icao\n";
            return;
        }

        libnav::arinc_leg_seq_t star_legs = apt.get_star(in[1], in[2]);
        for(auto i: star_legs)
        {
            std::cout << i.main_fix.id << " " << i.leg_type << "\n";
        }
    }

    inline void lssid(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Command expected 2 arguments: <airport icao> <runway id>\n";
            return;
        }

        libnav::Airport apt(in[0], av->db, av->cifp_dir_path);

        if(apt.err_code != libnav::DbErr::SUCCESS &&
            apt.err_code != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid airport icao\n";
            return;
        }

        std::set<std::string> sids = apt.get_sid_by_rwy(in[1]);

        if(!sids.size())
        {
            std::cout << "Invalid runway\n";
        }
        else
        {
            for(auto i: sids)
            {
                std::cout << i << "\n";
            }
        }
    }

    inline void lsstar(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Command expected 2 arguments: <airport icao> <runway id>\n";
            return;
        }

        libnav::Airport apt(in[0], av->db, av->cifp_dir_path);

        if(apt.err_code != libnav::DbErr::SUCCESS &&
            apt.err_code != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid airport icao\n";
            return;
        }

        std::set<std::string> stars = apt.get_star_by_rwy(in[1]);

        if(!stars.size())
        {
            std::cout << "Invalid runway\n";
        }
        else
        {
            for(auto i: stars)
            {
                std::cout << i << "\n";
            }
        }
    }

    inline void lssidtrans(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Invalid arguments provided\n";
            return;
        }

        libnav::Airport apt(in[0], av->db, av->cifp_dir_path);

        if(apt.err_code != libnav::DbErr::SUCCESS &&
            apt.err_code != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid airport icao\n";
            return;
        }

        std::set<std::string> trans = apt.get_trans_by_sid(in[1]);

        for(auto i: trans)
        {
            std::cout << i << "\n";
        }
    }

    inline void lsstartrans(Avionics* av, std::vector<std::string>& in)
    {
        if(in.size() != 2)
        {
            std::cout << "Invalid arguments provided\n";
            return;
        }

        libnav::Airport apt(in[0], av->db, av->cifp_dir_path);

        if(apt.err_code != libnav::DbErr::SUCCESS &&
            apt.err_code != libnav::DbErr::PARTIAL_LOAD)
        {
            std::cout << "Invalid airport icao\n";
            return;
        }

        std::set<std::string> trans = apt.get_trans_by_star(in[1]);

        for(auto i: trans)
        {
            std::cout << i << "\n";
        }
    }

    std::unordered_map<std::string, cmd> cmd_map = {
        {"set", set_var},
        {"name", name}, 
        {"poinfo", display_poi_info}, 
        {"get_path", get_path},
        {"quit", quit},
        {"allsid", allsid},
        {"getsid", getsid},
        {"getstar", getstar},
        {"lssid", lssid},
        {"lsstar", lsstar},
        {"lssidtrans", lssidtrans},
        {"lsstartrans", lsstartrans}
        };
}
