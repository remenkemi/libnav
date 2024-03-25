#include "cifp_parser.hpp"


namespace libnav
{
    TurnDir char2dir(char c)
    {
        switch (c)
        {
        case 'L':
            return TurnDir::LEFT;
        case 'R':
            return TurnDir::RIGHT;
        default:
            return TurnDir::EITHER;
        }
    }

    float str2rnp(std::string s)
    {
        if(s.length() == 3)
        {
            float num = atof(s.substr(0, 2).c_str());
            int exp = s[2] - '0';

            if(exp != 0)
            {
                num *= float(std::pow(0.1, exp));
            }

            return num;
        }
        return 0;
    }

    float str2outbd_crs(std::string s, bool* is_true)
    {
        if(s.length() == 4)
        {
            if(s[3] == 'T')
            {
                *is_true = true;
                return atof(s.substr(0, 3).c_str());
            }
            return atof(s.c_str()) * 0.1;
        }
        return 0;
    }

    float str2outbd_dist(std::string s, bool* as_time)
    {
        if(s.length() == 4)
        {
            if(s[0] == 'T')
            {
                *as_time = true;
                return atof(s.substr(1, 3).c_str()) * 0.1;
            }
            return atof(s.c_str()) * 0.1;
        }
        return 0;
    }


    // arinc_fix_entry_t definitions:

    waypoint_t arinc_fix_entry_t::to_waypoint_t(std::shared_ptr<NavaidDB> nav_db)
    {
        NavaidType lookup_type = NavaidType::NAV_NONE;

        if(db_section == 'D')
        {
            if(db_subsection == 'B')
            {
                lookup_type = NavaidType::NAV_NDB;
            }
            else
            {
                lookup_type = NavaidType::NAV_VHF_NAVAID;
            }
        }
        else if(db_section != 'D' && db_section != 'E')
        {
            return {};
        }
        
        std::vector<waypoint_entry_t> wpts;
        nav_db->get_wpt_data(fix_ident, &wpts, area_code, lookup_type);

        assert(wpts.size() < 2);

        if(wpts.size() == 1)
        {
            return {fix_ident, wpts[0]};
        }

        return {};
    }

    // arinc_str_t definitions:

    arinc_leg_t arinc_str_t::get_leg(std::shared_ptr<NavaidDB> nav_db)
    {
        arinc_leg_t out;
        out.rt_type = rt_type;

        out.main_fix = main_fix.to_waypoint_t(nav_db);

        out.wpt_desc = wpt_desc;
        out.turn_dir = char2dir(turn_dir);
        out.rnp = str2rnp(rnp);
        out.leg_type = leg_type;
        out.is_ovfy = tdv == 'Y';

        out.recd_navaid = recd_navaid.to_waypoint_t(nav_db);
        out.arc_radius = arc_radius;
        out.theta = theta;
        out.rho = rho;
        out.outbd_crs_deg = str2outbd_crs(outbd_mag_crs, &out.outbd_crs_true);
        out.outbd_dist_time = str2outbd_dist(outbd_dist_time, &out.outbd_dist_as_time);

        

        return out;
    }
}; // namespace libnav
