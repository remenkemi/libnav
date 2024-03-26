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

    AltMode char2alt_mode(char c)
    {
        switch (c)
        {
        case '+':
            return AltMode::AT_OR_ABOVE;
        case '-':
            return AltMode::AT_OR_BELOW;
        case 'B':
            return AltMode::WITHIN;
        case 'C':
            return AltMode::SID_AT_OR_ABOVE;
        case 'G':
            return AltMode::GS_AT;
        case 'H':
            return AltMode::GS_AT_OR_ABOVE;
        case 'I':
            return AltMode::GS_INTC_AT;
        case 'J':
            return AltMode::GS_INTC_AT_OR_ABOVE;
        case 'V':
            return AltMode::ALT_STEPDOWN_AT_AT_OR_ABOVE;
        case 'X':
            return AltMode::ALT_STEPDOWN_AT_AT;
        case 'Y':
            return AltMode::ALT_STEPDOWN_AT_AT_OR_BELOW;
        default:
            return AltMode::AT;
        }
    }

    int str2alt(std::string s)
    {
        if(s.length() == 5)
        {
            if(s[0] == 'F' && s[1] == 'L')
            {
                std::string num_part = s.substr(2, 3);
                if(strutils::is_numeric(num_part))
                {
                    return stoi(num_part) * 100;
                }
            }
            else
            {
                if(strutils::is_numeric(s))
                {
                    return stoi(s) * 100;
                }
            }
        }

        return 0;
    }

    SpeedMode char2spd_mode(char c)
    {
        switch(c)
        {
        case '+':
            return SpeedMode::AT_OR_ABOVE;
        case '-':
            return SpeedMode::AT_OR_BELOW;
        default:
            return SpeedMode::AT;
        }
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

        out.alt_desc = char2alt_mode(alt_desc);
        out.alt1_ft = str2alt(alt1);
        out.alt2_ft = str2alt(alt2);
        out.trans_alt = trans_alt;

        out.speed_desc = char2spd_mode(speed_desc);
        out.spd_lim_kias = spd_lim;
        out.vert_angle_deg = vert_angle;
        out.vert_scale_ft = vert_scale;

        out.center_fix = center_fix.to_waypoint_t(nav_db);
        
        out.multi_cod = multi_cod;
        out.gnss_ind = gnss_ind;
        out.rt_qual1 = rt_qual1;
        out.rt_qual2 = rt_qual2;

        return out;
    }
}; // namespace libnav
