#include "cifp_parser.hpp"


namespace libnav
{
    // Functions for decoding some arinc data fields:

    ProcType str2proc_type(std::string s)
    {
        if(s == "SID")
        {
            return ProcType::SID;
        }
        if(s == "STAR")
        {
            return ProcType::STAR;
        }
        if(s == "APPCH")
        {
            return ProcType::APPROACH;
        }
        if(s == "PRDAT")
        {
            return ProcType::PRDAT;
        }
        if(s == "RWY")
        {
            return ProcType::RWY;
        }
        
        return ProcType::NONE;
    }

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
        *is_true = false;

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
        *as_time = false;

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
                    return stoi(s);
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

    TCHType char2tch_type(char c)
    {
        switch (c)
        {
        case 'I':
            return TCHType::ILS_MLS;
        case 'R':
            return TCHType::RNAV;
        case 'V':
            return TCHType::VGSI;
        case 'D':
            return TCHType::DEFAULT;
        default:
            return TCHType::NONE;
        }
    }

    LSCategory char2ls_category(char c)
    {
        switch(c)
        {
        case '0':
            return LSCategory::ILS_LOC_ONLY;
        case '1':
            return LSCategory::LS_CAT_I;
        case '2':
            return LSCategory::LS_CAT_II;
        case '3':
            return LSCategory::LS_CAT_III;
        case 'I':
            return LSCategory::IGS;
        case 'L':
            return LSCategory::LDA_GS;
        case 'A':
            return LSCategory::LDA_NO_GS;
        case 'S':
            return LSCategory::SDF_GS;
        case 'F':
            return LSCategory::SDF_NO_GS;
        default:
            return LSCategory::NONE;
        }
    }


    // arinc_fix_entry_t definitions:

    waypoint_t arinc_fix_entry_t::to_waypoint_t(std::string& area_code, 
        std::shared_ptr<NavDB> nav_db)
    {
        NavaidType lookup_type = NavaidType::NAV_NONE;
        std::string lookup_area = "ENRT";

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
            // Add section P
        }
        else if(db_section == 'P')
        {
            if(db_subsection == 'C')
            {
                lookup_area = area_code;
            }
        }
        else if(db_section != 'D' && db_section != 'E')
        {
            return {};
        }
        
        std::vector<waypoint_entry_t> wpts;
        nav_db->get_wpt_data(fix_ident, &wpts, lookup_area, country_code, lookup_type);

        assert(wpts.size() < 2);

        if(wpts.size() == 1)
        {
            return {fix_ident, wpts[0]};
        }

        return {};
    }

    // arinc_str_t definitions:

    arinc_leg_t arinc_str_t::get_leg(std::string& area_code, 
        std::shared_ptr<NavDB> nav_db)
    {
        arinc_leg_t out;
        out.rt_type = rt_type;

        out.main_fix = main_fix.to_waypoint_t(area_code, nav_db);

        out.wpt_desc = wpt_desc;
        out.turn_dir = char2dir(turn_dir);
        out.rnp = str2rnp(rnp);
        out.leg_type = leg_type;
        out.is_ovfy = tdv == 'Y';

        out.recd_navaid = recd_navaid.to_waypoint_t(area_code, nav_db);
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

        out.center_fix = center_fix.to_waypoint_t(area_code, nav_db);

        out.multi_cod = multi_cod;
        out.gnss_ind = gnss_ind;
        out.rt_qual1 = rt_qual1;
        out.rt_qual2 = rt_qual2;

        return out;
    }


    inline void parse_flt_string(arinc_leg_full_t& full_leg, 
        std::vector<std::string>& in, std::string& area_code, 
        std::shared_ptr<NavDB> nav_db)
    {
        arinc_str_t tmp;

        tmp.rt_type = in[1][0];
        

        tmp.main_fix.fix_ident = in[4];
        tmp.main_fix.country_code = in[5];
        tmp.main_fix.db_section = in[6][0];
        tmp.main_fix.db_subsection = in[7][0];
        tmp.wpt_desc = in[8];

        tmp.turn_dir = in[9][0];
        tmp.rnp = strutils::strip(in[10], ' ');
        tmp.leg_type = strutils::strip(in[11], ' ');
        tmp.tdv = in[12][0];

        tmp.recd_navaid.fix_ident = strutils::strip(in[13], ' ');
        tmp.recd_navaid.country_code = strutils::strip(in[14], ' ');
        tmp.recd_navaid.db_section = in[15][0];
        tmp.recd_navaid.db_subsection = in[16][0];

        tmp.arc_radius = strutils::stof_with_strip(in[17]);
        tmp.theta = strutils::stof_with_strip(in[18]);
        tmp.rho = strutils::stof_with_strip(in[19]);
        tmp.outbd_mag_crs = strutils::strip(in[20], ' ');
        tmp.outbd_dist_time = strutils::strip(in[21], ' ');

        tmp.alt_desc = in[22][0];
        tmp.alt1 = strutils::strip(in[23], ' ');
        tmp.alt2 = strutils::strip(in[24], ' ');
        tmp.trans_alt = strutils::stoi_with_strip(in[25]);

        tmp.speed_desc = in[26][0];
        tmp.spd_lim = strutils::stoi_with_strip(in[27]);
        tmp.vert_angle = strutils::stof_with_strip(in[28]) * 0.01;
        tmp.vert_scale = strutils::stoi_with_strip(in[29]);

        tmp.center_fix.fix_ident = strutils::strip(in[30], ' ');
        tmp.center_fix.country_code = strutils::strip(in[31], ' ');
        tmp.center_fix.db_section = in[32][0];
        tmp.center_fix.db_subsection = in[33][0];

        tmp.multi_cod = in[34][0];
        tmp.gnss_ind = in[35][0];
        tmp.rt_qual1 = in[36][0];
        tmp.rt_qual2 = in[37][0];

        full_leg.leg = tmp.get_leg(area_code, nav_db);
    }


    arinc_leg_full_t str2full_arinc(std::string& s, std::string& area_code, 
        std::shared_ptr<NavDB> nav_db)
    {
        arinc_leg_full_t out;
        
        std::vector<std::string> s_split = strutils::str_split(s, ARINC_FIELD_SEP);
        std::vector<std::string> proc_tp = strutils::str_split(s_split[0], ':');

        out.p_type = str2proc_type(proc_tp[0]);

        if(out.p_type != ProcType::NONE && out.p_type != ProcType::PRDAT 
            && out.p_type != ProcType::RWY && s_split.size() == N_ARINC_FLT_PROC_COL)
        {
            out.proc_name = s_split[2];
            out.trans_name = s_split[3];

            parse_flt_string(out, s_split, area_code, nav_db);
        }
        
        return out;
    }
}; // namespace libnav
