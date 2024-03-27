#include "cifp_parser.hpp"


namespace libnav
{
    // Functions for decoding some arinc data fields:

    ProcType str2proc_type(std::string s)
    {
        char tmp[ARINC_MAX_TP_LENGTH+1];
        int sz = 0;

        while (sz < int(s.length()) && sz < ARINC_MAX_TP_LENGTH && s[sz] != ':')
        {
            tmp[sz] = s[sz];
            sz++;
        }
        
        if(strcmp(tmp, "SID") == 0)
        {
            return ProcType::SID;
        }
        if(strcmp(tmp, "STAR") == 0)
        {
            return ProcType::STAR;
        }
        if(strcmp(tmp, "APPCH") == 0)
        {
            return ProcType::APPROACH;
        }
        if(strcmp(tmp, "PRDAT") == 0)
        {
            return ProcType::PRDAT;
        }
        if(strcmp(tmp, "RWY") == 0)
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
            if(db_subsection == 'A')
            {
                airport_data_t apt_data;
                int ret = nav_db->get_airport_data(area_code, &apt_data);
                if(ret)
                {
                    return {area_code, {NavaidType::NAV_NONE, apt_data.pos, 
                        area_code, country_code}};
                }
            }
            lookup_area = area_code;
        }
        else if(db_section != 'D' && db_section != 'E' && db_section != 'P')
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

    arinc_str_t::arinc_str_t(std::vector<std::string>& in_split)
    {
        rt_type = in_split[1][0];
        
        main_fix.fix_ident = in_split[4];
        main_fix.country_code = in_split[5];
        main_fix.db_section = in_split[6][0];
        main_fix.db_subsection = in_split[7][0];
        wpt_desc = in_split[8];

        turn_dir = in_split[9][0];
        rnp = strutils::strip(in_split[10], ' ');
        leg_type = strutils::strip(in_split[11], ' ');
        tdv = in_split[12][0];

        recd_navaid.fix_ident = strutils::strip(in_split[13], ' ');
        recd_navaid.country_code = strutils::strip(in_split[14], ' ');
        recd_navaid.db_section = in_split[15][0];
        recd_navaid.db_subsection = in_split[16][0];

        arc_radius = strutils::stof_with_strip(in_split[17]);
        theta = strutils::stof_with_strip(in_split[18]);
        rho = strutils::stof_with_strip(in_split[19]);
        outbd_mag_crs = strutils::strip(in_split[20], ' ');
        outbd_dist_time = strutils::strip(in_split[21], ' ');

        alt_desc = in_split[22][0];
        alt1 = strutils::strip(in_split[23], ' ');
        alt2 = strutils::strip(in_split[24], ' ');
        trans_alt = strutils::stoi_with_strip(in_split[25]);

        speed_desc = in_split[26][0];
        spd_lim = strutils::stoi_with_strip(in_split[27]);
        vert_angle = strutils::stof_with_strip(in_split[28]) * 0.01;
        vert_scale = strutils::stoi_with_strip(in_split[29]);

        center_fix.fix_ident = strutils::strip(in_split[30], ' ');
        center_fix.country_code = strutils::strip(in_split[31], ' ');
        center_fix.db_section = in_split[32][0];
        center_fix.db_subsection = in_split[33][0];

        multi_cod = in_split[34][0];
        gnss_ind = in_split[35][0];
        rt_qual1 = in_split[36][0];
        rt_qual2 = in_split[37][0];
    }

    // arinc_leg_t definitions:

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

    // arinc_leg_full_t definitions:

    arinc_leg_full_t::arinc_leg_full_t(std::string& s, std::string& area_code, 
        std::shared_ptr<NavDB> nav_db)
    {        
        std::vector<std::string> s_split = strutils::str_split(s, ARINC_FIELD_SEP);

        if(s_split.size() == N_ARINC_FLT_PROC_COL)
        {
            proc_name = strutils::strip(s_split[2], ' ');
            trans_name = strutils::strip(s_split[3], ' ');

            arinc_str_t arnc_str(s_split);
            leg = arnc_str.get_leg(area_code, nav_db);

            err = DbErr::SUCCESS;
        }
        else
        {
            err = DbErr::DATA_BASE_ERROR;
        }
    }

    // arinc_rwy_full_t definitions:

    int arinc_rwy_full_t::get_pos_from_db(std::string& area_code, 
        std::shared_ptr<NavDB> nav_db)
    {
        runway_entry_t rnw;
        int ret = nav_db->get_rnw_data(area_code, id, &rnw);

        if(ret)
        {
            data.pos = rnw.start;
            data.thresh_displ_ft = rnw.displ_threshold_m * geo::M_TO_FT;
        }

        return ret;
    }

    void arinc_rwy_full_t::get_rwy_coords(std::string& s, std::string& area_code, 
        std::shared_ptr<NavDB> nav_db)
    {
        std::vector<std::string> second_part_splt = strutils::str_split(
            s, ARINC_FIELD_SEP);
        
        if(second_part_splt.size() == N_ARINC_RWY_COL_SECOND)
        {
            std::string lat_stripped = strutils::strip(second_part_splt[0], ' ');
            std::string lon_stripped = strutils::strip(second_part_splt[1], ' ');
            data.thresh_displ_ft = strutils::stoi_with_strip(second_part_splt[2]);

            data.pos.lat_deg = strutils::str_to_lat(lat_stripped);
            data.pos.lon_deg = strutils::str_to_lon(lon_stripped);

            if(data.pos.lat_deg == 0 || data.pos.lon_deg == 0)
            {
                if(!get_pos_from_db(area_code, nav_db))
                {
                    err = DbErr::DATA_BASE_ERROR;
                }
            }
        }
        else
        {
            if(!get_pos_from_db(area_code, nav_db))
            {
                err = DbErr::DATA_BASE_ERROR;
            }
        }
    }

    arinc_rwy_full_t::arinc_rwy_full_t(std::string& s, std::string& area_code, 
        std::shared_ptr<NavDB> nav_db)
    {
        /*
            Runway data is grouped into 2 parts separated by ';'. 
            The second(threshold location) part can be missing(e.g. HUEN).
            In that case we have to hope it's present in apt.dat.
        */
        err = DbErr::SUCCESS;

        std::vector<std::string> main_parts = strutils::str_split(s, ';');
        std::vector<std::string> first_part_splt = strutils::str_split(
            main_parts[0], ARINC_FIELD_SEP);
        std::vector<std::string> name_part = strutils::str_split(
            first_part_splt[0], ':');
        
        if(name_part.size() > 1 && first_part_splt.size() == N_ARINC_RWY_COL_FIRST)
        {
            std::string curr_id = strutils::strip(name_part[1], ' ');
            size_t id_length = curr_id.length();
            if(id_length > 2 && curr_id[0] == 'R' && 
                curr_id[1] == 'W' && isdigit(curr_id[2]))
            {
                id = strutils::normalize_rnw_id(curr_id.substr(2, id_length-2));
            }
            else
            {
                err = DbErr::DATA_BASE_ERROR;
                return;
            }

            data.grad_deg = strutils::stof_with_strip(first_part_splt[1]) * 0.001;
            data.ellips_height_m = strutils::stof_with_strip(first_part_splt[2]) * 0.1;
            data.thresh_elev_msl_ft = strutils::stoi_with_strip(first_part_splt[3]);

            data.tch_tp = char2tch_type(first_part_splt[4][0]);
            data.ls_ident = strutils::strip(first_part_splt[5], ' ');
            data.ls_cat = char2ls_category(first_part_splt[6][0]);
            data.tch_ft = strutils::stoi_with_strip(first_part_splt[7]);
        }
        else
        {
            err = DbErr::DATA_BASE_ERROR;
            return;
        }

        std::string pos_str = "";
        if(main_parts.size() == 2)
        {
            pos_str = main_parts[1];
        }

        get_rwy_coords(pos_str, area_code, nav_db);
    }
}; // namespace libnav
