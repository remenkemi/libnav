#include "libnav/awy_db.hpp"


namespace libnav
{
    NavaidType xp_awy_type_to_libnav(navaid_type_t type)
    {
        switch (type)
        {
        case XP_AWY_WPT:
            return NavaidType::WAYPOINT;
        case XP_AWY_NDB:
            return NavaidType::NDB;
        case XP_AWY_VHF:
            return NavaidType(static_cast<int>(NavaidType::DME) + 
                static_cast<int>(NavaidType::DME_ONLY) + 
                static_cast<int>(NavaidType::VOR) + 
                static_cast<int>(NavaidType::VOR_DME));
        default:
            return NavaidType::NONE;
        }
    }

    awy_line_t::awy_line_t(std::string& s)
    {
        std::vector<std::string> s_split = strutils::str_split(s);

        if(int(s_split.size()) == N_COL_AIRAC)
        {
            is_parsed = true;
            is_airac = true;
            airac_cycle = strutils::stoi_with_strip(s_split[AIRAC_CYCLE_WORD-1]);
        }
        else if(int(s_split.size()) == N_COL_NORML)
        {
            is_parsed = true;
            id_1 = s_split[0];
            reg_code_1 = s_split[1];
            tp_1 = uint16_t(strutils::stoi_with_strip(s_split[2]));
            id_2 = s_split[3];
            reg_code_2 = s_split[4];
            tp_2 = uint16_t(strutils::stoi_with_strip(s_split[5]));
            path_restr = s_split[6][0];

            lower_fl = uint32_t(strutils::stoi_with_strip(s_split[8]));
            upper_fl = uint32_t(strutils::stoi_with_strip(s_split[9]));
            awy_names = s_split[10];
        }
        else if(s_split.size() && s_split[0] == "99")
        {
            is_parsed = true;
            is_last = true;
        }
    }


    awy_point_t::awy_point_t(std::string nm, NavaidType tp, std::string r_c, 
        uint32_t lower, uint32_t upper)
    {
        id = nm;
        data.type = tp;
        data.reg_code = r_c;
        alt_restr.lower = lower;
        alt_restr.upper = upper;
    }

    // AwyDB member function definitions:
    // Public member functions:

    AwyDB::AwyDB(std::string awy_path)
    {
        load_airways(awy_path);
    }

    bool AwyDB::is_in_awy(std::string awy, std::string point)
    {
        if(awy_data.find(awy) != awy_data.end() && 
            awy_data[awy].find(point) != awy_data[awy].end())
        {
            return true;
        }
        return false;
    }

    int AwyDB::get_path(std::string awy, std::string start, 
            std::string end, std::vector<awy_point_t>* out)
    {
        if(is_in_awy(awy, start) && is_in_awy(awy, end))
        {
            std::unordered_map<std::string, std::string> prev;
            std::unordered_map<std::string, int> used;
            std::queue<std::string> q;
            std::vector<awy_point_t> out_rev;

            q.push(start);
            prev[start] = start;

            while(q.size())
            {
                std::string curr = q.front();
                q.pop();
                used[curr] = 1;

                if(curr == end)
                {
                    break;
                }

                for(auto it: awy_db[awy][curr])
                {
                    std::string tmp = it.first;
                    if(used.find(tmp) == used.end())
                    {
                        prev[tmp] = curr;
                        q.push(tmp);
                    }
                }
            }

            std::string curr = end;
            alt_restr_t r_past = awy_db[awy][prev[curr]][curr];
            while(prev[curr] != curr)
            {
                awy_point_t curr_wpt;
                curr_wpt.id = curr;
                curr_wpt.data = awy_data[awy][curr];
                curr_wpt.alt_restr = r_past;
                r_past = awy_db[awy][prev[curr]][curr];
                out_rev.push_back(curr_wpt);
                curr = prev[curr];
            }
            awy_point_t curr_wpt;
            curr_wpt.id = curr;
            curr_wpt.data = awy_data[awy][curr];
            curr_wpt.alt_restr = r_past;
            out_rev.push_back(curr_wpt);

            for(int i = out_rev.size() - 1; i > -1; i--)
            {
                out->push_back(out_rev[i]);
            }
        }

        return out->size();
    }

    AwyDB::~AwyDB()
    {

    }

    // Private member functions:

    DbErr AwyDB::load_airways(std::string awy_path)
    {
        std::ifstream file(awy_path);
		if (file.is_open())
		{
			std::string line;
            std::unordered_set<std::string> used;
			while (getline(file, line))
			{
                awy_line_t awy_line(line);

                if(!awy_line.is_last && awy_line.is_parsed && !awy_line.is_airac)
                {
                    std::string token = awy_line.id_1 + "_" + awy_line.id_2 
                        + "_" + awy_line.awy_names;
                    if(used.find(token) == used.end())
                    {
                        NavaidType type_1 = xp_awy_type_to_libnav(awy_line.tp_1);
                        NavaidType type_2 = xp_awy_type_to_libnav(awy_line.tp_2);

                        used.insert(token);
                        
                        awy_point_t p1(awy_line.id_1, type_1, 
                            awy_line.reg_code_1, awy_line.lower_fl, 
                            awy_line.upper_fl);
                        awy_point_t p2(awy_line.id_2, type_2, 
                            awy_line.reg_code_2, awy_line.lower_fl, 
                            awy_line.upper_fl);
                        add_to_awy_db(p1, p2, awy_line.awy_names, awy_line.path_restr);
                    }
                }
                else if(awy_line.is_last)
                {
                    break;
                }
            }

            file.close();
        }
        else
        {
            return DbErr::FILE_NOT_FOUND;
        }
        return DbErr::SUCCESS;
    }

    void AwyDB::add_to_awy_db(awy_point_t p1, awy_point_t p2, std::string awy_nm, char restr)
    {
        std::vector<std::string> awy_names = strutils::str_split(awy_nm, AWY_NAME_SEP);

        for(int i = 0; i < int(awy_names.size()); i++)
        {
            bool p1_found = awy_data[awy_names[i]].find(p1.id) != 
                awy_data[awy_names[i]].end();
            bool p2_found = awy_data[awy_names[i]].find(p2.id) != 
                awy_data[awy_names[i]].end();

            if(!p1_found)
            {
                awy_db[awy_names[i]][p1.id] = {};
                awy_data[awy_names[i]][p1.id] = p1.data;
            }
            if(!p2_found)
            {
                awy_db[awy_names[i]][p2.id] = {};
                awy_data[awy_names[i]][p2.id] = p2.data;
            }

            if(restr == AWY_RESTR_FWD || restr == AWY_RESTR_NONE)
            {
                awy_db[awy_names[i]][p1.id][p2.id] = p2.alt_restr;
            }
            if(restr == AWY_RESTR_BWD || restr == AWY_RESTR_NONE)
            {
                awy_db[awy_names[i]][p2.id][p1.id] = p1.alt_restr;
            }
        }
    }
}; // namespace libnav
