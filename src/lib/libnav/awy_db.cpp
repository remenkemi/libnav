#include "libnav/awy_db.hpp"


namespace libnav
{
    NavaidType xp_awy_type_to_libnav(navaid_type_t type)
    {
        switch (type)
        {
        case XP_AWY_WPT:
            return NavaidType::NAV_WAYPOINT;
        case XP_AWY_NDB:
            return NavaidType::NAV_NDB;
        case XP_AWY_VHF:
            return NavaidType(static_cast<int>(NavaidType::NAV_DME) + 
                static_cast<int>(NavaidType::NAV_DME_ONLY) + 
                static_cast<int>(NavaidType::NAV_VOR) + 
                static_cast<int>(NavaidType::NAV_VOR_DME));
        default:
            return NavaidType::NAV_NONE;
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

    void AwyDB::load_airways(std::string awy_path)
    {
        std::ifstream file(awy_path);
		if (file.is_open())
		{
			std::string line;
			int i = 0;
            std::unordered_set<std::string> used;
			while (getline(file, line))
			{
				std::string check_val;
				std::stringstream s(line);
				s >> check_val;

                if(check_val != "99" && i >= N_AWY_LINES_IGNORE)
                {
                    std::string reg_code_1;
                    navaid_type_t tp_1;
                    std::string id_2;
                    std::string reg_code_2;
                    navaid_type_t tp_2;
                    char path_restr;
                    std::string junk;
                    uint32_t lower;
                    uint32_t upper;
                    std::string awy_names;

                    s >> reg_code_1 >> tp_1 >> id_2 >> reg_code_2 >> tp_2 >> path_restr >>
                        junk >> lower >> upper >> awy_names;

                    std::string token = check_val + "_" + id_2 + "_" + awy_names;
                    if(used.find(token) == used.end())
                    {
                        NavaidType type_1 = xp_awy_type_to_libnav(tp_1);
                        NavaidType type_2 = xp_awy_type_to_libnav(tp_2);

                        used.insert(token);
                        
                        awy_point_t p1(check_val, type_1, reg_code_1, lower, upper);
                        awy_point_t p2(id_2, type_2, reg_code_2, lower, upper);
                        add_to_awy_db(p1, p2, awy_names, path_restr);
                    }
                }
                else if(check_val == "99")
                {
                    break;
                }

                i++;
            }

            file.close();
        }
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
