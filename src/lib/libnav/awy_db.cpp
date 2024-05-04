#include "libnav/awy_db.hpp"


namespace libnav
{
    awy_point_t::awy_point_t(std::string nm, std::string tp, std::string r_c, 
        uint32_t lower, uint32_t upper)
    {
        id = nm;
        data.xp_type = tp;
        data.reg_code = r_c;
        alt_restr.lower = lower;
        alt_restr.upper = upper;
    }

    std::string awy_point_t::get_uid()
    {
        return id + "_" + data.reg_code + "_" + data.xp_type;
    }

    awy_line_t::awy_line_t(std::string& s)
    {
        data.is_parsed = false;
        data.is_airac = false;
        data.is_last = false;

        std::vector<std::string> s_split = strutils::str_split(s);

        if(int(s_split.size()) == N_COL_AIRAC)
        {
            data.is_parsed = true;
            data.is_airac = true;
            data.airac_cycle = strutils::stoi_with_strip(s_split[AIRAC_CYCLE_WORD-1]);
        }
        else if(int(s_split.size()) == N_AWY_COL_NORML)
        {
            data.is_parsed = true;

            std::string tp_1 = s_split[2];
            std::string id_1 = s_split[0];
            std::string reg_code_1 = s_split[1];
            
            
            std::string tp_2 = s_split[5];
            std::string id_2 = s_split[3];
            std::string reg_code_2 = s_split[4];
            
            path_restr = s_split[6][0];

            lower_fl = uint32_t(strutils::stoi_with_strip(s_split[8]));
            upper_fl = uint32_t(strutils::stoi_with_strip(s_split[9]));
            awy_names = s_split[10];

            p1 = awy_point_t(id_1, tp_1, reg_code_1, lower_fl, upper_fl);
            p2 = awy_point_t(id_2, tp_2, reg_code_2, lower_fl, upper_fl);
        }
        else if(s_split.size() && s_split[0] == "99")
        {
            data.is_parsed = true;
            data.is_last = true;
        }
    }

    // AwyDB member function definitions:
    // Public member functions:

    AwyDB::AwyDB(std::string awy_path)
    {
        db_loaded = std::async(std::launch::async, [](AwyDB* db, std::string awy_path) -> 
				DbErr {return db->load_airways(awy_path); }, this, awy_path);
    }

    DbErr AwyDB::get_err()
    {
        return db_loaded.get();
    }

    int AwyDB::get_airac()
    {
        return airac_cycle;
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

    DbErr AwyDB::load_airways(std::string awy_path)
    {
        DbErr out_code = DbErr::SUCCESS;
        
        std::ifstream file(awy_path);
		if (file.is_open())
		{
			std::string line;
            std::unordered_set<std::string> used;
            int i = 1;
			while (getline(file, line))
			{
                awy_line_t awy_line(line);
                if(!awy_line.data.is_parsed && i > N_AWY_LINES_IGNORE)
                {
                    out_code = DbErr::PARTIAL_LOAD;
                }

                if(!awy_line.data.is_last && awy_line.data.is_parsed 
                    && !awy_line.data.is_airac)
                {
                    std::string token = awy_line.p1.id + "_" + awy_line.p2.id 
                        + "_" + awy_line.awy_names;
                    if(used.find(token) == used.end())
                    {
                        

                        used.insert(token);
                        
                        add_to_awy_db(awy_line.p1, awy_line.p2, 
                            awy_line.awy_names, awy_line.path_restr);
                    }
                }
                else if(awy_line.data.is_airac)
                {
                    airac_cycle = awy_line.data.airac_cycle;
                }
                else if(awy_line.data.is_last)
                {
                    break;
                }

                i++;
            }

            file.close();
        }
        else
        {
            return DbErr::FILE_NOT_FOUND;
        }
        return out_code;
    }

    // Private member functions:

    void AwyDB::add_to_awy_db(awy_point_t p1, awy_point_t p2, std::string awy_nm, char restr)
    {
        std::vector<std::string> awy_names = strutils::str_split(awy_nm, AWY_NAME_SEP);

        std::string uid_1 = p1.get_uid();
        std::string uid_2 = p2.get_uid();

        for(int i = 0; i < int(awy_names.size()); i++)
        {
            bool p1_found = awy_data[awy_names[i]].find(uid_1) != 
                awy_data[awy_names[i]].end();
            bool p2_found = awy_data[awy_names[i]].find(uid_2) != 
                awy_data[awy_names[i]].end();

            if(!p1_found)
            {
                awy_db[awy_names[i]][uid_1] = {};
                awy_data[awy_names[i]][uid_1] = p1.data;
            }
            if(!p2_found)
            {
                awy_db[awy_names[i]][uid_2] = {};
                awy_data[awy_names[i]][uid_2] = p2.data;
            }

            if(restr == AWY_RESTR_FWD || restr == AWY_RESTR_NONE)
            {
                awy_db[awy_names[i]][uid_1][uid_2] = p2.alt_restr;
            }
            if(restr == AWY_RESTR_BWD || restr == AWY_RESTR_NONE)
            {
                awy_db[awy_names[i]][uid_2][uid_1] = p1.alt_restr;
            }
        }
    }
}; // namespace libnav
