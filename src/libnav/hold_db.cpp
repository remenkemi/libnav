#include "libnav/hold_db.hpp"


namespace libnav
{
    hold_line_t::hold_line_t(std::string& s)
    {
        data.is_airac = false;
        data.is_last = false;
        data.is_parsed = false;

        std::vector<std::string> s_split = strutils::str_split(s);

        if(int(s_split.size()) == N_COL_AIRAC)
        {
            data.is_parsed = true;
            data.is_airac = true;
            data.db_version = strutils::stoi_with_strip(s_split[0]);
            data.airac_cycle = strutils::stoi_with_strip(s_split[AIRAC_CYCLE_WORD-1]);
        }
        else if(int(s_split.size()) == N_HOLD_COL_NORML)
        {
            data.is_parsed = true;

            uid = s_split[0] + AUX_ID_SEP + s_split[1] + AUX_ID_SEP + s_split[2] 
                + AUX_ID_SEP + s_split[3];

            hold_data.inbd_crs_mag = strutils::stof_with_strip(s_split[4]);
            hold_data.leg_time_min = strutils::stof_with_strip(s_split[5]);
            hold_data.dme_leg_dist_nm = strutils::stof_with_strip(s_split[6]);
            if(s_split[7][0] == 'L')
            {
                hold_data.turn_dir = HoldTurnDir::LEFT;
            }
            else
            {
                hold_data.turn_dir = HoldTurnDir::RIGHT;
            }
            hold_data.min_alt_ft = strutils::stoi_with_strip(s_split[8]);
            hold_data.max_alt_ft = strutils::stoi_with_strip(s_split[9]);
            hold_data.spd_kts = strutils::stoi_with_strip(s_split[10]);
        }
        else if(s_split.size() && s_split[0] == "99")
        {
            data.is_parsed = true;
            data.is_last = true;
        }
    }

    // HoldDB member function definitions:
    // Public member functions:

    HoldDB::HoldDB(std::string db_path)
    {
        hold_load_task = std::async(std::launch::async, [](HoldDB* db, std::string db_path) -> 
				DbErr {return db->load_holds(db_path); }, this, db_path);
    }

    DbErr HoldDB::get_err()
    {
        return hold_load_task.get();
    }

    int HoldDB::get_airac()
    {
        return airac_cycle;
    }

    int HoldDB::get_db_version()
    {
        return db_version;
    }

    const hold_db_t& HoldDB::get_db()
    {
        return hold_db;
    }

    bool HoldDB::has_hold(std::string& wpt_id)
    {
        return hold_db.find(wpt_id) != hold_db.end();
    }

    std::vector<hold_data_t> HoldDB::get_hold_data(std::string& wpt_id)
    {
        if(hold_db.find(wpt_id) != hold_db.end())
        {
            return hold_db[wpt_id];
        }
        return {};
    }

    DbErr HoldDB::load_holds(std::string& db_path)
    {
        DbErr out_code = DbErr::SUCCESS;
        
        std::ifstream file(db_path);
		if (file.is_open())
		{
			std::string line;
            int i = 1;
			while (getline(file, line))
			{
                hold_line_t hold_line(line);
                if(!hold_line.data.is_parsed && i > N_EARTH_LINES_IGNORE)
                {
                    out_code = DbErr::PARTIAL_LOAD;
                }

                if(!hold_line.data.is_last && hold_line.data.is_parsed 
                    && !hold_line.data.is_airac)
                {
                    hold_db[hold_line.uid].push_back(hold_line.hold_data);
                }
                else if(hold_line.data.is_airac)
                {
                    airac_cycle = hold_line.data.airac_cycle;
                    db_version = hold_line.data.db_version;
                }
                else if(hold_line.data.is_last)
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
};
