#include <fstream>
#include <string>
#include <unordered_map>
#include <mutex>
#include <future>
#include "str_utils.hpp"
#include "common.hpp"


namespace libnav
{
    enum class HoldTurnDir
    {
        LEFT,
        RIGHT
    };

    constexpr int N_HOLD_COL_NORML = 11;


    struct hold_data_t
    {
        float inbd_crs_mag, leg_time_min, dme_leg_dist_nm;

        HoldTurnDir turn_dir;
        
        int min_alt_ft, max_alt_ft, spd_kts;
    };

    struct hold_line_t
    {
        earth_data_line_t data;

        std::string uid;
        hold_data_t hold_data;


        hold_line_t(std::string& s);
    };


    class HoldDB
    {
    public:
        HoldDB(std::string db_path);

        DbErr holds_loaded();

        int get_airac();

        bool get_hold_data(std::string wpt_id, hold_data_t* out);

        DbErr load_holds(std::string db_path);

    private:
        int airac_cycle;
        std::unordered_map<std::string, hold_data_t> hold_db;

        std::future<DbErr> hold_load_task;
    };
};
