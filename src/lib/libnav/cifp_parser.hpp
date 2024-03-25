#include <string>
#include <sstream>
#include <fstream>
#include <assert.h>
#include "nav_db.hpp"
#include "common.hpp"


namespace libnav
{
    enum class TurnDir
    {
        LEFT,
        RIGHT,
        EITHER
    };

    enum class AltMode
    {
        AT_OR_ABOVE,
        AT_OR_BELOW,
        AT,
        WITHIN,
        SID_AT_OR_ABOVE
    };

    enum class SpeedMode
    {
        AT_OR_ABOVE,
        AT_OR_BELOW,
        AT
    };

    
    TurnDir char2dir(char c);

    float str2rnp(std::string s);

    float str2outbd_crs(std::string s, bool* is_true);

    float str2outbd_dist(std::string s, bool* as_time);


    struct arinc_fix_entry_t
    {
        std::string fix_ident;  //Ref: arinc424 spec, section 5.13/5.23/5.144/5.271
        std::string area_code;  //Ref: arinc424 spec, section 5.14
        char db_section;  //D - navaid, E - enroute, P - airport. Ref: arinc424 spec, section 5.4
        char db_subsection;  //Ref: arinc424 spec, section 5.5


        waypoint_t to_waypoint_t(std::shared_ptr<NavaidDB> nav_db);
    };

    struct arinc_leg_t
    {
        char rt_type;  // Column 2. Ref: arinc424 spec, section 5.7

        waypoint_t main_fix;  // Columns 5-8
        std::string wpt_desc;  //Column 9. Ref: arinc424 spec, section 5.17
        
        TurnDir turn_dir;
        float rnp;
        std::string leg_type;  // Column 12: Ref: arinc424 spec, section 5.21
        bool is_ovfy;

        waypoint_t recd_navaid;  // Columns 14-17. Recommended navaid.

        float arc_radius;  // Column 18. Arc radius *1000 nm. Ref: arinc424 spec, section 5.204
        float theta;  // Column 19. magnetic bearing from recd_navaid to main_fix *10. Ref: arinc424 spec, section 5.24
        float rho;  // Column 20. geodesic distance from recd_navaid to main_fix *10. Ref: arinc424 spec, section 5.25
        bool outbd_crs_true;
        float outbd_crs_deg;  // Treated as magnetic if outbd_crs_true is false.
        bool outbd_dist_as_time;
        float outbd_dist_time;
        
        AltMode alt_desc;  // Column 23
        int alt1_ft;
        int alt2_ft;
        int trans_alt;  // Column 26. In feet MSL. Ref: arinc424 spec, section 5.53

        SpeedMode speed_desc;  // Column 27. Ref: arinc424 spec, section 5.261
        int spd_lim_kias;  // Column 28. Speed limit in KIAS. Ref: arinc424 spec, section 5.28
        float vert_angle_deg; 
        int vert_scale_ft;  // Column 30. Vertical deviation scale in feet. Ref: arinc424 spec, section 5.293

        waypoint_t center_fix;  // Column 31-34. Center fix for RF/AF leg.

        char multi_cod;  // Column 35 Multi-Code / TAA Center. Ref: arinc424 spec, section 5.130/5.272

        char gnss_ind;  // Column 36. Ref: arinc424 spec, section 5.222

        char rt_qual1;  // Column 37. Ref: arinc424 spec, section 5.7
        char rt_qual2;  // Column 38. Ref: arinc424 spec, section 5.7
    };

    struct arinc_str_t
    {
        char rt_type;  // Column 2. Ref: arinc424 spec, section 5.7
        std::string proc_name;  // Column 3. Ref: arinc424 spec, section 5.9 & 5.10
        std::string trans_name;  // Column 4. Ref: arinc424 spec, section 5.11

        arinc_fix_entry_t main_fix;  // Columns 5-8
        std::string wpt_desc;  //Column 9. Ref: arinc424 spec, section 5.17
        
        char turn_dir;  // Column 10: L/R/E. Ref: arinc424 spec, section 5.20
        std::string rnp;  // Column 11: 2 digits for number, 3rd digit for negative exponent. Ref: arinc424 spec, section 5.211
        std::string leg_type;  // Column 12: Ref: arinc424 spec, section 5.21
        char tdv;  // Column 13: Y if waypoint is "overfly". Ref: arinc424 spec, section 5.22

        arinc_fix_entry_t recd_navaid;  // Columns 14-17. Recommended navaid.

        float arc_radius;  // Column 18. Arc radius *1000 nm. Ref: arinc424 spec, section 5.204
        float theta;  // Column 19. magnetic bearing from recd_navaid to main_fix *10. Ref: arinc424 spec, section 5.24
        float rho;  // Column 20. geodesic distance from recd_navaid to main_fix *10. Ref: arinc424 spec, section 5.25
        std::string outbd_mag_crs;  // Column 21. Magnetic course from main_fix *10. If true, has T at the end. 
            // Ref: arinc424 spec, section 5.26
        std::string outbd_dist_time;  // Column 22. Distance in nm/time(min) from main_fix to next wpt. *10. 
            // T at the end if time. Ref: arinc424 spec, section 5.27
        
        char alt_desc;  // Column 23
        std::string alt1;  // Column 24. In feet or flight level. Ref: arinc424 spec, section 5.30
        std::string alt2;  // Column 25. In feet or flight level. Ref: arinc424 spec, section 5.30
        int trans_alt;  // Column 26. In feet MSL. Ref: arinc424 spec, section 5.53

        char speed_desc;  // Column 27. Ref: arinc424 spec, section 5.261
        int spd_lim;  // Column 28. Speed limit in KIAS. Ref: arinc424 spec, section 5.28
        float vert_angle;  // Column 29. Vertical angle in deg*100. Ref: arinc424 spec, section 5.70
        int vert_scale;  // Column 30. Vertical deviation scale in feet. Ref: arinc424 spec, section 5.293

        arinc_fix_entry_t center_fix;  // Column 31-34. Center fix for RF/AF leg.

        char multi_cod;  // Column 35 Multi-Code / TAA Center. Ref: arinc424 spec, section 5.130/5.272

        char gnss_ind;  // Column 36. Ref: arinc424 spec, section 5.222

        char rt_qual1;  // Column 37. Ref: arinc424 spec, section 5.7
        char rt_qual2;  // Column 38. Ref: arinc424 spec, section 5.7


        arinc_leg_t get_leg(std::shared_ptr<NavaidDB> nav_db);
    };


    typedef std::vector<arinc_leg_t> arinc_leg_seq_t;


    class Airport
    {
        typedef std::unordered_map<std::string, std::vector<int>> trans_db_t;
        typedef std::unordered_map<std::string, trans_db_t> proc_db_t;

    public:
        DbErr err_code;


        Airport(std::string icao, std::shared_ptr<NavaidDB> nav_db, 
            std::string cifp_path="");

    private:
        arinc_leg_seq_t arinc_legs;

        proc_db_t trans_per_proc_rwy;
        proc_db_t trans_per_rwy_proc;
        proc_db_t trans_per_proc_trans;


        DbErr load_db();
    };
}; // namespace libnav