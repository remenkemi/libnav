#include "main_helpers.hpp"
#include <str_utils.hpp>


int main()
{
	dbg::Avionics avncs("apt.dat", "777_arpt.dat", "777_rnw.dat", "earth_fix.dat", 
		"earth_nav.dat", "earth_awy.dat");
	std::cout << "Avionics loaded\n";
	
	while(1)
	{
		std::string in_raw;
		std::getline(std::cin, in_raw);
		
		std::string in_proc = strutils::strip(in_raw, ' ');

		std::vector<std::string> line_split = strutils::str_split(in_proc, ' ');
		std::string cmd_name = line_split[0];
		std::vector<std::string> args = std::vector<std::string>(line_split.begin() + 1, 
			line_split.end());

		if(dbg::cmd_map.find(cmd_name) != dbg::cmd_map.end())
		{
			dbg::cmd_map[cmd_name](&avncs, args);
		}
		else
		{
			std::cout << "Invalid command name\n";
		}
	}
	
	return 0;
}
