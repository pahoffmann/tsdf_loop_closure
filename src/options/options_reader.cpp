#include <loop_closure/options/options_reader.h>

lc_options_reader::lc_options_reader()
{
}

lc_options_reader::~lc_options_reader()
{
}

int lc_options_reader::read_options(int argc, char **argv)
{   
    // just creates an options description
    create_options();

    try {
        vm = new po::variables_map();
        po::store(po::parse_command_line(argc, argv, *desc), *vm);

        if(vm->count("help")) {
            std::cout << *desc << std::endl;
            return 2;
        }
        else if(path_method < 0 || path_method > 2) {
            std::cerr << "Invalid path method! See program discription below" << std::endl;
            std::cout << *desc << std::endl;
            return 1;
        }

        po::notify(*vm);
    }
    catch(std::exception& e)
    {
        std::cerr << "Error while reading the command line parameters: " << e.what() << "\n";
        return 1;
    }
    catch(...)
    {
        std::cerr << "Unknown error while reading the command line paramerters!" << "\n";
        return 1;
    }

    // print the options in a beautiful way
    print_options();

    return 0;
}

void lc_options_reader::create_options() {
    // Declare the supported options.

    desc = new po::options_description("Allowed options");

    desc->add_options()
        ("help", "produce help message")
        ("hor_res", po::value<int>(&hor_res)->default_value(1024), "Set horizontal resolution")
        ("vert_res", po::value<int>(&vert_res)->default_value(128), "Set vertical resolution")
        ("opening_degree", po::value<int>(&opening_degree)->default_value(45), "Set raytracer opening degree")
        ("step_size", po::value<double>(&step_size)->default_value(0.032), "Set step size")
        ("ray_size", po::value<double>(&ray_size)->default_value(0.01), "Set ray size")
        ("map_file_name", po::value<std::string>(&map_file_name)->required(), "Set global map filename (.h5)")
        ("poses_file_name", po::value<std::string>(&poses_file_name)->default_value(""), "Set poses filename (.json)")
        ("base_file_name", po::value<std::string>(&base_file_name)->default_value(""), "Set base file name to store association data")
        ("path_method", po::value<int>(&path_method)->default_value(0), "Set path method [0 (Default)] = GlobalMap Path, [1] = Path Extraction, [2] = Use json path")
    ;
}

/**
 * @brief appends char 'c' n-times and returns string
 * 
 * @param n 
 * @param c 
 * @return std::string 
 */
std::string chars(int n, char c = ' ') {
    std::string blank_str = "";
    for(int i = 0; i < n; i++) {
        blank_str += c;
    }

    return blank_str;
}

void lc_options_reader::print_options() {

    // assuming the longest parameter value is one of the paths
    int max_length = std::max(std::max(map_file_name.length(), poses_file_name.length()), base_file_name.length());

    std::cout << std::endl;

    std::cout << chars(max_length + 21, '*') << std::endl;

    std::string cur_param = std::to_string((*vm)["hor_res"].as<int>());
    std::cout << "* Hor-Res:         " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = std::to_string((*vm)["vert_res"].as<int>());
    std::cout << "* Vert-Res:        " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = std::to_string((*vm)["opening_degree"].as<int>());
    std::cout << "* Opening-Degree:  " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = std::to_string((*vm)["step_size"].as<double>());
    std::cout << "* Step-Size:       " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = std::to_string((*vm)["ray_size"].as<double>());
    std::cout << "* Ray-Size:        " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["map_file_name"].as<std::string>();
    std::cout << "* Map-File-Name:   " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["poses_file_name"].as<std::string>();
    std::cout << "* Poses-File-Name: " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["base_file_name"].as<std::string>();
    std::cout << "* Base-File-Name:  " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = std::to_string((*vm)["path_method"].as<int>());
    std::cout << "* Path-Method:     " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    std::cout << chars(max_length + 21, '*') << std::endl;

    std::cout << std::endl;
}