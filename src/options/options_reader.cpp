#include "options_reader.h"

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
            std::cout << desc << std::endl;
            return 2;
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
        ("hor_res", po::value<int>(&hor_res)->default_value(16), "Set horizontal resolution")
        ("vert_res", po::value<int>(&vert_res)->default_value(16), "Set vertical resolution")
        ("side_length_xy", po::value<int>(&side_length_xy)->default_value(201), "Set sidelength xy")
        ("side_length_z", po::value<int>(&side_length_z)->default_value(95), "Set sidelength z")
        ("opening_degree", po::value<int>(&opening_degree)->default_value(16), "Set raytracer opening degree")
        ("step_size", po::value<double>(&step_size)->default_value(16), "Set step size")
        ("ray_size", po::value<double>(&ray_size)->default_value(16), "Set ray size")
        ("map_file_name", po::value<std::string>(&map_file_name)->required(), "Set global map filename (.h5)")
        ("poses_file_name", po::value<std::string>(&poses_file_name)->required(), "Set poses filename (.json)")
        ("base_file_name", po::value<std::string>(&base_file_name)->required(), "Set base file name to store association data")
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

    int max_length = std::max(std::max(map_file_name.length(), poses_file_name.length()), base_file_name.length());

    std::cout << "***********" << chars(max_length + 2, '*') << std::endl;

    std::string cur_param = (*vm)["hor_res"].as<std::string>();
    std::cout << "* Hor-Res:         " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["vert_res"].as<std::string>();
    std::cout << "* Vert-Res:        " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["side_length_xy"].as<std::string>();
    std::cout << "* Side-Length-XY:  " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["side_length_z"].as<std::string>();
    std::cout << "* Side-Length-Z:   " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["opening_degree"].as<std::string>();
    std::cout << "* Opening-Degree:  " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["step_size"].as<std::string>();
    std::cout << "* Step-Size:       " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["ray_size"].as<std::string>();
    std::cout << "* Ray-Size:        " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["map_file_name"].as<std::string>();
    std::cout << "* Map-File-Name:   " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["poses_file_name"].as<std::string>();
    std::cout << "* Poses-File-Name: " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    cur_param = (*vm)["base_file_name"].as<std::string>();
    std::cout << "* Base-File-Name:  " << cur_param << chars(max_length - cur_param.length()) << " *" << std::endl;

    std::cout << "***********" << chars(max_length + 2, '*') << std::endl;
}